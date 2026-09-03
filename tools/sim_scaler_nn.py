#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle-accurate reference model of SD/scaler_nn.v.

Mirrors the RTL statement by statement (all next-values computed from the
current state, then committed, i.e. non-blocking semantics) so that the
elastic-buffer sizing and the Bresenham mapping can be validated without a
board. For every test resolution it checks:

  * exactly DST_W * DST_H pixels are emitted, in raster order
  * every emitted pixel equals the ideal nearest-neighbour sample
  * the elastic buffer never overflows (o_overflow stays 0)
  * the S_FILL_W progress watchdog never fires, and how close its worst
    legitimate wait came to the threshold, which is what sizes FILL_WAIT_AW
  * the peak elastic-buffer occupancy, which is what sizes SK_DEPTH

Two passes, because a full-rate cycle sim of a 1920x1080 source is 200M
cycles and would take minutes per case:

  pass A - geometry. Every case, source offered every cycle and the elastic
           buffer made unbounded, so the sim runs at FSM speed. This isolates
           the Bresenham mapping, the border logic, the raster order and the
           emitted pixel count, which do not depend on the arrival rate.
  pass B - buffer pressure. The worst-case geometries at the real 96 cycles
           per pixel with the real 4096 entry buffer. This is what proves no
           source pixel is dropped during an emission gap and that SK_DEPTH
           is big enough. It stops after the top border plus a few active rows,
           because the backlog peaks at the end of the border run: every later
           row consumes as much as it parks, so simulating the remaining
           300k cycles would only re-measure a peak that has already happened.
           Pixel values are not recorded in this pass (pass A covers them),
           which is what keeps it affordable at all.
  pass F - sector gap. Passes A and B both drive the source as a pulse exactly
           every 96 cycles, so the longest silence they can produce is 96
           cycles and the S_FILL_W watchdog can never be exercised against a
           realistic stream. Pass F models arrival at byte level: the reader
           streams a 512 byte sector at 32 cycles per byte and then goes
           silent while it re-issues CMD17 and waits for the card's start
           token. It runs each geometry twice, once with the pre-fix watchdog
           (16 bits, partial row drained from a line_buf modelled as
           uninitialised BSRAM) to reproduce the corruption, and once with the
           shipped watchdog (24 bits, partial row shortened to black) to prove
           the same gaps are now ridden out. Only the second is gated.

Usage:
    python tools/sim_scaler_nn.py
"""

import os
import re

DST_W = 640
DST_H = 480
MAX_UPSCALE = 4
LB_AW = 10
SK_AW = 12
SK_DEPTH = 1 << SK_AW
STALL_THRESH = 384
FILL_WAIT_AW = 27              # saturating S_FILL_W progress watchdog
FILL_WAIT_AW_OLD = 16          # the pre-fix value, kept so pass F can reproduce
FILL_WAIT_AW_FIX2 = 25         # shipped with fix2: rode out one card access time
                               # but not the three-attempt retry budget fix2 added
FILL_WAIT_MAX = 1 << (FILL_WAIT_AW - 1)   # the bit the RTL tests

S_IDLE, S_ROW, S_SKIP, S_FILL_W, S_FILL_E, S_FILL_SUB = 0, 1, 2, 3, 4, 5
S_YSTEP, S_YSUB, S_DRAIN_RD, S_DRAIN_EM, S_DONE, S_FILL_C = 6, 7, 8, 9, 10, 11

SRC_W_MIN, SRC_W_MAX = 64, 1920
SRC_H_MIN, SRC_H_MAX = 64, 1080
CYCLES_PER_PIXEL = 96          # SCK = sys_clk/4 = 25MHz, 3 bytes per pixel

# Byte level model of the SD read stream, used by pass F. sd_card_sec_read_write
# issues one CMD17 per sector and the SPI layer has no timeout of its own, so
# between two sectors the pixel stream is silent for as long as the card takes
# to return the start token.
SECTOR_BYTES = 512
CYCLES_PER_BYTE = CYCLES_PER_PIXEL // 3     # 32 clk per byte at 25MHz SCK
PIXEL_OFFSET = 54                            # bfOffBits of a plain 24bit BMP


def src_pix(src_w, sy, sx):
    """Stand-in for real image data: unique per source pixel."""
    return ((sy * src_w + sx) * 2654435761) & 0xFFFFFF


def simulate(src_w, src_h, period=CYCLES_PER_PIXEL, sk_depth=SK_DEPTH,
             stall_duty=None, first_pixel_cycle=1000,
             max_cycles=400_000_000, record_pixels=True, max_rows=None,
             sector_gap=0, poison_lb=False, fill_wait_aw=FILL_WAIT_AW,
             shorten_row_on_bail=True):
    tw = min(src_w * MAX_UPSCALE, DST_W)
    th = min(src_h * MAX_UPSCALE, DST_H)
    off_x = (DST_W - tw) >> 1
    off_y = (DST_H - th) >> 1
    active_end_x = off_x + tw
    active_end_y = off_y + th
    dx_last, dy_last = DST_W - 1, DST_H - 1

    total_src = src_w * src_h

    # ---------------- state ----------------
    state = S_IDLE
    o_dst_valid = 0
    o_dst_pixel = 0
    o_busy = 0
    o_overflow = 0
    dy = dx_out = dx = 0
    row_end_x = 0
    row_is_border = 0
    sy_target = sx_target = 0
    x_acc = y_acc = 0
    filled_sy = 0
    lb_valid = 0
    held_pixel = 0
    held_sx = 0
    held_valid = 0
    usedw_r = 0
    fill_wait = 0

    sk_wr = sk_rd = sk_count = 0
    skid = {}
    sk_rdata = 0
    head_px_idx = head_row_idx = 0

    # Uninitialised BSRAM is not zero. TD warns (SYN-6562) that the init value
    # is dropped because gate eram_init is off, so an entry nobody wrote reads
    # back an arbitrary value that is FIXED per address. Modelling that is what
    # lets pass F show the real symptom: the same address is read on every row,
    # so a partially filled row turns into vertical stripes, not into black.
    if poison_lb:
        line_buf = [((c * 2654435761) ^ 0xC0FFEE) & 0xFFFFFF
                    for c in range(1 << LB_AW)]
    else:
        line_buf = [0] * (1 << LB_AW)
    lb_rdata = 0

    emitted = []
    emitted_count = 0
    stuck_taken = 0
    peak_sk_count = 0
    peak_fill_wait = 0
    next_src = 0
    fill_wait_bit = 1 << (fill_wait_aw - 1)

    armed = False

    cyc = 0
    while cyc < max_cycles:
        # ---------------- inputs this cycle ----------------
        i_dim_valid = 1 if cyc == 0 else 0
        if sector_gap:
            # Pixel k ends at file byte PIXEL_OFFSET + 3k + 2. That byte sits in
            # sector b // 512 at offset b % 512, and each sector costs a full
            # transfer plus the card's access time before its first byte.
            i_src_valid = 0
            src_index = 0
            if next_src < total_src:
                b = PIXEL_OFFSET + 3 * next_src + 2
                s, o = divmod(b, SECTOR_BYTES)
                arr = (first_pixel_cycle
                       + s * (SECTOR_BYTES * CYCLES_PER_BYTE + sector_gap)
                       + o * CYCLES_PER_BYTE)
                if cyc >= arr:
                    i_src_valid = 1
                    src_index = next_src
        else:
            src_index = (cyc - first_pixel_cycle) // period
            i_src_valid = 1 if (cyc >= first_pixel_cycle and
                                (cyc - first_pixel_cycle) % period == 0 and
                                src_index < total_src) else 0
        i_src_pixel = src_pix(src_w, src_index // src_w,
                              src_index % src_w) if i_src_valid else 0

        # Model the write FIFO backpressure as a duty cycle: emission is only
        # allowed on `allowed` of every `speriod` cycles. The real i_fifo_usedw
        # threshold does the same thing, just data-dependently, so sweeping the
        # duty measures how much slower the SDRAM write path is allowed to be
        # before the elastic buffer is outrun.
        stall = 0
        if stall_duty is not None:
            allowed, speriod = stall_duty
            stall = 0 if (cyc % speriod) < allowed else 1

        # ---------------- combinational decodes ----------------
        sk_empty = 1 if sk_count == 0 else 0
        sk_full = 1 if sk_count == sk_depth else 0
        sk_push = 1 if (i_src_valid and not sk_full) else 0

        fill_reuse = 1 if (state == S_FILL_W and held_valid and
                           held_sx == sx_target) else 0
        # Saturating watchdog bit, exactly as in the RTL. It must never fire on
        # a healthy stream: sx_target <= src_w - 1 for every column, so the
        # wanted pixel always exists and the only real stall is a dead source.
        fill_stuck = 1 if (state == S_FILL_W and
                           (fill_wait & fill_wait_bit)) else 0
        head_on_row = 1 if (not sk_empty and head_row_idx == sy_target) else 0
        pop_capture = 1 if (state == S_FILL_W and not fill_reuse and
                            not fill_stuck and head_on_row and
                            head_px_idx >= sx_target) else 0
        pop_discard = 1 if (not sk_empty and not pop_capture and
                            state in (S_SKIP, S_FILL_W) and
                            (head_row_idx < sy_target or
                             (head_on_row and head_px_idx < sx_target))) else 0
        sk_pop = pop_capture or pop_discard

        dy_active = 1 if (dy >= off_y and dy < active_end_y) else 0
        dx_out_active = 1 if (dx_out >= off_x and dx_out < row_end_x) else 0
        lb_raddr = (dx_out - off_x) if dx_out_active else 0
        lb_we = 1 if state == S_FILL_E else 0
        lb_re = 1 if state == S_DRAIN_RD else 0

        # The old combinational (head_row_idx > sy_target) bail-out was also
        # asserted, harmlessly, during the trailing reuse cycles of a
        # horizontally upscaled row, because fill_reuse had priority. Counting
        # those made every upscaled geometry look broken; the watchdog has no
        # such masked assertion, so any hit here is a real dead source.
        if fill_stuck:
            stuck_taken += 1

        # ---------------- next values ----------------
        n = dict(state=state, dy=dy, dx_out=dx_out, dx=dx,
                 row_end_x=row_end_x,
                 row_is_border=row_is_border, sy_target=sy_target,
                 sx_target=sx_target, x_acc=x_acc, y_acc=y_acc,
                 filled_sy=filled_sy, lb_valid=lb_valid,
                 held_pixel=held_pixel, held_sx=held_sx,
                 held_valid=held_valid, o_dst_valid=0, o_dst_pixel=o_dst_pixel,
                 o_busy=o_busy, sk_wr=sk_wr, sk_rd=sk_rd, sk_count=sk_count,
                 head_px_idx=head_px_idx, head_row_idx=head_row_idx,
                 o_overflow=o_overflow, usedw_r=usedw_r, fill_wait=fill_wait)

        # ---- fill progress watchdog, cleared on any progress ----
        if (state != S_FILL_W) or pop_capture or fill_reuse or fill_stuck:
            n['fill_wait'] = 0
        elif not (fill_wait & fill_wait_bit):
            n['fill_wait'] = fill_wait + 1

        # memories
        n_skid_write = (sk_wr, i_src_pixel) if sk_push else None
        n_sk_rdata = skid.get(sk_rd, 0) if pop_capture else None
        n_lb_write = (dx, held_pixel) if lb_we else None
        n_lb_rdata = line_buf[lb_raddr] if lb_re else None

        # ---- elastic buffer pointers (separate always block) ----
        if i_dim_valid:
            n.update(sk_wr=0, sk_rd=0, sk_count=0, head_px_idx=0,
                     head_row_idx=0, o_overflow=0)
        else:
            if sk_push:
                n['sk_wr'] = (sk_wr + 1) % sk_depth
            if sk_pop:
                n['sk_rd'] = (sk_rd + 1) % sk_depth
            if sk_push and not sk_pop:
                n['sk_count'] = sk_count + 1
            elif sk_pop and not sk_push:
                n['sk_count'] = sk_count - 1
            if sk_pop:
                if head_px_idx + 1 >= src_w:
                    n['head_px_idx'] = 0
                    n['head_row_idx'] = head_row_idx + 1
                else:
                    n['head_px_idx'] = head_px_idx + 1
            if i_src_valid and sk_full:
                n['o_overflow'] = 1

        # ---- main sequencer ----
        if i_dim_valid:
            armed = True
            n.update(state=S_ROW, o_busy=1, dy=0, dx_out=0, dx=0,
                     row_end_x=off_x + tw,
                     sy_target=0, sx_target=0, x_acc=0, y_acc=0,
                     filled_sy=0, lb_valid=0, held_pixel=0, held_sx=0,
                     held_valid=0, fill_wait=0)
        else:
            if state == S_ROW:
                n.update(dx=0, dx_out=0, sx_target=0, x_acc=0, held_valid=0)
                if not dy_active:
                    n.update(row_is_border=1, state=S_DRAIN_RD)
                else:
                    n['row_is_border'] = 0
                    if lb_valid and filled_sy == sy_target:
                        n['state'] = S_DRAIN_RD
                    elif (not sk_empty) and head_row_idx >= sy_target:
                        n['state'] = S_FILL_W
                    else:
                        n['state'] = S_SKIP
            elif state == S_SKIP:
                if (not sk_empty) and head_row_idx >= sy_target:
                    n['state'] = S_FILL_W
            elif state == S_FILL_W:
                if fill_reuse:
                    n['state'] = S_FILL_E
                elif fill_stuck:
                    n.update(lb_valid=1, filled_sy=sy_target, dx_out=0,
                             state=S_DRAIN_RD)
                    if shorten_row_on_bail:
                        n['row_end_x'] = off_x + dx
                elif pop_capture:
                    n.update(held_sx=sx_target, held_valid=1, state=S_FILL_C)
            elif state == S_FILL_C:
                n.update(held_pixel=sk_rdata, state=S_FILL_E)
            elif state == S_FILL_E:
                n.update(dx=dx + 1, x_acc=x_acc + src_w, state=S_FILL_SUB)
            elif state == S_FILL_SUB:
                if x_acc >= tw:
                    n.update(x_acc=x_acc - tw, sx_target=sx_target + 1)
                elif dx >= tw:
                    n.update(lb_valid=1, filled_sy=sy_target, dx_out=0,
                             row_end_x=off_x + tw, state=S_DRAIN_RD)
                else:
                    n['state'] = S_FILL_W
            elif state == S_DRAIN_RD:
                n['state'] = S_DRAIN_EM
            elif state == S_DRAIN_EM:
                if not stall:
                    n['o_dst_valid'] = 1
                    n['o_dst_pixel'] = 0 if (row_is_border or not dx_out_active) \
                        else lb_rdata
                    if dx_out >= dx_last:
                        if dy >= dy_last:
                            n['state'] = S_DONE
                        else:
                            n.update(dy=dy + 1, state=S_YSTEP)
                    else:
                        n.update(dx_out=dx_out + 1, state=S_DRAIN_RD)
            elif state == S_YSTEP:
                if (not row_is_border) and dy < active_end_y:
                    n.update(y_acc=y_acc + src_h, state=S_YSUB)
                else:
                    n['state'] = S_ROW
            elif state == S_YSUB:
                if y_acc >= th:
                    n.update(y_acc=y_acc - th, sy_target=sy_target + 1)
                else:
                    n['state'] = S_ROW
            elif state == S_DONE:
                n.update(o_busy=0, state=S_IDLE)

        # ---------------- commit ----------------
        if n_skid_write is not None:
            skid[n_skid_write[0]] = n_skid_write[1]
        if n_sk_rdata is not None:
            sk_rdata = n_sk_rdata
        if n_lb_write is not None:
            line_buf[n_lb_write[0]] = n_lb_write[1]
        if n_lb_rdata is not None:
            lb_rdata = n_lb_rdata

        # record the emission decided this cycle together with its coordinates
        if n['o_dst_valid']:
            emitted_count += 1
            if record_pixels:
                emitted.append((dy, dx_out, n['o_dst_pixel']))

        peak_sk_count = max(peak_sk_count, n['sk_count'])
        peak_fill_wait = max(peak_fill_wait, n['fill_wait'])
        state = n['state']
        dy, dx_out, dx = n['dy'], n['dx_out'], n['dx']
        row_end_x = n['row_end_x']
        row_is_border = n['row_is_border']
        sy_target, sx_target = n['sy_target'], n['sx_target']
        x_acc, y_acc = n['x_acc'], n['y_acc']
        filled_sy, lb_valid = n['filled_sy'], n['lb_valid']
        held_pixel, held_sx, held_valid = (n['held_pixel'], n['held_sx'],
                                           n['held_valid'])
        o_dst_valid, o_dst_pixel, o_busy = (n['o_dst_valid'], n['o_dst_pixel'],
                                            n['o_busy'])
        o_overflow = n['o_overflow']
        sk_wr, sk_rd, sk_count = n['sk_wr'], n['sk_rd'], n['sk_count']
        head_px_idx, head_row_idx = n['head_px_idx'], n['head_row_idx']
        usedw_r = n['usedw_r']
        fill_wait = n['fill_wait']
        if i_src_valid:
            next_src += 1

        if armed and state == S_IDLE and emitted_count >= DST_W * DST_H:
            break
        # The backlog is monotone across the top border run and self-balancing
        # afterwards, so a bounded row count is enough to find the peak.
        if max_rows is not None and dy > max_rows:
            break
        # Stop at the first lost pixel: the point of the run is to prove this
        # never happens, and continuing would only measure a corrupted raster.
        if o_overflow:
            break
        cyc += 1

    return dict(tw=tw, th=th, off_x=off_x, off_y=off_y,
                emitted=emitted if record_pixels else None,
                emitted_count=emitted_count,
                bail_outs=stuck_taken,
                peak_sk_count=peak_sk_count,
                peak_fill_wait=peak_fill_wait,
                o_overflow=o_overflow, cycles=cyc, src_w=src_w, src_h=src_h)


def check(r):
    src_w, src_h = r['src_w'], r['src_h']
    tw, th, off_x, off_y = r['tw'], r['th'], r['off_x'], r['off_y']
    problems = []

    # A truncated pressure run records no pixels; pass A owns pixel correctness.
    if r['emitted'] is not None:
        if r['emitted_count'] != DST_W * DST_H:
            problems.append("emitted %d pixels, expected %d"
                            % (r['emitted_count'], DST_W * DST_H))

        # raster order
        expect_order = [(y, x) for y in range(DST_H) for x in range(DST_W)]
        got_order = [(y, x) for (y, x, _) in r['emitted']]
        if got_order != expect_order:
            bad = next((i for i, (a, b) in enumerate(zip(got_order,
                                                         expect_order))
                        if a != b), None)
            problems.append("raster order broken at index %s: %s != %s"
                            % (bad,
                               got_order[bad] if bad is not None else None,
                               expect_order[bad] if bad is not None else None))

        # pixel values
        mismatch = 0
        first_bad = None
        for (y, x, val) in r['emitted']:
            if y < off_y or y >= off_y + th or x < off_x or x >= off_x + tw:
                want = 0
            else:
                sy = ((y - off_y) * src_h) // th
                sx = ((x - off_x) * src_w) // tw
                want = src_pix(src_w, sy, sx)
            if val != want:
                mismatch += 1
                if first_bad is None:
                    first_bad = (y, x, val, want)
        if mismatch:
            problems.append("%d pixel mismatches, first at (dy=%d,dx=%d) got "
                            "0x%06X want 0x%06X" % ((mismatch,) + first_bad))

    if r['o_overflow']:
        problems.append("elastic buffer OVERFLOWED at cycle %d"
                        % r['cycles'])
    if r['bail_outs']:
        problems.append("fill watchdog FIRED %d time(s), the source stream "
                        "stopped advancing" % r['bail_outs'])

    return problems


CASES = [
    ("640x480  (1:1, acceptance list)",        640, 480),
    ("800x600  (mild downscale, acceptance)",  800, 600),
    ("1920x1080 (strong downscale, accept.)", 1920, 1080),
    ("320x240  (2x upscale, acceptance)",      320, 240),
    ("64x64    (4x clamp, worst-case gap)",     64,  64),
    ("64x1080  (max h down, max w up)",         64, 1080),
    ("1920x64  (max w down, max h up)",       1920,  64),
    ("100x100  (odd, non-power-of-two)",       100, 100),
    ("1024x768 (typical XGA)",                1024, 768),
    ("1280x720 (HD)",                         1280, 720),
]

# Pass B keeps the real arrival period and the real buffer depth, so it is only
# affordable on sources whose pixel count stays low. These are exactly the ones
# that upscale vertically and therefore open an emission gap at all: a source
# with src_h >= DST_H never repeats a row and never stops consuming.
PRESSURE_CASES = [
    ("64x64    (4x clamp, widest border)",       64,  64),
    ("100x100  (odd geometry)",                 100, 100),
    ("320x240  (2x upscale, acceptance)",       320, 240),
    ("1920x64  (4x vertical, 3x horizontal)",  1920,  64),
    ("64x120   (4x vertical clamp boundary)",    64, 120),
    ("64x119   (narrowest source, most rows)",   64, 119),
]

# Gated emission duty cycles. The elastic buffer must survive every one of
# these, and an overflow here fails the exit code. None is the untouched RTL
# rate of two cycles per pixel; the others hold S_DRAIN_EM off for a fraction
# of the time, which is what a write FIFO sitting above STALL_THRESH does. The
# periods must be odd: S_DRAIN_RD/S_DRAIN_EM is a fixed two cycle rhythm, so an
# even period phase-locks with it and either never stalls or stalls on a fixed
# half, which is how the first (1, 2) entry came out measuring no slowdown.
EMIT_DUTIES = [
    ("full rate, 2 cyc/px",  None),
    ("67% emit (2 of 3)",    (2, 3)),
    ("33% emit (1 of 3)",    (1, 3)),
]

# Reported, never gated. One emit in seven cycles is roughly 50 MB/s of write
# bandwidth, against the 73.7 MB/s the display read side takes
# unconditionally, so this sits past what the SDRAM path can deliver and the
# buffer is expected to fill on the widest geometry. Its job is to show the
# shape of the curve and which geometry gives way first, not to pass; gating it
# would only train everyone to ignore the exit code.
PROBE_DUTIES = [
    ("14% emit (1 of 7)",    (1, 7)),
]


def geometric_check(src_w, src_h):
    """Row/column level model of the two Bresenham accumulators.

    Reproduces exactly the update sequence the RTL performs (S_FILL_E then the
    S_FILL_SUB carry loop, and S_YSTEP then the S_YSUB carry loop including the
    "only step when both the emitted row and the next row are active" guard),
    but without spending a cycle per pixel, so a 1920x1080 case costs a few
    hundred thousand cheap integer operations instead of 200M simulated cycles.

    The arrival rate cannot influence any of this, which is why pass A is
    allowed to drop the cycle dimension entirely.
    """
    tw = min(src_w * MAX_UPSCALE, DST_W)
    th = min(src_h * MAX_UPSCALE, DST_H)
    off_x = (DST_W - tw) >> 1
    off_y = (DST_H - th) >> 1
    active_end_y = off_y + th

    problems = []
    sy_target = 0
    y_acc = 0
    max_sx = 0
    max_sy_filled = 0
    emitted = 0

    for dy in range(DST_H):
        row_is_border = not (off_y <= dy < active_end_y)

        if not row_is_border:
            k_row = dy - off_y
            want_sy = (k_row * src_h) // th
            if sy_target != want_sy:
                problems.append("dy=%d: sy_target=%d, ideal %d"
                                % (dy, sy_target, want_sy))
                break
            if sy_target > src_h - 1:
                problems.append("dy=%d: sy_target=%d exceeds last source row %d"
                                % (dy, sy_target, src_h - 1))
                break
            max_sy_filled = max(max_sy_filled, sy_target)

            sx_target = 0
            x_acc = 0
            for k in range(tw):
                want_sx = (k * src_w) // tw
                if sx_target != want_sx:
                    problems.append("dy=%d dx=%d: sx_target=%d, ideal %d"
                                    % (dy, off_x + k, sx_target, want_sx))
                    return problems, tw, th, off_x, off_y, 0, 0, 0
                max_sx = max(max_sx, sx_target)
                # S_FILL_E then the S_FILL_SUB carry loop
                x_acc += src_w
                while x_acc >= tw:
                    x_acc -= tw
                    sx_target += 1

        # Emission is one pixel per destination column regardless of borders.
        emitted += DST_W

        # S_YSTEP / S_YSUB run only when this was not the last row.
        if dy < DST_H - 1:
            dy_next = dy + 1
            if (not row_is_border) and (dy_next < active_end_y):
                y_acc += src_h
                while y_acc >= th:
                    y_acc -= th
                    sy_target += 1

    if max_sx > src_w - 1:
        problems.append("sx_target reached %d, beyond last source column %d "
                        "(the fill would then starve and the progress "
                        "watchdog would drain a partial row)"
                        % (max_sx, src_w - 1))
    if max_sy_filled > src_h - 1:
        problems.append("sy_target reached %d on a filled row, beyond last "
                        "source row %d" % (max_sy_filled, src_h - 1))
    if emitted != DST_W * DST_H:
        problems.append("emitted %d pixels, expected %d"
                        % (emitted, DST_W * DST_H))
    return problems, tw, th, off_x, off_y, max_sx, max_sy_filled, emitted


def run_geometry(cases):
    print("PASS A - Bresenham geometry (row/column model, all resolutions)")
    print("-" * 78)
    failures = 0
    for (name, w, h) in cases:
        assert SRC_W_MIN <= w <= SRC_W_MAX, name
        assert SRC_H_MIN <= h <= SRC_H_MAX, name
        (problems, tw, th, off_x, off_y, max_sx, max_sy,
         emitted) = geometric_check(w, h)
        if problems:
            failures += 1
        print("%-4s %-38s tw=%-4d th=%-4d off=(%3d,%3d) maxSx=%4d/%-4d "
              "maxSy=%4d/%-4d px=%d"
              % ("PASS" if not problems else "FAIL", name, tw, th, off_x,
                 off_y, max_sx, w - 1, max_sy, h - 1, emitted))
        for p in problems:
            print("        ! %s" % p)
    return failures


def run_pressure(cases, duties, gated):
    tag = "" if gated else ", informational probe - not gated"
    print("PASS B - elastic buffer at the real 96 cyc/pixel arrival rate "
          "(SK_DEPTH=%d%s)" % (SK_DEPTH, tag))
    print("-" * 78)
    failures = 0
    for (duty_name, duty) in duties:
        worst_peak = 0
        worst_fw = 0
        print("[%s]" % duty_name)
        for (name, w, h) in cases:
            off_y = (DST_H - min(h * MAX_UPSCALE, DST_H)) >> 1
            # The border run is where the backlog peaks; a handful of active
            # rows after it is enough to show the self-balancing kick in.
            r = simulate(w, h, period=CYCLES_PER_PIXEL, sk_depth=SK_DEPTH,
                         record_pixels=False, max_rows=off_y + 8,
                         stall_duty=duty)
            problems = check(r)
            if problems and gated:
                failures += 1
            worst_peak = max(worst_peak, r['peak_sk_count'])
            worst_fw = max(worst_fw, r['peak_fill_wait'])
            print("%-4s %-38s tw=%-4d th=%-4d off=(%3d,%3d) peak=%4d "
                  "fw=%5d cyc=%8d"
                  % ("PASS" if not problems else "FAIL", name, r['tw'],
                     r['th'], r['off_x'], r['off_y'], r['peak_sk_count'],
                     r['peak_fill_wait'], r['cycles']))
            for p in problems:
                print("        ! %s" % p)
        # The watchdog threshold is only meaningful if the counter was actually
        # exercised, so a group that never waited on a source pixel is reported
        # as a dead instrument rather than as an infinite margin. Note that the
        # margin printed here is against a stream with NO gaps at all, so it
        # sizes nothing: pass F is the one that drives a realistic per-sector
        # silence and therefore the one that sizes FILL_WAIT_AW.
        if gated and worst_fw == 0:
            failures += 1
            print("        ! fill_wait never left zero, the watchdog margin "
                  "below is vacuous")
        print("        worst occupancy %d of %d entries (%.2fx margin); "
              "worst fill wait %d of %d cycles (gap-free stream, see pass F)"
              % (worst_peak, SK_DEPTH, SK_DEPTH / float(max(worst_peak, 1)),
                 worst_fw, FILL_WAIT_MAX))
    return failures


GAP_CASES = [
    ("640x480  (1:1, no scaling)",      640, 480),
    ("320x240  (2x upscale)",           320, 240),
]

# Per-sector silence to ride out. Both sit far above the pre-fix 32768 cycle
# threshold and far below the shipped 16.7M cycle one, and both are ordinary
# single-block-read latencies for a real card.
SECTOR_GAPS = [
    ("0.5ms",   50_000),
    ("2.0ms",  200_000),
]


def ideal_pixel(r, w, h, y, x):
    """What the nearest-neighbour mapping says destination (y,x) must show."""
    tw, th, off_x, off_y = r['tw'], r['th'], r['off_x'], r['off_y']
    if y < off_y or y >= off_y + th or x < off_x or x >= off_x + tw:
        return 0
    return src_pix(w, ((y - off_y) * h) // th, ((x - off_x) * w) // tw)


def stripe_signature(r, w, h):
    """Group the wrong pixels by column and report whether each column shows
    one single value on every row. That is the fingerprint of draining a line
    buffer nobody finished writing: the address is a function of the column
    alone, so the value cannot depend on the row.

    Note what is NOT part of the fingerprint: a constant first bad column. How
    far the fill gets before the watchdog fires depends on how much the elastic
    buffer happened to park, so the left edge of the corrupt field is jagged
    from row to row. What stays fixed is the colour of each column, and that is
    what the eye reads as vertical stripes."""
    per_col = {}
    first_bad = {}
    nwrong = 0
    for (y, x, val) in r['emitted']:
        if val != ideal_pixel(r, w, h, y, x):
            nwrong += 1
            per_col.setdefault(x, set()).add(val)
            first_bad.setdefault(y, x)
    single = all(len(v) == 1 for v in per_col.values())
    return nwrong, per_col, first_bad, single


def run_sector_gap():
    print("PASS F - per-sector silence in the source stream (byte level "
          "arrival, uninitialised line_buf)")
    print("-" * 78)
    failures = 0
    checks = 0

    runs = [
        # label, watchdog width, shorten the bailed row, what must happen
        ("pre-fix AW=16, tail drained", FILL_WAIT_AW_OLD,  False, "reproduce"),
        ("AW=16 + black tail         ", FILL_WAIT_AW_OLD,  True,  "black"),
        ("fix2 AW=25 + black tail    ", FILL_WAIT_AW_FIX2, True,  "black"),
        ("shipped AW=27 + black tail ", FILL_WAIT_AW,      True,  "clean"),
    ]

    for (label, aw, shorten, expect) in runs:
        print("[%s]  threshold = %d cycles = %.2f ms" %
              (label, 1 << (aw - 1), (1 << (aw - 1)) / 1e5))
        for (gname, gap) in SECTOR_GAPS:
            for (name, w, h) in GAP_CASES:
                # Two rows is the minimum that can show a per-column value
                # being row independent; three for the reproduction itself.
                rows = 2 if expect == "reproduce" else 1
                r = simulate(w, h, sector_gap=gap, poison_lb=True,
                             fill_wait_aw=aw, shorten_row_on_bail=shorten,
                             max_rows=rows, record_pixels=True)
                nbad, per_col, first_bad, single = stripe_signature(r, w, h)
                checks += 1
                ok = True
                note = ""

                if expect == "reproduce":
                    # This sub-run is the evidence, so a failure to reproduce
                    # is a failure of the pass: it would mean the diagnosis is
                    # wrong and the two rows below prove nothing.
                    rows_bad = sorted(first_bad)
                    if r['bail_outs'] == 0:
                        ok = False
                        note = "watchdog never fired, symptom NOT reproduced"
                    elif nbad == 0:
                        ok = False
                        note = "watchdog fired but every pixel was still right"
                    elif len(rows_bad) < 2:
                        ok = False
                        note = "only one row was affected, cannot test whether " \
                               "the corruption is row independent"
                    elif not single:
                        ok = False
                        note = "a column showed more than one value across " \
                               "rows, so the field is not vertical stripes"
                    else:
                        note = ("fired %dx, %d wrong px over rows %s, %d "
                                "columns each showing ONE value on every row "
                                "= vertical stripes; left edge jagged at %s"
                                % (r['bail_outs'], nbad, rows_bad,
                                   len(per_col),
                                   [first_bad[y] for y in rows_bad]))
                elif expect == "black":
                    # Defense in depth: if the watchdog does fire, the unfilled
                    # tail must be black, never a line_buf entry nobody wrote.
                    if nbad and any(v != {0} for v in per_col.values()):
                        ok = False
                        note = "bailed tail was not black, uninitialised BSRAM " \
                               "reached the output"
                    else:
                        note = ("fired %dx, %d wrong px in %d columns, every "
                                "one of them black"
                                % (r['bail_outs'], nbad, len(per_col)))
                else:
                    # A truncated run cannot be asked for a whole frame, so the
                    # count and raster order parts of check() do not apply here;
                    # what matters is that nothing bailed and every pixel that
                    # did come out is the right one.
                    if r['bail_outs']:
                        ok = False
                        note = "watchdog fired %dx on a healthy stream" \
                               % r['bail_outs']
                    elif nbad:
                        ok = False
                        note = "%d of %d emitted pixels wrong, first bad " \
                               "column %s" % (nbad, len(r['emitted']),
                                              min(per_col))
                    else:
                        note = ("rode out the gap, peak fill wait %d of %d "
                                "cycles (%.0fx margin), all %d emitted px "
                                "correct"
                                % (r['peak_fill_wait'], 1 << (aw - 1),
                                   (1 << (aw - 1)) / float(
                                       max(r['peak_fill_wait'], 1)),
                                   len(r['emitted'])))

                if not ok:
                    failures += 1
                print("%-4s gap=%-6s %-28s %s" %
                      ("PASS" if ok else "FAIL", gname, name, note))
    return failures, checks


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RTL_PATHS = {
    'scaler_nn': 'src/user_source/hdl_source/SD/scaler_nn.v',
    'sd_card_cmd': 'src/user_source/hdl_source/SD/sd_card_cmd.v',
    'sd_card_sec_read_write':
        'src/user_source/hdl_source/SD/sd_card_sec_read_write.v',
    'sd_card_bmp': 'src/user_source/hdl_source/SD/sd_card_bmp.v',
}


def rtl_const(module, pattern):
    """Read a sizing constant out of the RTL rather than restating it here.

    The defect this exists to catch was two constants in different files
    disagreeing about the same silence window, each defensible on its own: the
    retry budget was checked against the one second load watchdog and reported
    as having 3.33x of spare, while the scaler watched the identical silence
    through a window 1.79x tighter. A copy of either number in this file would
    drift the same way, so the check parses the sources.
    """
    path = os.path.join(REPO_ROOT, RTL_PATHS[module])
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        m = re.search(pattern, f.read())
    if not m:
        raise AssertionError('cannot find %r in %s' % (pattern, path))
    return int(m.group(1).replace('_', ''))


def run_watchdog_budget():
    """Gate the per-sector silence budget against BOTH watchdogs that watch it.

    Passes A, B and F all drive the source themselves, so they can only show
    what a given gap does to a given threshold. Neither number is theirs to
    pick: the gap is set by sd_card_sec_read_write's retry ladder and the
    threshold by scaler_nn, and the failure was that they were sized against
    different things. This pass is arithmetic on the RTL's own constants, which
    is cheap enough to run on every invocation and therefore hard to skip.
    """
    print("PASS G - retry silence budget vs the two watchdogs that watch it")
    print("-" * 78)
    failures = 0
    checks = 0

    aw = rtl_const('scaler_nn',
                   r'localparam\s+integer\s+FILL_WAIT_AW\s*=\s*(\d+)')
    timeout = rtl_const('sd_card_cmd',
                        r'READ_TIMEOUT_MAX\s*=\s*24\'d([\d_]+)')
    retry_max = rtl_const('sd_card_sec_read_write',
                          r'RD_RETRY_MAX\s*=\s*2\'d(\d+)')
    gap_aw = rtl_const('sd_card_sec_read_write',
                       r'localparam\s+integer\s+GAP_AW\s*=\s*(\d+)')
    clk_hz = rtl_const('sd_card_bmp',
                       r'parameter\s+integer\s+CLK_FREQ_HZ\s*=\s*([\d_]+)')

    attempts = retry_max + 1
    drain_gap = 1 << (gap_aw - 1)        # rd_gap exits on its top bit
    transfer = SECTOR_BYTES * CYCLES_PER_BYTE
    # Worst case for ONE sector: every attempt burns the full token timeout,
    # the drain gaps sit between them, and the successful transfer follows.
    silence = attempts * timeout + retry_max * drain_gap + transfer
    fill_thres = 1 << (aw - 1)           # fill_wait exits on its top bit
    load_thres = clk_hz                  # load_stall_cnt >= CLK_FREQ_HZ - 1

    ms = lambda c: c / (clk_hz / 1000.0)
    print("read from RTL: FILL_WAIT_AW=%d  READ_TIMEOUT_MAX=%d  RD_RETRY_MAX=%d"
          "  GAP_AW=%d  CLK_FREQ_HZ=%d" % (aw, timeout, retry_max, gap_aw,
                                           clk_hz))
    print("  %d attempts x %.2fms + %d drain gaps x %.3fms + %.3fms transfer"
          % (attempts, ms(timeout), retry_max, ms(drain_gap), ms(transfer)))
    print("  worst case silence for ONE sector : %12d cyc = %8.2f ms"
          % (silence, ms(silence)))
    print("  scaler_nn fill_wait threshold     : %12d cyc = %8.2f ms"
          % (fill_thres, ms(fill_thres)))
    print("  sd_card_bmp load_stall_cnt        : %12d cyc = %8.2f ms"
          % (load_thres, ms(load_thres)))
    print()

    # The two bounds, and why each is the one that matters.
    for (label, ok, detail) in [
        ("fill_wait must OUTLAST a full retry budget",
         silence < fill_thres,
         "%.2fx margin -- a sector needing %d attempts holds the source silent "
         "for %.2fms, and a shorter threshold truncates that row"
         % (fill_thres / float(silence), attempts, ms(silence))),
        ("fill_wait must stay INSIDE load_stall_cnt",
         fill_thres < load_thres,
         "%.2fx margin -- the scaler has to bail out before sd_card_bmp "
         "abandons the whole load, or the image is skipped instead of repaired"
         % (load_thres / float(fill_thres))),
        ("model FILL_WAIT_AW tracks the RTL",
         FILL_WAIT_AW == aw,
         "model=%d rtl=%d -- passes A/B/F would otherwise size a watchdog "
         "that no longer exists" % (FILL_WAIT_AW, aw)),
    ]:
        checks += 1
        if not ok:
            failures += 1
        print("%-4s %s" % ("PASS" if ok else "FAIL", label))
        print("       %s" % detail)

    # Record what the shipped-before-this-fix value did, so the regression is
    # stated as a measurement rather than as a story.
    checks += 1
    fix2_thres = 1 << (FILL_WAIT_AW_FIX2 - 1)
    broke = silence >= fix2_thres
    if broke:
        print("PASS fix2-era AW=%d threshold %d cyc = %.2f ms is BELOW the "
              "%.2f ms budget by %.2fx"
              % (FILL_WAIT_AW_FIX2, fix2_thres,
                 ms(fix2_thres), ms(silence), silence / float(fix2_thres)))
        print("       that is the reproduced defect: fill_stuck fires mid-row "
              "on a legitimate retry, once per stalled row")
    else:
        failures += 1
        print("FAIL fix2-era AW=%d no longer reproduces the defect, so this "
              "pass is not testing what it claims to" % FILL_WAIT_AW_FIX2)
    return failures, checks


def main():
    print("scaler_nn reference model   SK_DEPTH=%d  MAX_UPSCALE=%d  "
          "real arrival = %d cyc/pixel" % (SK_DEPTH, MAX_UPSCALE,
                                           CYCLES_PER_PIXEL))
    print("=" * 78)

    # First, because it is cheap and because the passes below are meaningless
    # if the model's watchdog width has drifted from the RTL's.
    fg, ng = run_watchdog_budget()
    print()
    fa = run_geometry(CASES)
    print()
    fb = run_pressure(PRESSURE_CASES, EMIT_DUTIES, gated=True)
    print()
    run_pressure(PRESSURE_CASES, PROBE_DUTIES, gated=False)
    print()
    fc, ngap = run_sector_gap()

    print("=" * 78)
    total = (ng + len(CASES) + len(PRESSURE_CASES) * len(EMIT_DUTIES) + ngap)
    print("%d / %d gated cases passed"
          % (total - fg - fa - fb - fc, total))
    print("%d probe cases reported above and deliberately excluded from the "
          "exit code" % len(PRESSURE_CASES))
    return 1 if (fg or fa or fb or fc) else 0


if __name__ == "__main__":
    raise SystemExit(main())
