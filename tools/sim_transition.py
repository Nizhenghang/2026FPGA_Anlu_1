#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reference model for the stage 4 transition effects.

Two pieces of RTL are involved and they live in different clock domains, so
this script models both and, more importantly, models the coupling between
them, which is where a mistake would actually hide.

  video_transition.v   video_clk, one decision per frame. Chooses the two
                       buffer selectors and the fade level.
  frame_fifo_read.v    ext_mem_clk, one burst at a time. Turns the two
                       selectors into read base addresses, and during a wipe
                       redirects the address once per frame at a boundary that
                       is an exact multiple of two lines.

Passes
  A  controller sequencing, frame granularity.
  B  frame_fifo_read regression. The stage 4 module and the pre-stage-4 module
     are run side by side on identical pseudo random stimulus and every
     observable register is compared every cycle. This is the guard on the one
     change that touches a module already verified on hardware.
  C  frame_fifo_read at the real geometry, one frame per selected boundary
     position, checking the buffer attribution of every single word.
  D  coupled run: the controller drives the selectors that the read model
     consumes, at a scaled down geometry so that hundreds of frames are cheap,
     verifying what the panel would actually show.
  E  parameter consistency, read back out of the RTL sources so the check
     cannot drift away from what is actually instantiated.

Exit code is 0 only if every gated check passed.
"""

import math
import os
import random
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RTL_ROOT = os.path.normpath(os.path.join(
    HERE, os.pardir, 'src', 'user_source', 'hdl_source'))

# ---------------------------------------------------------------------------
# frame_fifo_read, cycle accurate
# ---------------------------------------------------------------------------

S_IDLE, S_ACK, S_CHECK_FIFO, S_READ_BURST, S_READ_BURST_END, S_END = range(6)
STATE_NAMES = ['S_IDLE', 'S_ACK', 'S_CHECK_FIFO', 'S_READ_BURST',
               'S_READ_BURST_END', 'S_END']


class FrameFifoRead(object):
    """One mem_clk cycle per step(), non blocking assignment semantics.

    wipe=False reproduces the module exactly as it was before stage 4.
    wipe=True adds read_addr_index_top plus the group aligned redirect. When
    the two selectors are driven equal the streams must be identical, and that
    equivalence is what Pass B hammers on.
    """

    def __init__(self, read_addrs, read_len, burst_size=256, addr_bits=21,
                 burst_bits=9, fifo_depth=512, wipe=False,
                 wipe_grp_max=240, wipe_grp_step=8):
        self.read_addrs = tuple(read_addrs)
        self.read_len = read_len
        self.BURST_SIZE = burst_size
        self.addr_mask = (1 << addr_bits) - 1
        self.burst_mask = (1 << burst_bits) - 1
        self.FIFO_DEPTH = fifo_depth
        self.wipe = wipe
        self.WIPE_GRP_MAX = wipe_grp_max
        self.WIPE_GRP_STEP = wipe_grp_step
        # word offset within the frame at which the address was redirected,
        # recorded for reporting; -1 means no redirect happened this frame
        self.last_cross_word = -1
        self.words_this_frame = 0
        self.reset()

    def reset(self):
        self.read_req_d0 = 0
        self.read_req_d1 = 0
        self.read_req_d2 = 0
        self.read_len_d0 = 0
        self.read_len_d1 = 0
        self.read_len_latch = 0
        self.read_cnt = 0
        self.state = S_IDLE
        self.idx_d0 = 0
        self.idx_d1 = 0
        self.idx_top_d0 = 0
        self.idx_top_d1 = 0
        self.app_rd_addr_r = 0
        self.burst_cnt = 0
        self.rd_delay = 0
        self.app_rd_en_r = 0
        self.app_rd_en_d0 = 0
        self.fifo_aclr = 0
        self.read_req_ack = 0
        self.wipe_pos = 0
        self.grp_to_cross = 0
        self.burst_in_grp = 0
        self.wipe_delta = 0
        # Per frame recorders, not RTL registers. frame_grp / frame_top /
        # frame_bot are snapshotted at s_ack_first, so they say what the frame
        # about to be read is actually read with. grp_to_cross has counted down
        # to nothing by the time the frame ends, so reading it afterwards would
        # report the wrong boundary.
        self.frame_grp = 0
        self.frame_top = 0
        self.frame_bot = 0
        self.last_cross_word = -1
        self.words_this_frame = 0
        self._words_seen = 0

    # -- combinational, evaluated from the registers as they stand ------------
    def _comb(self):
        rd_vld = 1 if (self.state == S_READ_BURST
                       and self.burst_cnt >= self.BURST_SIZE) else 0
        rd_burst_finish = 1 if (rd_vld and self.rd_delay == 10) else 0
        app_rd_en = self.app_rd_en_d0
        base_bot = self.read_addrs[self.idx_d1]
        if self.wipe:
            base_top = self.read_addrs[self.idx_top_d1]
            sel_diff = 1 if self.idx_top_d1 != self.idx_d1 else 0
            start_base = base_top if sel_diff else base_bot
            s_ack_first = 1 if (self.state == S_ACK
                                and self.read_req_ack == 0) else 0
            grp_cross = 1 if (rd_burst_finish and self.burst_in_grp == 4
                              and self.grp_to_cross == 1) else 0
            # saturating one step of the ramp, mirrors wipe_pos_next
            if self.wipe_pos >= (self.WIPE_GRP_MAX - self.WIPE_GRP_STEP):
                pos_next = self.WIPE_GRP_MAX
            else:
                pos_next = self.wipe_pos + self.WIPE_GRP_STEP
        else:
            base_top = base_bot
            sel_diff = 0
            start_base = base_bot
            s_ack_first = 0
            grp_cross = 0
            pos_next = 0
        return (rd_vld, rd_burst_finish, app_rd_en, base_top, base_bot,
                sel_diff, start_base, s_ack_first, grp_cross, pos_next)

    def observable(self):
        """Everything Pass B compares. Deliberately exhaustive."""
        return (self.app_rd_addr_r, self.app_rd_en_d0, self.state,
                self.burst_cnt, self.read_cnt, self.rd_delay, self.app_rd_en_r,
                self.fifo_aclr, self.read_req_ack, self.read_len_latch,
                self.idx_d1)

    def wipe_regs(self):
        """The stage 4 registers that must stay at zero while the two selectors
        are driven equal. burst_in_grp is excluded on purpose: it free runs on
        every burst boundary regardless, and is harmless because grp_cross also
        needs grp_to_cross == 1, which never happens when the selectors agree.
        This is the other half of the Pass B statement: the new logic is inert,
        not merely coincidentally equal."""
        return (self.wipe_pos, self.grp_to_cross, self.wipe_delta)

    def step(self, read_req, idx, idx_top, wrusedw, app_wr_busy,
             sdr_init_done=1):
        (rd_vld, rd_burst_finish, app_rd_en, base_top, base_bot, sel_diff,
         start_base, s_ack_first, grp_cross, pos_next) = self._comb()

        # what the SDRAM sees on this cycle
        out_addr = self.app_rd_addr_r
        out_en = app_rd_en
        if out_en:
            self._words_seen += 1
            self.words_this_frame += 1
        if grp_cross:
            self.last_cross_word = self.words_this_frame

        # ---- block 1, the two beat synchronisers
        n_req_d0 = read_req
        n_req_d1 = self.read_req_d0
        n_req_d2 = self.read_req_d1
        n_len_d0 = self.read_len
        n_len_d1 = self.read_len_d0
        n_idx_d0 = idx
        n_idx_d1 = self.idx_d0
        n_idx_top_d0 = idx_top
        n_idx_top_d1 = self.idx_top_d0

        # ---- block 2, rd_delay
        if app_rd_en:
            n_rd_delay = 0
        elif self.rd_delay < 10:
            n_rd_delay = self.rd_delay + 1
        else:
            n_rd_delay = self.rd_delay

        # ---- block 3, burst counter, address, read enable
        if self.state == S_CHECK_FIFO:
            n_burst_cnt = 0
        elif app_rd_en:
            n_burst_cnt = (self.burst_cnt + 1) & self.burst_mask
        else:
            n_burst_cnt = self.burst_cnt

        if self.state == S_ACK:
            n_addr = start_base
        elif grp_cross:
            n_addr = (self.app_rd_addr_r + self.wipe_delta) & self.addr_mask
        elif app_rd_en:
            n_addr = (self.app_rd_addr_r + 1) & self.addr_mask
        else:
            n_addr = self.app_rd_addr_r

        n_en_d0 = 1 if (self.app_rd_en_r
                        and (self.burst_cnt + app_rd_en) < self.BURST_SIZE) else 0

        # ---- block 4, stage 4 wipe bookkeeping
        # Two registers carry the same number on purpose. wipe_pos accumulates
        # across frames and is written only here, on the first cycle of S_ACK.
        # grp_to_cross is loaded from the same value and then counts down
        # inside the frame, which turns the crossing test into a compare
        # against 1. Sharing one register was the bug this model caught: the in
        # frame countdown reaches zero long before the next frame read starts,
        # so the accumulated position is destroyed and the wipe restarts from
        # its first step every single frame.
        n_wipe_pos = self.wipe_pos
        n_grp_to_cross = self.grp_to_cross
        n_burst_in_grp = self.burst_in_grp
        n_wipe_delta = self.wipe_delta
        if self.wipe:
            if s_ack_first:
                n_burst_in_grp = 0
                n_wipe_delta = (base_bot - base_top) & self.addr_mask
                self.words_this_frame = 0
                self.last_cross_word = -1
                self.frame_grp = pos_next if sel_diff else 0
                self.frame_top = self.idx_top_d1
                self.frame_bot = self.idx_d1
                if sel_diff:
                    n_wipe_pos = pos_next
                    n_grp_to_cross = pos_next
                else:
                    n_wipe_pos = 0
                    n_grp_to_cross = 0
            elif rd_burst_finish:
                if self.burst_in_grp == 4:
                    n_burst_in_grp = 0
                    if self.grp_to_cross != 0:
                        n_grp_to_cross = self.grp_to_cross - 1
                else:
                    n_burst_in_grp = self.burst_in_grp + 1

        # ---- block 5, state machine
        n_state = self.state
        n_read_cnt = self.read_cnt
        n_len_latch = self.read_len_latch
        n_fifo_aclr = self.fifo_aclr
        n_ack = self.read_req_ack
        n_en_r = self.app_rd_en_r
        st = self.state
        if st == S_IDLE:
            if self.read_req_d2 == 1 and sdr_init_done:
                n_state = S_ACK
            n_ack = 0
        elif st == S_ACK:
            if self.read_req_d2 == 0:
                n_state = S_CHECK_FIFO
                n_fifo_aclr = 0
                n_ack = 0
            else:
                n_ack = 1
                n_fifo_aclr = 1
                n_len_latch = self.read_len_d1
            n_read_cnt = 0
        elif st == S_CHECK_FIFO:
            if self.read_req_d2 == 1:
                n_state = S_ACK
            elif (wrusedw < (self.FIFO_DEPTH - self.BURST_SIZE)
                  and not app_wr_busy):
                n_state = S_READ_BURST
                n_en_r = 1
        elif st == S_READ_BURST:
            if rd_burst_finish:
                n_en_r = 0
                n_state = S_READ_BURST_END
                n_read_cnt = self.read_cnt + self.BURST_SIZE
        elif st == S_READ_BURST_END:
            if self.read_req_d2 == 1:
                n_state = S_ACK
            elif self.read_cnt < self.read_len_latch:
                n_state = S_CHECK_FIFO
            else:
                n_state = S_END
        elif st == S_END:
            n_state = S_IDLE
        else:
            n_state = S_IDLE

        # ---- commit
        self.read_req_d0, self.read_req_d1, self.read_req_d2 = \
            n_req_d0, n_req_d1, n_req_d2
        self.read_len_d0, self.read_len_d1 = n_len_d0, n_len_d1
        self.idx_d0, self.idx_d1 = n_idx_d0, n_idx_d1
        self.idx_top_d0, self.idx_top_d1 = n_idx_top_d0, n_idx_top_d1
        self.rd_delay = n_rd_delay
        self.burst_cnt = n_burst_cnt
        self.app_rd_addr_r = n_addr
        self.app_rd_en_d0 = n_en_d0
        self.wipe_pos = n_wipe_pos
        self.grp_to_cross = n_grp_to_cross
        self.burst_in_grp = n_burst_in_grp
        self.wipe_delta = n_wipe_delta
        self.state = n_state
        self.read_cnt = n_read_cnt
        self.read_len_latch = n_len_latch
        self.fifo_aclr = n_fifo_aclr
        self.read_req_ack = n_ack
        self.app_rd_en_r = n_en_r

        return out_addr, out_en


# ---------------------------------------------------------------------------
# video_transition, clock accurate but only ever ticked on interesting edges
# ---------------------------------------------------------------------------

ST_IDLE, ST_FADE_OUT, ST_FADE_IN, ST_WIPE, ST_WIPE_END = range(5)
ST_NAMES = ['IDLE', 'FADE_OUT', 'FADE_IN', 'WIPE', 'WIPE_END']


class VideoTransition(object):
    """video_transition.v. tick() is one video_clk cycle."""

    def __init__(self, fade_max=8, wipe_hold=40, wipe_settle=2):
        self.FADE_MAX = fade_max
        self.WIPE_HOLD = wipe_hold
        self.WIPE_SETTLE = wipe_settle
        self.state = ST_IDLE
        self.cur_idx = 0
        self.tgt_idx = 0
        self.hold_cnt = 0
        self.mode_wipe = 0
        self.dv_d = 0
        self.bot_idx = 0
        self.top_idx = 0
        self.img_idx = 0
        self.fade_level = 0

    def tick(self, display_valid, disp_idx, frame_start):
        dv_rise = 1 if (display_valid and not self.dv_d) else 0
        pending = 1 if disp_idx != self.cur_idx else 0
        self.dv_d = display_valid          # dv_d <= I_display_valid, unconditional

        if not display_valid:
            self.state = ST_IDLE
            self.hold_cnt = 0
            self.fade_level = 0
            self.top_idx = self.bot_idx
        elif dv_rise:
            self.cur_idx = disp_idx
            self.tgt_idx = disp_idx
            self.bot_idx = disp_idx
            self.top_idx = disp_idx
            self.img_idx = disp_idx
            self.fade_level = 0
            self.hold_cnt = 0
            self.state = ST_FADE_IN
        elif frame_start:
            s = self.state
            if s == ST_IDLE:
                if pending:
                    self.tgt_idx = disp_idx
                    self.hold_cnt = 0
                    if self.mode_wipe:
                        self.top_idx = disp_idx
                        self.state = ST_WIPE
                    else:
                        self.state = ST_FADE_OUT
                    self.mode_wipe ^= 1
            elif s == ST_FADE_OUT:
                if self.fade_level <= 1:
                    self.fade_level = 0
                    self.cur_idx = self.tgt_idx
                    self.bot_idx = self.tgt_idx
                    self.top_idx = self.tgt_idx
                    self.img_idx = self.tgt_idx
                    self.state = ST_FADE_IN
                else:
                    self.fade_level -= 1
            elif s == ST_FADE_IN:
                if self.fade_level >= self.FADE_MAX:
                    self.fade_level = self.FADE_MAX
                    self.state = ST_IDLE
                else:
                    self.fade_level += 1
            elif s == ST_WIPE:
                if self.hold_cnt >= self.WIPE_HOLD - 1:
                    self.cur_idx = self.tgt_idx
                    self.bot_idx = self.tgt_idx
                    self.img_idx = self.tgt_idx
                    self.hold_cnt = 0
                    self.state = ST_WIPE_END
                else:
                    self.hold_cnt += 1
            elif s == ST_WIPE_END:
                if self.hold_cnt >= self.WIPE_SETTLE - 1:
                    self.hold_cnt = 0
                    self.state = ST_IDLE
                else:
                    self.hold_cnt += 1
            else:
                self.state = ST_IDLE


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

failures = 0


def fail(msg):
    global failures
    failures += 1
    print("    FAIL  " + msg)


def ok(msg):
    print("    ok    " + msg)


def run_one_frame(m, idx, idx_top, wrusedw_fn=None, app_wr_busy_fn=None,
                  req_hold=8):
    """Drive one complete frame read and return the emitted word addresses.

    read_req is held until read_req_ack, exactly like video_timing_data does,
    and the run then continues until the state machine comes back to S_IDLE.

    Returns the emitted word addresses, the cycle count, and the boundary /
    selectors the model snapshotted at s_ack_first, which is what this frame
    was actually read with.
    """
    addrs = []
    cyc = 0
    read_req = 1
    acked = 0
    while cyc < 4000000:
        wrusedw = 0 if wrusedw_fn is None else wrusedw_fn(cyc)
        abw = 0 if app_wr_busy_fn is None else app_wr_busy_fn(cyc)
        addr, en = m.step(read_req, idx, idx_top, wrusedw, abw)
        if en:
            addrs.append(addr)
        if m.read_req_ack:
            acked = 1
            read_req = 0
        if acked and m.state == S_IDLE and len(addrs) >= m.read_len:
            break
        cyc += 1
    else:
        raise RuntimeError("frame read did not complete")
    return addrs, cyc, m.frame_grp, m.frame_top, m.frame_bot


# ---------------------------------------------------------------------------
# Pass A -- controller sequencing
# ---------------------------------------------------------------------------

def pass_a():
    print("Pass A  video_transition sequencing, frame granularity")
    t = VideoTransition(fade_max=8, wipe_hold=36, wipe_settle=2)
    # a couple of clocks with display_valid low so dv_d is properly cleared
    for _ in range(3):
        t.tick(0, 0, 0)

    if t.fade_level != 0 or t.state != ST_IDLE:
        fail("after reset with display_valid low, expected black and IDLE")

    # display_valid rises: first picture must come up with a fade in, and the
    # selectors must already agree so no wipe is implied
    t.tick(1, 2, 0)
    if (t.bot_idx, t.top_idx, t.img_idx) != (2, 2, 2):
        fail("on display_valid rise the selectors must all follow disp_idx, "
             "got %d/%d/%d" % (t.bot_idx, t.top_idx, t.img_idx))
    if t.state != ST_FADE_IN or t.fade_level != 0:
        fail("on display_valid rise expected FADE_IN from level 0, got %s/%d"
             % (ST_NAMES[t.state], t.fade_level))

    levels = []
    for _ in range(20):
        t.tick(1, 2, 1)
        levels.append(t.fade_level)
    if levels[:9] != [1, 2, 3, 4, 5, 6, 7, 8, 8]:
        fail("power on fade in ramp is %s, expected 1..8 then held" % levels[:9])
    if t.state != ST_IDLE:
        fail("expected IDLE after the power on fade in, got %s"
             % ST_NAMES[t.state])
    ok("power on fade in ramps 0 -> 8 over 8 frames and lands in IDLE")

    # ---- first transition must be a fade, because mode_wipe resets to 0
    seq = []
    for f in range(60):
        t.tick(1, 3, 1)                 # sd_card_bmp has moved on to picture 3
        seq.append((f, ST_NAMES[t.state], t.fade_level, t.bot_idx, t.top_idx,
                    t.img_idx))
    fade = [r for r in seq if r[1] == 'FADE_OUT']
    if not fade:
        fail("the first transition should have been a fade, no FADE_OUT seen")
    black = [r for r in seq if r[2] == 0]
    if len(black) != 1:
        fail("expected exactly one black frame in a fade, got %d" % len(black))
    else:
        b = black[0]
        # the swap happens on the same frame_start that drives the level to 0
        if b[3] != 3 or b[4] != 3 or b[5] != 3:
            fail("on the black frame the selectors should already be the "
                 "target, got bot=%d top=%d img=%d" % (b[3], b[4], b[5]))
        ok("fade: dimmed 8 -> 1, exactly 1 black frame, handover on that black "
           "frame")
    settle = [r[0] for r in seq if r[1] == 'IDLE' and r[2] == 8]
    if not settle:
        fail("the fade never returned to IDLE at full level")
    else:
        ok("fade returns to IDLE at full level %d frame_starts after it began, "
           "%.2fs at 60Hz" % (settle[0], settle[0] / 60.0))

    # ---- second transition must be a wipe
    seq = []
    for f in range(60):
        t.tick(1, 0, 1)                 # on to picture 0
        seq.append((f, ST_NAMES[t.state], t.fade_level, t.bot_idx, t.top_idx,
                    t.img_idx))
    apart = [r for r in seq if r[3] != r[4]]
    if not apart:
        fail("the second transition should have been a wipe, the selectors "
             "never disagreed")
    else:
        # seq is recorded after the tick, so the frame_start that pulled
        # top_idx across is already in here: the count is the hold directly
        hold = len(apart)
        if hold != 36:
            fail("wipe held the selectors apart for %d frames, expected "
                 "WIPE_HOLD = 36" % hold)
        if apart[-1][2] != 8:
            fail("fade level must stay at %d during a wipe, saw %d"
                 % (8, apart[-1][2]))
        if apart[0][4] != 0 or apart[0][3] != 3:
            fail("during a wipe top must be the target (0) and bottom the "
                 "outgoing picture (3), got top=%d bot=%d"
                 % (apart[0][4], apart[0][3]))
        ok("wipe: selectors apart for %d frames, top=target bot=outgoing, "
           "fade held at full" % hold)
        end = [r[0] for r in seq if r[1] == 'IDLE']
        if not end:
            fail("the wipe never returned to IDLE")
        else:
            ok("wipe returns to IDLE %d frame_starts after it began, %.2fs at "
               "60Hz" % (end[0], end[0] / 60.0))

    # ---- a target that moves mid transition must not drag the goalposts
    t2 = VideoTransition(fade_max=4, wipe_hold=8, wipe_settle=2)
    for _ in range(3):
        t2.tick(0, 0, 0)
    t2.tick(1, 1, 0)
    for _ in range(8):
        t2.tick(1, 1, 1)
    seen = []
    targets = [2, 3, 3, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
    for d in targets:
        t2.tick(1, d, 1)
        seen.append((ST_NAMES[t2.state], t2.bot_idx, t2.top_idx))
    if any(a != b for (_s, a, b) in seen if _s == 'WIPE'):
        fail("top and bottom diverged from the latched target during a wipe")
    ok("a target that moves mid transition is latched, not chased")

    print("  Pass A done")


# ---------------------------------------------------------------------------
# Pass B -- frame_fifo_read regression against the pre stage 4 module
# ---------------------------------------------------------------------------

def pass_b(cycles=200000, seed=20260902):
    print("Pass B  frame_fifo_read, stage 4 vs pre stage 4, identical stimulus")
    rng = random.Random(seed)
    addrs = (0, 307200, 614400, 921600)
    # deliberately small geometry: this pass is about equivalence of the state
    # machine, not about the real frame, and a short frame means the random
    # stimulus reaches S_END and restarts many times over
    old = FrameFifoRead(addrs, read_len=64, burst_size=8, wipe=False)
    new = FrameFifoRead(addrs, read_len=64, burst_size=8, wipe=True,
                        wipe_grp_max=8, wipe_grp_step=2)

    req = 0
    idx = 0
    since = 0
    gap = 0
    mismatches = 0
    inert = 0
    frames = 0
    for c in range(cycles):
        # A read_req that behaves like video_timing_data: one per frame, held
        # until ack, and never re asserted while the previous frame read is
        # still running. Re asserting mid frame sends S_READ_BURST_END back to
        # S_ACK, which clears read_cnt, so the frame never completes and the
        # regression never reaches S_END.
        since += 1
        if req == 0 and since > gap and old.state == S_IDLE:
            req = 1
            since = 0
            gap = rng.randrange(0, 40)
        wrusedw = rng.choice([0, 0, 0, 16, 200, 300, 500])
        abw = rng.choice([0, 0, 0, 1])
        if rng.random() < 0.02:
            # the real index change lands about 100 mem_clk after read_req, so
            # mid frame is the normal case and both models must shrug it off
            idx = rng.randrange(4)

        ao, eo = old.step(req, idx, idx, wrusedw, abw)
        an, en = new.step(req, idx, idx, wrusedw, abw)
        if old.read_req_ack:
            req = 0
        if (ao, eo) != (an, en) or old.observable() != new.observable():
            mismatches += 1
            if mismatches <= 5:
                fail("cycle %d diverged: addr %d/%d en %d/%d state %s/%s"
                     % (c, ao, an, eo, en, STATE_NAMES[old.state],
                        STATE_NAMES[new.state]))
        wr = new.wipe_regs()
        if wr != (0, 0, 0):
            inert += 1
            if inert <= 3:
                fail("cycle %d: with the selectors driven equal the stage 4 "
                     "registers are %s, they must stay at zero" % (c, wr))
        if old.state == S_END and new.state == S_END:
            frames += 1
    if mismatches == 0:
        ok("%d cycles, %d frame reads completed, every observable register "
           "identical" % (cycles, frames))
    if frames < 200:
        fail("only %d frame reads completed, the stimulus is not exercising "
             "the state machine enough to be a meaningful regression" % frames)
    else:
        ok("%d complete frame reads, so S_END and the read_cnt wrap were both "
           "reached many times over" % frames)

    if inert == 0:
        ok("the stage 4 registers stayed at zero for all %d cycles, so the new "
           "logic is inert while the selectors agree rather than merely "
           "coincidentally equal" % cycles)
    print("  Pass B done")


# ---------------------------------------------------------------------------
# Pass C -- real geometry, buffer attribution of every word
# ---------------------------------------------------------------------------

REAL_ADDRS = (0, 307200, 614400, 921600)
REAL_LEN = 307200
REAL_LINE = 640
REAL_GROUPS = 240
REAL_STEP = 8
REAL_BURST = 256


def preload_for(m, grp):
    """Arrange for the next frame read to run with a boundary of grp.

    S_ACK computes wipe_pos_next = saturate(wipe_pos + step) and uses that, so
    the honest way to aim at a boundary is to load grp - step into wipe_pos and
    let the real RTL path do the rest. Writing grp straight into the crossing
    counter would bypass the very logic under test and would also be wrong, the
    way S_ACK steps it once more.
    """
    m.wipe_pos = max(0, grp - m.WIPE_GRP_STEP)
    m.grp_to_cross = 0
    m.burst_in_grp = 0


def check_frame(addrs, top_idx, bot_idx, grp, label):
    """Every word of a frame must come from the buffer its line belongs to."""
    if len(addrs) != REAL_LEN:
        fail("%s: emitted %d words, expected %d" % (label, len(addrs), REAL_LEN))
        return
    top_base = REAL_ADDRS[top_idx]
    bot_base = REAL_ADDRS[bot_idx]
    bad = 0
    first_bad = None
    for w, a in enumerate(addrs):
        g = w // (2 * REAL_LINE)
        want = (top_base if g < grp else bot_base) + w
        if a != want:
            bad += 1
            if first_bad is None:
                first_bad = (w, a, want, g)
    if bad:
        w, a, want, g = first_bad
        fail("%s: %d words wrong, first at word %d (group %d) addr %d "
             "expected %d" % (label, bad, w, g, a, want))
    else:
        # the boundary must land on an even line, i.e. on a whole group
        boundary = grp * 2 * REAL_LINE
        if grp not in (0, REAL_GROUPS) and boundary % (2 * REAL_LINE):
            fail("%s: boundary at word %d is not group aligned" % (label, boundary))


def pass_c():
    print("Pass C  frame_fifo_read at the real geometry, word by word")
    if REAL_LEN % REAL_BURST:
        fail("the frame is not a whole number of bursts")
    if (2 * REAL_LINE) % REAL_BURST:
        fail("a two line group is not a whole number of bursts")
    bursts = REAL_LEN // REAL_BURST
    per_group = (2 * REAL_LINE) // REAL_BURST
    ok("geometry: %d words, %d bursts of %d, %d bursts per two line group, "
       "%d groups per frame" % (REAL_LEN, bursts, REAL_BURST, per_group,
                                REAL_GROUPS))

    # boundary positions worth spending a full frame on: no wipe, the first
    # ramp step, the middle, one short of the end, and the saturated end
    cases = [(0, 1, 0, "no wipe, selectors equal"),
             (2, 0, 8, "first ramp step"),
             (3, 1, 120, "mid panel"),
             (0, 3, 232, "one step short of the bottom"),
             (1, 2, 240, "saturated, whole frame from the top buffer")]
    for top_idx, bot_idx, grp, label in cases:
        m = FrameFifoRead(REAL_ADDRS, REAL_LEN, burst_size=REAL_BURST,
                          wipe=True, wipe_grp_max=REAL_GROUPS,
                          wipe_grp_step=REAL_STEP)
        preload_for(m, grp)
        idx_top = top_idx if grp else bot_idx
        addrs, cyc, fgrp, ftop, fbot = run_one_frame(m, bot_idx, idx_top)
        if fgrp != grp or ftop != idx_top or fbot != bot_idx:
            fail("%s: frame was read with boundary %d top %d bot %d, expected "
                 "%d %d %d" % (label, fgrp, ftop, fbot, grp, idx_top, bot_idx))
        check_frame(addrs, ftop, fbot, fgrp, "%s (grp=%d)" % (label, grp))
        want_cross = grp * (2 * REAL_LINE) if grp else -1
        if m.last_cross_word != want_cross:
            fail("%s: redirect fired at word %d, expected %d"
                 % (label, m.last_cross_word, want_cross))
        if grp:
            ok("%-38s grp=%3d  boundary at line %3d  redirect at word %6d  "
               "%d cycles" % (label, grp, grp * 2, m.last_cross_word, cyc))
        else:
            ok("%-38s grp=%3d  whole frame from one buffer, no redirect  "
               "%d cycles" % (label, grp, cyc))

    # the redirect must fire at most once per frame, and the burst and enable
    # pattern must not depend on the boundary position at all
    shapes = []
    for grp in (0, 8, 120, 240):
        m = FrameFifoRead(REAL_ADDRS, REAL_LEN, burst_size=REAL_BURST,
                          wipe=True, wipe_grp_max=REAL_GROUPS,
                          wipe_grp_step=REAL_STEP)
        preload_for(m, grp)
        idx_top = 2 if grp else 1
        trace = []
        cyc = 0
        read_req = 1
        acked = 0
        while cyc < 4000000:
            addr, en = m.step(read_req, 1, idx_top, 0, 0)
            trace.append(en)
            if m.read_req_ack:
                acked = 1
                read_req = 0
            if acked and m.state == S_IDLE and sum(trace) >= REAL_LEN:
                break
            cyc += 1
        shapes.append((grp, tuple(trace), cyc))
    ref = shapes[0]
    for grp, tr, cyc in shapes[1:]:
        if tr != ref[1]:
            fail("the App_rd_en stream at grp=%d differs from grp=0, so the "
                 "wipe is changing the shape of the frame read" % grp)
        if cyc != ref[2]:
            fail("the frame at grp=%d took %d cycles against %d at grp=0"
                 % (grp, cyc, ref[2]))
    if failures == 0:
        ok("App_rd_en stream and frame duration are identical at every "
           "boundary position: the wipe changes only addresses")
    print("  Pass C done")


# ---------------------------------------------------------------------------
# Pass D -- coupled, scaled geometry so hundreds of frames are affordable
# ---------------------------------------------------------------------------

SC_LINE = 20            # words per line
SC_BURST = 8            # 5 bursts per two line group, same ratio as the real one
SC_LINES = 20           # 10 groups per frame
SC_GROUPS = SC_LINES // 2
SC_LEN = SC_LINE * SC_LINES
SC_ADDRS = (0, 10000, 20000, 30000)
SC_STEP = 1
SC_HOLD = SC_GROUPS + 4


def pass_d(frames=400):
    print("Pass D  coupled run, controller drives the read model")
    if (2 * SC_LINE) % SC_BURST:
        fail("scaled geometry is not burst aligned, the test proves nothing")
    t = VideoTransition(fade_max=3, wipe_hold=SC_HOLD, wipe_settle=2)
    m = FrameFifoRead(SC_ADDRS, SC_LEN, burst_size=SC_BURST, addr_bits=21,
                      burst_bits=9, wipe=True, wipe_grp_max=SC_GROUPS,
                      wipe_grp_step=SC_STEP)
    for _ in range(3):
        t.tick(0, 0, 0)

    # sd_card_bmp advances every AUTO frames, like the 1 second auto play tick
    AUTO = 22
    disp_idx = 0
    display_valid = 0
    panel = []
    bad = 0
    n_fade = 0
    n_wipe = 0
    for f in range(frames):
        if f == 4:
            display_valid = 1
            disp_idx = 0
        elif f > 4 and (f - 4) % AUTO == 0:
            disp_idx = (disp_idx + 1) % 4

        # The frame read happens on the vsync edge, which precedes
        # I_frame_start by video_delay's 20 taps, so it sees the selectors and
        # the fade level as they stood at the end of the previous frame.
        lvl = t.fade_level
        st_before = t.state
        addrs, _cyc, fgrp, ftop, fbot = run_one_frame(m, t.bot_idx, t.top_idx)
        t.tick(display_valid, disp_idx, 1)
        if st_before == ST_IDLE and t.state == ST_FADE_OUT:
            n_fade += 1
        elif st_before == ST_IDLE and t.state == ST_WIPE:
            n_wipe += 1

        # Attribute every word to a buffer using the boundary the read model
        # snapshotted at s_ack_first. a - w is the base the word came from, so
        # the set of those is the set of buffers this frame touched.
        bases = set()
        for w, a in enumerate(addrs):
            g = w // (2 * SC_LINE)
            want = ftop if (ftop != fbot and g < fgrp) else fbot
            bases.add(a - w)
            if a != SC_ADDRS[want] + w:
                bad += 1
                if bad <= 3:
                    fail("frame %d word %d addr %d expected %d (group %d, "
                         "boundary %d, top %d, bot %d)"
                         % (f, w, a, SC_ADDRS[want] + w, g, fgrp, ftop, fbot))
        panel.append((f, lvl, fbot, ftop, tuple(sorted(bases)), fgrp))

    if len(panel) != frames:
        fail("expected %d frames, got %d" % (frames, len(panel)))
    if bad == 0:
        ok("every word of all %d frames came from the buffer its line belongs "
           "to, given the boundary in force for that frame" % frames)

    # every frame must draw from at most two buffers
    torn = 0
    for (f, lvl, bot, top, bases, wg) in panel:
        if len(bases) > 2:
            torn += 1
            if torn <= 3:
                fail("frame %d drew from %d buffers" % (f, len(bases)))
    if torn == 0:
        ok("no frame ever drew from more than two buffers")

    # The wipe must sweep, not sit on one step. That stall is the exact
    # symptom of the shared register bug: the in frame countdown wiped the
    # accumulated position, so every frame restarted the ramp from zero and
    # the boundary never reached the bottom of the panel.
    runs = []
    cur = []
    for p in panel:
        if p[3] != p[2]:
            cur.append(p[5])
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    if not runs:
        fail("no wipe happened in %d frames" % frames)
    else:
        stuck = [r for r in runs if len(set(r)) < 2]
        if stuck:
            fail("%d of %d wipe run(s) never advanced past one boundary, e.g. "
                 "%s" % (len(stuck), len(runs), stuck[0]))
        else:
            ok("%d wipe runs, boundaries swept %s"
               % (len(runs), [sorted(set(r)) for r in runs][:2]))
        back = 0
        for r in runs:
            prev = None
            for g in r:
                if prev is not None and g < prev:
                    back += 1
                    if back <= 3:
                        fail("boundary went backwards, %d after %d" % (g, prev))
                prev = g
        if back == 0:
            ok("within a wipe the boundary only ever moved downwards")
        if panel[-1][3] == panel[-1][2] and len(runs) != n_wipe:
            fail("the controller started %d wipes but the read model only saw "
                 "%d runs of disagreed selectors" % (n_wipe, len(runs)))

    apart = [p for p in panel if p[3] != p[2]]
    sats = [p for p in apart if p[5] == SC_GROUPS]
    if not sats:
        fail("the wipe never saturated at WIPE_GRP_MAX = %d, so WIPE_HOLD = %d "
             "is too short for the ramp" % (SC_GROUPS, SC_HOLD))
    else:
        ok("%d frames had the selectors apart, %d of them at the whole panel "
           "boundary %d" % (len(apart), len(sats), SC_GROUPS))

    # after each wipe the boundary must come back to zero before the next one
    zeroes = [p[5] for p in panel if p[3] == p[2]]
    if set(zeroes) != {0}:
        fail("with the selectors equal the boundary should be 0, saw %s"
             % sorted(set(zeroes)))
    else:
        ok("the boundary returns to 0 on every frame the selectors agree")

    # brightness: a wipe runs at full level, and a fade is black for exactly
    # one frame, the handover frame
    dim_wipe = [p[0] for p in apart if p[1] != 3]
    if dim_wipe:
        fail("%d frames of a wipe were not at full brightness, e.g. %s"
             % (len(dim_wipe), dim_wipe[:5]))
    blacks = [p[0] for p in panel if p[1] == 0 and p[0] > 4]
    want_blacks = 1 + n_fade          # one for the power on fade in
    if len(blacks) != want_blacks:
        fail("%d black frames against %d fade transitions plus the power on "
             "fade in, expected %d" % (len(blacks), n_fade, want_blacks))
    else:
        ok("%d black frames for %d fades plus power on: one per handover, so "
           "the cut is never visible" % (len(blacks), n_fade))
    fades = [p for p in panel if p[1] not in (0, 3)]
    ok("%d transitions (%d fade, %d wipe), %d partially faded frames over %d "
       "frames" % (n_fade + n_wipe, n_fade, n_wipe, len(fades), frames))
    print("  Pass D done")


# ---------------------------------------------------------------------------
# Pass E -- parameter consistency, read back out of the RTL
# ---------------------------------------------------------------------------

def read_text(*rel):
    with open(os.path.join(RTL_ROOT, *rel), 'r', encoding='utf-8',
              errors='replace') as f:
        return f.read()


def find_param(text, name, default=None):
    m = re.search(r'\.%s\s*\(\s*(\d+)\'[dhb]?(\d+)\s*\)' % re.escape(name),
                  text)
    if m:
        return int(m.group(2))
    m = re.search(r'parameter\s+(?:\[\s*\d+\s*:\s*0\s*\]\s*)?%s\s*=\s*'
                  r'(?:\d+\'[dhb])?(\d+)' % re.escape(name), text)
    if m:
        return int(m.group(1))
    return default


def check_reset_polarity():
    """Lint every async reset in the HDL tree for a polarity mismatch.

    A behavioural model cannot catch this class of mistake, because it never
    looks at the sensitivity list. `always @(posedge clk or posedge rst)` with
    `if (!rst)` synthesises, simulates in a forgiving simulator, and powers up
    to zero on the FPGA, so it looks fine right up until the reset is actually
    asserted -- at which point the async reset never fires and the else branch
    is evaluated on the rising edge of rst instead, which synthesis reads as an
    asynchronous SET next to the asynchronous RESET. video_transition.v shipped
    with exactly that on its first pass and only the synthesis log said so.
    """
    blocks = 0
    bad = 0
    for dp, _dn, fn in os.walk(RTL_ROOT):
        for f in sorted(fn):
            if not f.endswith('.v'):
                continue
            path = os.path.join(dp, f)
            with open(path, 'r', encoding='utf-8', errors='replace') as fh:
                ls = fh.read().split('\n')
            for i, l in enumerate(ls):
                m = re.search(r'always\s*@\s*\(\s*posedge\s+(\w+)\s+or\s+'
                              r'(posedge|negedge)\s+(\w+)\s*\)', l)
                if not m:
                    continue
                blocks += 1
                edge, rst = m.group(2), m.group(3)
                for j in range(i + 1, min(i + 4, len(ls))):
                    c = re.search(r"if\s*\(\s*(!\s*)?" + re.escape(rst) +
                                  r"\s*(?:==\s*1'b1)?\s*\)", ls[j])
                    if not c:
                        continue
                    inv = bool(c.group(1))
                    # posedge rst wants an active high test, negedge rst_n an
                    # active low one; anything else is the mismatch
                    if (edge == 'posedge') == inv:
                        bad += 1
                        fail("%s:%d  %s  pairs with  %s"
                             % (os.path.relpath(path, RTL_ROOT), j + 1,
                                l.strip(), ls[j].strip()))
                    break
    if bad == 0:
        ok("reset polarity consistent across all %d async reset always blocks "
           "in the HDL tree" % blocks)
    if blocks < 50:
        fail("only %d async reset blocks found, the lint is not seeing the "
             "whole tree" % blocks)


def pass_e():
    print("Pass E  parameter consistency, parsed out of the RTL")
    top_src = read_text('top_tf_hdmi_audio.v')
    frw_src = read_text('SD', 'frame_read_write.v')
    ffr_src = read_text('SD', 'frame_fifo_read.v')

    hold = find_param(top_src, 'WIPE_HOLD')
    settle = find_param(top_src, 'WIPE_SETTLE')
    fade_max = find_param(top_src, 'FADE_MAX')
    gmax = find_param(frw_src, 'WIPE_GRP_MAX')
    gstep = find_param(frw_src, 'WIPE_GRP_STEP')
    fmax2 = find_param(ffr_src, 'WIPE_GRP_MAX')
    fstep2 = find_param(ffr_src, 'WIPE_GRP_STEP')

    if None in (hold, settle, fade_max, gmax, gstep):
        fail("could not parse the wipe parameters out of the RTL: hold=%s "
             "settle=%s fade=%s gmax=%s gstep=%s"
             % (hold, settle, fade_max, gmax, gstep))
        print("  Pass E done")
        return
    if (fmax2, fstep2) != (gmax, gstep):
        fail("frame_read_write forwards WIPE_GRP_MAX/STEP as %s/%s but its own "
             "defaults are %s/%s" % (gmax, gstep, fmax2, fstep2))

    ramp = int(math.ceil(float(gmax) / gstep))
    # the controller holds the selectors apart for WIPE_HOLD frame_starts, and
    # the first ramped frame is the one after that, so the ramp gets exactly
    # WIPE_HOLD frames. It needs ramp frames plus at least one of slack for the
    # frame offset between I_frame_start and the read request.
    need = ramp + 2
    if hold < need:
        fail("WIPE_HOLD = %d frames cannot cover a %d frame ramp plus the one "
             "frame offset between I_frame_start and read_req; needs >= %d"
             % (hold, ramp, need))
    else:
        ok("WIPE_HOLD %d >= ramp %d + 2, leaving %d saturated frames of guard"
           % (hold, ramp, hold - ramp))

    total = hold + settle
    auto_hz = 100_000_000
    frame_hz = 60
    if total / float(frame_hz) >= 1.0:
        fail("a wipe plus settle takes %d frames = %.2fs, which is not shorter "
             "than the 1s auto play interval in sd_card_bmp"
             % (total, total / float(frame_hz)))
    else:
        ok("wipe plus settle is %d frames = %.2fs at %dHz, inside the %.2fs "
           "auto play interval" % (total, total / float(frame_hz), frame_hz,
                                   auto_hz / float(auto_hz)))

    if gmax * 2 != 480:
        fail("WIPE_GRP_MAX %d does not cover a 480 line panel at 2 lines per "
             "group" % gmax)
    else:
        ok("WIPE_GRP_MAX %d x 2 lines = 480 lines, the whole panel" % gmax)
    if (2 * 640) % 256:
        fail("a two line group is not a whole number of 256 word bursts")
    else:
        ok("2 lines = 1280 words = %d bursts of 256, so the redirect never "
           "splits a burst" % (1280 // 256))
    if gmax % gstep:
        print("    note  WIPE_GRP_STEP %d does not divide WIPE_GRP_MAX %d "
              "exactly, the clamp in frame_fifo_read covers it"
              % (gstep, gmax))

    fade_frames = 2 * fade_max
    ok("fade is %d dimming frames + %d brightening = %d frames = %.2fs"
       % (fade_max, fade_max, fade_frames, fade_frames / 60.0))
    check_reset_polarity()
    print("  Pass E done")


def main():
    print("stage 4 transition reference model")
    print("=" * 72)
    pass_e()
    print()
    pass_a()
    print()
    pass_b()
    print()
    pass_c()
    print()
    pass_d()
    print()
    print("=" * 72)
    if failures:
        print("%d FAILURE(S)" % failures)
        return 1
    print("all passes clean")
    return 0


if __name__ == '__main__':
    sys.exit(main())
