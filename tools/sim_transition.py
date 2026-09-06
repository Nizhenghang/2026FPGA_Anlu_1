#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reference model for the stage 4 transition effects.

Two pieces of RTL are involved and they live in different clock domains, so
this script models both and, more importantly, models the coupling between
them, which is where a mistake would actually hide.

  video_transition.v   video_clk, one decision per frame. Chooses the two
                       buffer selectors, the 3-bit effect code and the fade
                       level. I_mode 000 auto-cycles the six band effects plus
                       fade, 001..110 force one effect, 111 forces fade.
  frame_fifo_read.v    ext_mem_clk, one burst at a time. Turns the two
                       selectors plus the effect code into read base addresses.
                       During a band effect it consults
                       select_top(group, progress, effect) at every two-line
                       group boundary and redirects the address by +/- the
                       buffer delta whenever the selection flips, so a whole
                       family of vertical sweeps (wipe down/up, blinds, split,
                       random bars, comb) shares the one proven redirect
                       mechanism. effect=1 reproduces the original single
                       crossing wipe bit for bit.

Passes
  A  controller sequencing, frame granularity.
  B  frame_fifo_read regression. The stage 4 module and the pre-stage-4 module
     are run side by side on identical pseudo random stimulus and every
     observable register is compared every cycle. This is the guard on the one
     change that touches a module already verified on hardware.
  C  frame_fifo_read at the real geometry, one frame per selected boundary
     position, checking the buffer attribution of every single word (effect=1,
     the legacy single crossing wipe).
  D  coupled run: the controller drives the selectors that the read model
     consumes, at a scaled down geometry so that hundreds of frames are cheap,
     verifying what the panel would actually show. Forced to wipe-down so it
     stays the hardware-verified single-boundary regression.
  E  parameter consistency, read back out of the RTL sources so the check
     cannot drift away from what is actually instantiated.
  F  DIP I_mode (3 bit) selects which effect the controller commits to: the
     auto-cycle rotation, one of the six forced band effects, or fade.
  G  band effect geometry: every effect across the whole ramp, checking the
     select_top pattern, full coverage at saturation, word-by-word buffer
     attribution through the FSM, and that redirects only ever fire on a group
     boundary. Carries a negative control on the saturation guard.

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
    wipe=True adds read_addr_index_top plus the group aligned redirect, now
    generalized from ONE crossing to a programmable per group buffer selection
    driven by `effect`. select_top(g, progress, effect) decides, for each two
    line group g, whether that group reads from the top buffer (1) or the bottom
    buffer (0); the address is redirected by +/- wipe_delta at every group
    boundary where the selection flips, which preserves the intra frame word
    offset exactly as the single crossing did. effect=1 (wipe down) reproduces
    the original single crossing wipe bit for bit, which is what Pass C and
    Pass D still rely on. When the two selectors are driven equal the band
    engine is inert (progress=g=cur_sel=next_sel_r=deltas all stay zero) and the
    stream is identical to wipe=False; that equivalence is what Pass B hammers.
    """

    def __init__(self, read_addrs, read_len, burst_size=256, addr_bits=21,
                 burst_bits=9, fifo_depth=512, wipe=False,
                 wipe_grp_max=240, wipe_grp_step=8, effect=1):
        self.read_addrs = tuple(read_addrs)
        self.read_len = read_len
        self.BURST_SIZE = burst_size
        self.addr_mask = (1 << addr_bits) - 1
        self.burst_mask = (1 << burst_bits) - 1
        self.FIFO_DEPTH = fifo_depth
        self.wipe = wipe
        self.WIPE_GRP_MAX = wipe_grp_max
        self.WIPE_GRP_STEP = wipe_grp_step
        # band geometry code, 1..6; 0 and 7 mean "no band redirect" and are
        # never passed to select_top. effect=1 is the legacy wipe down.
        self.effect = effect
        # word offset within the frame at which the address was redirected,
        # recorded for reporting; -1 means no redirect happened this frame
        self.last_cross_word = -1
        self.words_this_frame = 0
        self.reset()

    # -- band geometry: which buffer group g reads from ----------------------
    # Pure function of the group index, the per frame progress and the effect
    # code. Mirrors the select_top() Verilog function exactly, including the
    # progress >= grp_max saturation guard that reveals the last row of blinds
    # and the outermost group of split. Effects 1 and 2 scale with grp_max so
    # they also run at the reduced coupled geometry; effects 3..6 hardcode the
    # real 240 group panel constants (16 group slats, centre 120, bitrev8).
    @staticmethod
    def select_top(g, progress, eff, grp_max=240):
        if eff == 1:                                  # wipe down
            return 1 if g < progress else 0
        if eff == 2:                                  # wipe up
            return 1 if g >= (grp_max - progress) else 0
        if eff == 3:                                  # blinds, 15 slats of 16
            thresh = progress >> 4
            return 1 if ((g & 0xF) < thresh or progress >= grp_max) else 0
        if eff == 4:                                  # split from the centre
            half = grp_max >> 1
            diff = (g - half) if g > half else (half - g)
            return 1 if (diff < (progress >> 1) or progress >= grp_max) else 0
        if eff == 5:                                  # random bars, bitrev8
            rank = int(format(g & 0xFF, '08b')[::-1], 2)
            scaled = progress + (progress >> 3)
            return 1 if (rank < scaled or progress >= grp_max) else 0
        if eff == 6:                                  # comb, odd/even reversed
            if g & 1:
                return 1 if g >= (grp_max - progress) else 0
            return 1 if g < progress else 0
        return 0

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
        self.effect_d0 = 0
        self.effect_d1 = 0
        self.app_rd_addr_r = 0
        self.burst_cnt = 0
        self.rd_delay = 0
        self.app_rd_en_r = 0
        self.app_rd_en_d0 = 0
        self.fifo_aclr = 0
        self.read_req_ack = 0
        self.progress = 0
        self.g = 0
        self.g_plus1 = 1
        self.cur_sel = 0
        self.next_sel_r = 0
        self.burst_in_grp = 0
        self.wipe_delta = 0
        self.neg_wipe_delta = 0
        # Per frame recorders, not RTL registers. frame_grp / frame_top /
        # frame_bot / frame_effect are snapshotted at s_ack_first, so they say
        # what the frame about to be read is actually read with.
        self.frame_grp = 0
        self.frame_top = 0
        self.frame_bot = 0
        self.frame_effect = 0
        self.cross_words = []
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
            eff = self.effect_d1
            s_ack_first = 1 if (self.state == S_ACK
                                and self.read_req_ack == 0) else 0
            # saturating one step of the ramp, mirrors wipe_pos_next
            if self.progress >= (self.WIPE_GRP_MAX - self.WIPE_GRP_STEP):
                pos_next = self.WIPE_GRP_MAX
            else:
                pos_next = self.progress + self.WIPE_GRP_STEP
            # the first group's buffer for this frame. first_sel is computed from
            # pos_next (the value progress freezes to on the s_ack_first cycle)
            # and is what cur_sel latches there. The start base, however, is
            # derived from the REGISTERED cur_sel on every S_ACK cycle rather than
            # from first_sel directly, so select_top never enters the 21 bit
            # ext_mem_clk address register D path. That is safe: S_ACK lasts
            # several cycles and reloads the address on each one, cur_sel already
            # holds first_sel from the second cycle onward, and App_rd_en is low
            # throughout S_ACK, so the single stale load on the s_ack_first cycle
            # is never emitted. The first word read in S_READ_BURST therefore sees
            # the correct group-0 base. Recomputing from pos_next here would also
            # be wrong on the later S_ACK cycles, because progress has frozen and
            # pos_next has stepped on to the next frame's ramp value.
            first_sel = (self.select_top(0, pos_next, eff, self.WIPE_GRP_MAX)
                         if sel_diff else 0)
            start_sel = self.cur_sel
            start_base = base_top if start_sel else base_bot
            grp_boundary = 1 if (rd_burst_finish
                                 and self.burst_in_grp == 4) else 0
            # the redirect decision reads the REGISTERED next_sel_r, so the
            # select_top combinational depth never enters the address path
            do_redirect = 1 if (sel_diff and grp_boundary
                                and self.next_sel_r != self.cur_sel) else 0
            # g_plus1 is the REGISTERED g+1, mirroring the RTL flop that keeps the
            # leading incrementer off the ext_mem_clk path into next_sel_r. It lags
            # g by one cycle only in the single cycle after g advances at a group
            # boundary; next_sel_r is consumed only at the next boundary ~1280
            # cycles later, so the lag is never observed.
            next_sel_comb = self.select_top(self.g_plus1, self.progress, eff,
                                            self.WIPE_GRP_MAX)
        else:
            base_top = base_bot
            sel_diff = 0
            eff = 0
            s_ack_first = 0
            pos_next = 0
            first_sel = 0
            start_base = base_bot
            grp_boundary = 0
            do_redirect = 0
            next_sel_comb = 0
        return (rd_vld, rd_burst_finish, app_rd_en, base_top, base_bot,
                sel_diff, start_base, s_ack_first, do_redirect, pos_next,
                first_sel, grp_boundary, next_sel_comb)

    def observable(self):
        """Everything Pass B compares. Deliberately exhaustive."""
        return (self.app_rd_addr_r, self.app_rd_en_d0, self.state,
                self.burst_cnt, self.read_cnt, self.rd_delay, self.app_rd_en_r,
                self.fifo_aclr, self.read_req_ack, self.read_len_latch,
                self.idx_d1)

    def wipe_regs(self):
        """The band registers that must stay at zero while the two selectors are
        driven equal. burst_in_grp is excluded on purpose: it free runs on every
        burst boundary regardless, and is harmless because do_redirect is gated
        by sel_diff. g never advances when sel_diff is low, so g stays 0 and
        next_sel_r = select_top(1, 0, eff) = 0 for every effect. This is the
        other half of the Pass B statement: the new logic is inert, not merely
        coincidentally equal."""
        return (self.progress, self.g, self.cur_sel, self.next_sel_r,
                self.wipe_delta, self.neg_wipe_delta)

    def step(self, read_req, idx, idx_top, wrusedw, app_wr_busy,
             sdr_init_done=1, effect_in=None):
        if effect_in is None:
            effect_in = self.effect
        (rd_vld, rd_burst_finish, app_rd_en, base_top, base_bot, sel_diff,
         start_base, s_ack_first, do_redirect, pos_next, first_sel,
         grp_boundary, next_sel_comb) = self._comb()

        # what the SDRAM sees on this cycle
        out_addr = self.app_rd_addr_r
        out_en = app_rd_en
        if out_en:
            self._words_seen += 1
            self.words_this_frame += 1
        if do_redirect:
            self.last_cross_word = self.words_this_frame
            self.cross_words.append(self.words_this_frame)

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
        n_eff_d0 = effect_in
        n_eff_d1 = self.effect_d0

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
        elif do_redirect:
            # entering the top buffer subtracts (base_bot - base_top), entering
            # the bottom buffer adds it; neg_wipe_delta is precomputed so this is
            # a 2:1 mux, not a subtractor, on the ext_mem_clk address path
            delta = self.neg_wipe_delta if self.next_sel_r else self.wipe_delta
            n_addr = (self.app_rd_addr_r + delta) & self.addr_mask
        elif app_rd_en:
            n_addr = (self.app_rd_addr_r + 1) & self.addr_mask
        else:
            n_addr = self.app_rd_addr_r

        n_en_d0 = 1 if (self.app_rd_en_r
                        and (self.burst_cnt + app_rd_en) < self.BURST_SIZE) else 0

        # ---- block 4, band bookkeeping
        # progress accumulates across frames and is written only here, on the
        # first cycle of S_ACK, then frozen for the whole frame so every group
        # evaluates select_top against the same value (no intra frame tearing).
        # g counts groups within the frame; cur_sel is the buffer the current
        # group reads from and always equals select_top(g, progress, eff);
        # next_sel_r is select_top(g+1, ...) registered every cycle, so the
        # address mux only ever sees registered signals. Sharing one register
        # for progress and the in frame position was the bug the original model
        # caught; keeping progress separate from g/cur_sel avoids it.
        n_progress = self.progress
        n_g = self.g
        n_cur_sel = self.cur_sel
        n_burst_in_grp = self.burst_in_grp
        n_wipe_delta = self.wipe_delta
        n_neg_wipe_delta = self.neg_wipe_delta
        n_next_sel_r = next_sel_comb              # registered every cycle
        # g_plus1 <= g + 1, unconditional every cycle, read from the CURRENT g
        # (same clock-edge snapshot as n_g), mirroring the RTL flop.
        n_g_plus1 = self.g + 1
        if self.wipe:
            if s_ack_first:
                n_burst_in_grp = 0
                n_g = 0
                n_wipe_delta = (base_bot - base_top) & self.addr_mask
                n_neg_wipe_delta = (base_top - base_bot) & self.addr_mask
                self.words_this_frame = 0
                self.cross_words = []
                self.last_cross_word = -1
                self.frame_grp = pos_next if sel_diff else 0
                self.frame_top = self.idx_top_d1
                self.frame_bot = self.idx_d1
                self.frame_effect = self.effect_d1
                if sel_diff:
                    n_progress = pos_next
                    n_cur_sel = first_sel
                else:
                    n_progress = 0
                    n_cur_sel = 0
            elif rd_burst_finish:
                if self.burst_in_grp == 4:
                    n_burst_in_grp = 0
                    if sel_diff:
                        n_g = self.g + 1
                        n_cur_sel = self.next_sel_r
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
        self.effect_d0, self.effect_d1 = n_eff_d0, n_eff_d1
        self.rd_delay = n_rd_delay
        self.burst_cnt = n_burst_cnt
        self.app_rd_addr_r = n_addr
        self.app_rd_en_d0 = n_en_d0
        self.progress = n_progress
        self.g = n_g
        self.g_plus1 = n_g_plus1
        self.cur_sel = n_cur_sel
        self.next_sel_r = n_next_sel_r
        self.burst_in_grp = n_burst_in_grp
        self.wipe_delta = n_wipe_delta
        self.neg_wipe_delta = n_neg_wipe_delta
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

ST_IDLE, ST_FADE_OUT, ST_FADE_IN, ST_BAND, ST_WIPE_END = range(5)
ST_NAMES = ['IDLE', 'FADE_OUT', 'FADE_IN', 'BAND', 'WIPE_END']

# effect codes shared with frame_fifo_read.select_top. 0 and 7 are both "fade"
# (no band redirect); 1..6 are the horizontal band geometries.
EFF_NAMES = {0: 'fade', 1: 'wipe-down', 2: 'wipe-up', 3: 'blinds',
             4: 'split', 5: 'random-bars', 6: 'comb', 7: 'fade'}


class VideoTransition(object):
    """video_transition.v. tick() is one video_clk cycle.

    I_mode is now 3 bits: 000 auto-cycle (rotate effect_cnt through the six
    band effects plus fade, one per picture change), 001..110 force one band
    effect, 111 forces fade. The effect for a transition is sampled once, at
    the ST_IDLE/pending branch, so a mid-flight DIP change cannot tear it.
    """

    def __init__(self, fade_max=8, wipe_hold=40, wipe_settle=2):
        self.FADE_MAX = fade_max
        self.WIPE_HOLD = wipe_hold
        self.WIPE_SETTLE = wipe_settle
        self.state = ST_IDLE
        self.cur_idx = 0
        self.tgt_idx = 0
        self.hold_cnt = 0
        self.effect_cnt = 7        # auto-cycle counter; 7 makes the first a fade
        self.dv_d = 0
        self.bot_idx = 0
        self.top_idx = 0
        self.img_idx = 0
        self.fade_level = 0
        self.o_effect = 0

    def tick(self, display_valid, disp_idx, frame_start, mode=0):
        dv_rise = 1 if (display_valid and not self.dv_d) else 0
        pending = 1 if disp_idx != self.cur_idx else 0
        self.dv_d = display_valid          # dv_d <= I_display_valid, unconditional

        if not display_valid:
            self.state = ST_IDLE
            self.hold_cnt = 0
            self.fade_level = 0
            self.top_idx = self.bot_idx
            self.o_effect = 0
        elif dv_rise:
            self.cur_idx = disp_idx
            self.tgt_idx = disp_idx
            self.bot_idx = disp_idx
            self.top_idx = disp_idx
            self.img_idx = disp_idx
            self.fade_level = 0
            self.hold_cnt = 0
            self.o_effect = 0
            self.state = ST_FADE_IN
        elif frame_start:
            s = self.state
            if s == ST_IDLE:
                if pending:
                    self.tgt_idx = disp_idx
                    self.hold_cnt = 0
                    # chosen_effect mirrors the RTL wire: auto (000) rotates the
                    # free running counter, 111 forces fade, 001..110 force that
                    # band effect. It reads the OLD effect_cnt, matching non
                    # blocking semantics, then the counter advances.
                    if mode == 0b000:
                        chosen = self.effect_cnt
                    elif mode == 0b111:
                        chosen = 7
                    else:
                        chosen = mode
                    use_band = 1 if (chosen != 0 and chosen != 7) else 0
                    if use_band:
                        self.o_effect = chosen
                        self.top_idx = disp_idx
                        self.state = ST_BAND
                    else:
                        self.o_effect = 0
                        self.state = ST_FADE_OUT
                    # auto-cycle counter keeps running in every mode, 1..7
                    self.effect_cnt = 1 if self.effect_cnt >= 7 \
                        else self.effect_cnt + 1
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
            elif s == ST_BAND:
                if self.hold_cnt >= self.WIPE_HOLD - 1:
                    self.cur_idx = self.tgt_idx
                    self.bot_idx = self.tgt_idx
                    self.img_idx = self.tgt_idx
                    self.hold_cnt = 0
                    self.o_effect = 0
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

    # ---- first transition must be a fade, because effect_cnt resets to 7 and
    # auto (mode 000) reads it before advancing, so the first choice is fade
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
    if any(a != b for (_s, a, b) in seen if _s == 'BAND'):
        fail("top and bottom diverged from the latched target during a band effect")
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
        if wr != (0, 0, 0, 0, 0, 0):
            inert += 1
            if inert <= 3:
                fail("cycle %d: with the selectors driven equal the band "
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

    s_ack_first computes pos_next = saturate(progress + step) and freezes it
    into progress for the whole frame, so the honest way to aim at a boundary is
    to load grp - step into progress and let the real RTL path do the rest.
    Writing grp straight into progress would bypass the very ramp logic under
    test and would also be wrong, the way s_ack_first steps it once more. g,
    cur_sel and next_sel_r are all re-derived at s_ack_first, so pre-zeroing them
    here only keeps the model honest between back-to-back frames.
    """
    m.progress = max(0, grp - m.WIPE_GRP_STEP)
    m.g = 0
    m.cur_sel = 0
    m.next_sel_r = 0
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
                      wipe_grp_step=SC_STEP, effect=1)
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
        # Forced to wipe-down (mode 001) so this stays the single-boundary
        # regression the hardware was verified against: the read model runs
        # effect=1 and the attribution below assumes exactly one top/bottom
        # split. The multi-effect band geometry is Pass G's job.
        t.tick(display_valid, disp_idx, 1, mode=0b001)
        if st_before == ST_IDLE and t.state == ST_FADE_OUT:
            n_fade += 1
        elif st_before == ST_IDLE and t.state == ST_BAND:
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


def _drive_transition(t, new_idx, mode):
    """Advance the controller to new_idx and run one full transition.

    Assumes t is in ST_IDLE with display_valid already asserted and dv_d=1, so
    no dv_rise re-init fires. Ticks one frame at a time (frame_start=1), records
    the effect CODE the controller committed to when it left ST_IDLE, then keeps
    ticking until it settles back in ST_IDLE having reached new_idx. Returns the
    code (1..6 band, 7 fade), or None if it never committed (caller treats None
    as a fail). The band code is read off o_effect on the very tick that enters
    ST_BAND, before ST_BAND completion clears it back to 0.
    """
    effect = None
    guard = 4 * (t.FADE_MAX + t.WIPE_HOLD + t.WIPE_SETTLE) + 64
    for _ in range(guard):
        st_before = t.state
        t.tick(1, new_idx, 1, mode=mode)
        if st_before == ST_IDLE and t.state == ST_BAND:
            effect = t.o_effect            # 1..6
        elif st_before == ST_IDLE and t.state == ST_FADE_OUT:
            effect = 7                     # fade
        if effect is not None and t.state == ST_IDLE and t.cur_idx == new_idx:
            return effect
    return effect


def _effect_sequence(mode, n_transitions, cls=VideoTransition):
    """Return the list of effect codes for n picture changes under a fixed
    I_mode. Small counters keep it fast; the effect choice is decided once per
    transition and is independent of FADE_MAX/WIPE_HOLD/WIPE_SETTLE."""
    t = cls(fade_max=3, wipe_hold=6, wipe_settle=2)
    for _ in range(3):
        t.tick(0, 0, 0, mode=mode)          # display_valid low, dv_d clears
    t.tick(1, 0, 1, mode=mode)              # dv_rise commits idx 0 -> ST_FADE_IN
    for _ in range(8):
        t.tick(1, 0, 1, mode=mode)          # let the initial fade-in reach ST_IDLE
    if t.state != ST_IDLE or t.cur_idx != 0:
        return None
    seq = []
    idx = 0
    for _ in range(n_transitions):
        idx = (idx + 1) % 4
        eff = _drive_transition(t, idx, mode)
        if eff is None:
            return None
        seq.append(eff)
    return seq


class _BrokenVideoTransition(VideoTransition):
    """Negative control: the two forced wipe codes swapped, i.e. exactly the
    mis-wiring where 001 drives wipe-up and 010 drives wipe-down."""

    def tick(self, display_valid, disp_idx, frame_start, mode=0):
        swapped = {0b001: 0b010, 0b010: 0b001}.get(mode, mode)
        return VideoTransition.tick(self, display_valid, disp_idx, frame_start,
                                    mode=swapped)


def _names(seq):
    return ','.join(EFF_NAMES.get(e, '?%d' % e) for e in seq)


def pass_f():
    print("Pass F  DIP I_mode (3 bit) selects the transition effect")
    N = 8
    seqs = {m: _effect_sequence(m, N) for m in range(8)}
    if any(seqs[m] is None for m in range(8)):
        fail("a mode never settled into a clean transition sequence: %s"
             % {m: seqs[m] for m in range(8)})
        print("  Pass F done")
        return

    # expected: auto (000) rotates 7,1,2,3,4,5,6,7,...; forced 001..110 are a
    # constant code; 111 forces fade. The rotation starts at 7 because
    # effect_cnt resets to 7 and is read before it advances.
    exp = {}
    c = 7
    auto = []
    for _ in range(N):
        auto.append(c)
        c = 1 if c >= 7 else c + 1
    exp[0] = auto
    for m in range(1, 7):
        exp[m] = [m] * N
    exp[7] = [7] * N
    labels = {0: 'auto-cycle', 7: 'fade only'}

    for m in range(8):
        name = labels.get(m, EFF_NAMES[m] + ' only')
        if seqs[m] == exp[m]:
            ok("mode %s (%-13s): %s"
               % (format(m, '03b'), name, _names(seqs[m])))
        else:
            fail("mode %s (%s) expected %s got %s"
                 % (format(m, '03b'), name, _names(exp[m]), _names(seqs[m])))

    # Negative control: inject the swapped-code bug and confirm the mode-001
    # signature changes, proving the checks above can actually fail rather than
    # passing for any old mapping.
    broken = _effect_sequence(0b001, N, cls=_BrokenVideoTransition)
    if broken == exp[1]:
        fail("negative control is toothless: swapping 001/010 still produced the "
             "wipe-down signature")
    elif broken is None:
        fail("negative control did not run")
    else:
        ok("negative control: swapping the forced codes turns mode 001 into %s, "
           "which the wipe-down check rejects" % _names(broken))
    print("  Pass F done")


# ---------------------------------------------------------------------------
# Pass G -- band effect geometry, all six effects across the ramp
# ---------------------------------------------------------------------------

def _band_sel(eff, progress, grp_max=REAL_GROUPS):
    """select_top over every group of one frame, as a plain list of 0/1."""
    return [FrameFifoRead.select_top(g, progress, eff, grp_max)
            for g in range(grp_max)]


def _band_strip(eff, progress, grp_max=REAL_GROUPS):
    """ASCII picture of the frame: '#' is a group revealed from the top buffer,
    '.' still shows the outgoing bottom buffer. One row per progress value makes
    the vertical sweep visible to the eye."""
    return ''.join('#' if s else '.' for s in _band_sel(eff, progress, grp_max))


class _BrokenBandEngine(FrameFifoRead):
    """Negative control for Pass G: select_top WITHOUT the progress >= grp_max
    saturation guard on blinds (3) and split (4). At full ramp the last slat of
    the blinds and the outermost group of the split then never reveal, leaving
    holes in the panel -- exactly what the coverage assertion forbids. The
    override is reached because _comb calls self.select_top, which resolves to
    this subclass method."""

    @staticmethod
    def select_top(g, progress, eff, grp_max=240):
        if eff == 3:                                   # blinds, guard dropped
            return 1 if (g & 0xF) < (progress >> 4) else 0
        if eff == 4:                                   # split, guard dropped
            half = grp_max >> 1
            diff = (g - half) if g > half else (half - g)
            return 1 if diff < (progress >> 1) else 0
        return FrameFifoRead.select_top(g, progress, eff, grp_max)


def pass_g():
    print("Pass G  band effect geometry, six effects across the ramp")
    ramp = list(range(REAL_STEP, REAL_GROUPS + 1, REAL_STEP))   # 8,16,..,240
    if ramp[-1] != REAL_GROUPS or len(ramp) != REAL_GROUPS // REAL_STEP:
        fail("the ramp does not land exactly on WIPE_GRP_MAX = %d" % REAL_GROUPS)

    # ---- G0, the eff4 split timing rewrite is identical to the golden form ----
    # frame_fifo_read.v computes split as two PARALLEL compares against prog
    # derived bounds, (gi > half-K) && (gi < half+K) with K = prog>>1, instead of
    # the series |gi-half| < K, so gi no longer feeds a subtractor -> mux ->
    # compare chain on the ext_mem_clk path into next_sel_r (the -0.184ns
    # violation). |gi-half| < K is algebraically (gi > half-K) && (gi < half+K)
    # for K >= 0, including the gi == half and K == 0 edges; prove it over the
    # whole reachable input space so the structural fix cannot shift geometry.
    # The prog >= grp_max guard short circuits before half-K could underflow.
    def _split_rtl(gi, prog, grp_max):
        if prog >= grp_max:
            return 1
        half = grp_max >> 1
        k = prog >> 1
        return 1 if (gi > (half - k) and gi < (half + k)) else 0

    def _split_gold(gi, prog, grp_max):
        if prog >= grp_max:
            return 1
        half = grp_max >> 1
        diff = (gi - half) if gi > half else (half - gi)
        return 1 if diff < (prog >> 1) else 0

    mism = 0
    for grp_max in (REAL_GROUPS, 16, 8):     # real panel + reduced coupled geoms
        for prog in range(0, grp_max + 1):
            for gi in range(0, grp_max + 1):
                if _split_rtl(gi, prog, grp_max) != _split_gold(gi, prog, grp_max):
                    mism += 1
                    if mism <= 5:
                        fail("eff4 split rewrite diverges: grp_max %d prog %d gi %d "
                             "rtl %d gold %d"
                             % (grp_max, prog, gi, _split_rtl(gi, prog, grp_max),
                                _split_gold(gi, prog, grp_max)))
    if mism == 0:
        ok("G0  eff4 split two-compare RTL form == |gi-half|<K golden form over "
           "all gi/prog at grp_max %s" % [REAL_GROUPS, 16, 8])

    # ---- G1, pure function: monotonic sweep to full coverage ----
    for eff in range(1, 7):
        counts = [sum(_band_sel(eff, P)) for P in ramp]
        if counts[-1] != REAL_GROUPS:
            fail("effect %d (%s) does not fully reveal at progress %d: %d/%d "
                 "groups from the top buffer"
                 % (eff, EFF_NAMES[eff], REAL_GROUPS, counts[-1], REAL_GROUPS))
        if any(counts[i + 1] < counts[i] for i in range(len(counts) - 1)):
            fail("effect %d (%s) revealed count is not monotonic over the ramp: "
                 "%s" % (eff, EFF_NAMES[eff], counts))
        # wipe down/up and comb must already show something on the first step;
        # blinds legitimately waits until progress reaches one slat (16)
        if counts[0] == 0 and eff in (1, 2, 6):
            fail("effect %d (%s) reveals nothing at the first ramp step"
                 % (eff, EFF_NAMES[eff]))
    ok("G1  all six effects sweep monotonically to full coverage at progress %d"
       % REAL_GROUPS)

    # eyeball the sweep: first step, mid ramp, saturated, one strip per effect
    for eff in range(1, 7):
        print("      effect %d %-12s" % (eff, EFF_NAMES[eff]))
        for P in (ramp[0], REAL_GROUPS // 2, REAL_GROUPS):
            print("        p=%3d |%s|" % (P, _band_strip(eff, P)))

    # ---- G2, cycle accurate: word-by-word attribution through the FSM ----
    bot_idx, top_idx = 1, 2
    base_bot, base_top = REAL_ADDRS[bot_idx], REAL_ADDRS[top_idx]
    probes = (ramp[0], REAL_GROUPS // 2, REAL_GROUPS)
    for eff in range(1, 7):
        for P in probes:
            m = FrameFifoRead(REAL_ADDRS, REAL_LEN, burst_size=REAL_BURST,
                              wipe=True, wipe_grp_max=REAL_GROUPS,
                              wipe_grp_step=REAL_STEP, effect=eff)
            preload_for(m, P)
            addrs, cyc, fgrp, ftop, fbot = run_one_frame(m, bot_idx, top_idx)
            if fgrp != P or ftop != top_idx or fbot != bot_idx:
                fail("effect %d progress %d: frame read with grp %d top %d bot "
                     "%d, expected %d %d %d"
                     % (eff, P, fgrp, ftop, fbot, P, top_idx, bot_idx))
                continue
            sel = _band_sel(eff, P)
            bad = 0
            first_bad = None
            for W, a in enumerate(addrs):
                g = W // (2 * REAL_LINE)
                want = (base_top if sel[g] else base_bot) + W
                if a != want:
                    bad += 1
                    if first_bad is None:
                        first_bad = (W, g, a, want)
            if bad:
                W, g, a, want = first_bad
                fail("effect %d (%s) progress %d: %d words mis-attributed, first "
                     "at word %d group %d addr %d expected %d"
                     % (eff, EFF_NAMES[eff], P, bad, W, g, a, want))
            # the redirects must land exactly on the select_top flips, each on a
            # whole two-line group boundary; sel[240] is evaluated separately
            # because the list only covers groups 0..239
            exp_cross = []
            for g in range(REAL_GROUPS):
                nxt = FrameFifoRead.select_top(g + 1, P, eff, REAL_GROUPS)
                if nxt != sel[g]:
                    exp_cross.append((g + 1) * (2 * REAL_LINE))
            if m.cross_words != exp_cross:
                fail("effect %d (%s) progress %d: %d redirects %s expected %d %s"
                     % (eff, EFF_NAMES[eff], P, len(m.cross_words),
                        m.cross_words[:8], len(exp_cross), exp_cross[:8]))
            if any(cw % (2 * REAL_LINE) for cw in m.cross_words):
                fail("effect %d progress %d: a redirect fired off a group "
                     "boundary" % (eff, P))
        ok("G2  effect %d (%-12s): attribution + group-aligned redirects correct "
           "at progress %s" % (eff, EFF_NAMES[eff], list(probes)))

    # ---- G3, negative control: the saturation guard is load bearing ----
    real_full = all(sum(_band_sel(e, REAL_GROUPS)) == REAL_GROUPS for e in (3, 4))
    holes = {e: REAL_GROUPS - sum(_BrokenBandEngine.select_top(
                 g, REAL_GROUPS, e, REAL_GROUPS) for g in range(REAL_GROUPS))
             for e in (3, 4)}
    if not real_full:
        fail("the real engine does not reach full coverage at saturation, so the "
             "guard check proves nothing")
    elif all(h == 0 for h in holes.values()):
        fail("negative control is toothless: dropping the saturation guard still "
             "reached full coverage for blinds and split")
    else:
        ok("G3  negative control: without the saturation guard blinds/split leave "
           "%s groups unrevealed at full ramp, which the coverage check rejects"
           % holes)
    print("  Pass G done")


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
    pass_f()
    print()
    pass_g()
    print()
    print("=" * 72)
    if failures:
        print("%d FAILURE(S)" % failures)
        return 1
    print("all passes clean")
    return 0


if __name__ == '__main__':
    sys.exit(main())
