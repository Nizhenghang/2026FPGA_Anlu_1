#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_uart_ctrl.py -- cycle-accurate model of the Stage 1 串口屏 control path.

Why this exists
---------------
There is no Verilog simulator on this machine (see the project memory
"no-verilog-simulator-use-python-models"), so every RTL change is proven with a
cycle-accurate Python model before it goes anywhere near the board. This file
models the exact non-blocking behaviour of the three clock domains touched by
the serial-screen feature:

  * clk        (50 MHz)  -- uart_screen_ctrl RX FSM + TJC parser, the
                            mode/marquee override latch, the command->toggle
                            generator, and the brightness merge.
  * sd_card_clk(100 MHz) -- the toggle-CDC synchroniser chain that rebuilds
                            single-cycle cmd_*_pulse_sd pulses for sd_card_bmp.
  * video_clk  (25 MHz)  -- the 2FF synchroniser that carries the override
                            levels into the trans_mode / marquee_en muxes.

The three clocks are run on one merged time axis with deliberate phase offsets
(clk at t=0 mod 20ns, sd_card_clk at t=3 mod 10ns, video_clk at t=7 mod 40ns)
so no two posedges ever coincide. That makes the CDC sampling realistic: a
toggle flipped in the clk domain is captured by sd_card_clk on a genuinely
asynchronous edge, exactly like hardware.

What it verifies (passes A-F)
-----------------------------
  A. RX byte decode at the real divider (CLKS_PER_BIT = 50e6/9600 = 5208):
     a "NEXT" frame arrives as the exact 7 bytes on the wire.
  B. Parser command effects for all ten commands, plus the exact-length /
     argument guards. MODE and FILT take ONE uppercase hex character and span
     the whole 0..F range; lowercase, 'G', a two-digit argument, and every
     out-of-range digit on the other commands must NOT fire. A rejected frame
     must not wedge the parser.
  C. Toggle-CDC: every clk-domain command produces exactly ONE sd_card_clk
     pulse (no loss, no double), and IMGX carries the right 2-bit value.
  D. Override mux + brightness merge + the FILT/FONT/MUSC crossing chains:
       - no screen command  -> trans_mode == ~sw and marquee_en == sw4,
                               bit-identical to the verified baseline;
       - MODE n / MARQ n    -> override wins, including the codes 8..15 that
                               the 3-bit physical branch can never produce;
       - physical sw change -> override cleared, physical path reclaims;
       - BRGT / BRUP / key3 -> brightness set, cycle-wrap, and OR-merge;
       - FILT n / FONT n    -> clk latch, bare 2FF into video_clk, then a
                               frame-atomic stage that only advances on
                               video_frame_start, so a mid-frame switch can
                               never tear the picture into a processed top
                               half and an unprocessed bottom half.
                               trans_mode / marquee_en stay bare 2FF.
       - MUSC n             -> clk latch into music_en, then a BARE 2FF into
                               both video_clk (the audio output mux and the
                               OSD's I_asrc) and sd_card_clk (sd_card_bmp's
                               music_req). Deliberately not frame-atomic: both
                               audio sources emit a continuous audio_valid
                               stream, so switching mid-frame costs at most one
                               sample of phase discontinuity, and gating it
                               would only delay the music_req withdrawal by a
                               frame and waste a sector read.
  E. Negative controls (the project mandates these):
       - only two 0xFF       -> no dispatch;
       - unknown keyword     -> no effect;
       - "NEXTX" (clen==5)   -> rejected by the exact-length guard;
       - half frame "MO"+FFF then a valid "NEXT" -> no false fire, later works;
       - full retreat: zero commands ever sent -> outputs track sw exactly,
         not one spurious pulse is emitted, filt_frame/font_frame stay 0
         (passthrough / flat glyphs), the audio source stays at its
         AUDIO_SRC_DEFAULT reset value in both domains, and the physical
         trans_mode branch never exceeds 7 despite the 4-bit wire -- the
         property that keeps the new transitions serial-port-only.
  F. Link-debug registers behind LEDs A4/A3/C10 (uart_tx has no readback, so
     these are the only window onto the link). They must be observation-only:
     zero traffic leaves both at reset and dispatches nothing. Then the three
     bring-up signatures must be distinguishable -- bytes but no terminator
     (toggle lit, terminator flag dark), exactly two 0xFF (terminator flag lit,
     still no dispatch), and a clean frame (all three layers respond). Also
     locks two framing properties the screen project relies on: contiguous
     frames with no idle gap both dispatch, and a single inter-frame 0x20
     sacrifices exactly the next frame and then self-heals.

Honest limits
-------------
  * Passes B-F shrink CLKS_PER_BIT to 16 to run fast. The RX FSM and parser are
     parameter-independent (CPB only sets bit width; a constant 2-cycle
     synchroniser latency shifts every sample point uniformly and stays inside
     each bit window for any CPB > 4), so this is a legitimate speed-up. Pass A
     re-runs the real 5208 divider end-to-end to prove it.
  * The OR-merge inside sd_card_bmp (key_next_press || cmd_next_pulse) and the
     img_loaded_count gate on IMGX are NOT modelled end-to-end -- this harness
     proves the pulses and values that FEED that logic are correct, which is the
     CDC-sensitive part. The OR itself is a one-line `||`.
  * The physical key debouncers (key_press_debounce) are not modelled; key3 is
     injected directly as a clk-domain pulse to exercise the brightness merge.

Usage
-----
    python tools/sim_uart_ctrl.py            # run all passes, print PASS/FAIL
    python tools/sim_uart_ctrl.py --verbose  # extra per-command tracing
Exit code is 0 on success, 1 if any check fails.
"""

import argparse
import sys

# ---------------------------------------------------------------------------
# clock geometry (nanoseconds). Offsets guarantee no two posedges coincide.
# ---------------------------------------------------------------------------
CLK_PERIOD = 20      # 50 MHz
SD_PERIOD = 10       # 100 MHz
VID_PERIOD = 40      # 25 MHz
CLK_OFF = 0
SD_OFF = 3
VID_OFF = 7

RX_IDLE, RX_START, RX_DATA, RX_STOP = 0, 1, 2, 3
CPB_FAST = 16                 # shrunk divider for the fast passes
CPB_REAL = 50_000_000 // 9600  # 5208, what actually ships

# The single-character argument alphabet for MODE and FILT. Uppercase only:
# the RTL's c5_is_hex excludes 'a'-'f', and Pass B asserts that.
HEX_DIGITS = "0123456789ABCDEF"


def frame(cmd_str):
    """ASCII bytes of a command + the TJC three-byte terminator."""
    return [ord(c) for c in cmd_str] + [0xFF, 0xFF, 0xFF]


def build_rx_wave(byte_list, cpb, idle_before=8, idle_between=2, idle_after=48):
    """
    Expand a byte list into per-clk-cycle uart_rx pin levels: idle high, then
    for each byte a low start bit, 8 LSB-first data bits, a high stop bit (each
    cpb clk cycles wide), with idle gaps between bytes and at the end.
    """
    segs = [(1, idle_before)]
    for i, b in enumerate(byte_list):
        segs.append((0, cpb))                       # start bit
        for k in range(8):
            segs.append(((b >> k) & 1, cpb))        # data bits, LSB first
        segs.append((1, cpb))                       # stop bit
        if i != len(byte_list) - 1:
            segs.append((1, idle_between))
    segs.append((1, idle_after))
    levels = []
    for lvl, n in segs:
        levels.extend([lvl] * n)
    return levels


class System(object):
    """Mirrors every register of the three clock domains in the RTL."""

    def __init__(self, cpb=CPB_FAST, sw=0xF, audio_default=0):
        self.cpb = cpb
        self.sw = sw                # physical DIP: sw[2:0]=SW1-3, sw[3]=SW4
        self.audio_default = audio_default   # top's AUDIO_SRC_DEFAULT parameter
        self.key3_press = 0         # one-shot clk-domain pulse (injectable)
        self.vs_pin = 1             # video_clk vsync level (injectable, idles high)
        self.rx_levels = []
        self.clk_n = 0
        self.sd_n = 0
        self.vid_n = 0
        self.st = self._reset_state()
        self.log = {'next': 0, 'auto': 0, 'bright_cycle': 0,
                    'bright_set': [], 'mode_set': [], 'marquee_set': [],
                    'img_set': [], 'filt_set': [], 'font_set': [],
                    'audio_set': [], 'rx': []}
        self.sdlog = {'next': 0, 'auto': 0, 'img': 0, 'img_vals': []}

    def _reset_state(self):
        s = {}
        # --- clk domain: uart_screen_ctrl RX ---
        s['rx_sync'] = 0b11
        s['rx_state'] = RX_IDLE
        s['rx_cnt'] = 0
        s['rx_bit'] = 0
        s['rx_shift'] = 0
        s['rx_byte'] = 0
        s['rx_valid'] = 0
        # --- clk domain: parser ---
        s['c0'] = s['c1'] = s['c2'] = s['c3'] = s['c4'] = s['c5'] = 0
        s['clen'] = 0
        s['ffc'] = 0
        s['dbg_rx_toggle'] = 0
        s['dbg_rx_ff'] = 0
        s['cmd_next_pulse'] = 0
        s['cmd_auto_pulse'] = 0
        s['cmd_bright_cycle_pulse'] = 0
        s['cmd_bright_set'] = 0
        s['cmd_bright_set_v'] = 0
        s['cmd_mode'] = 0
        s['cmd_mode_set'] = 0
        s['cmd_marquee'] = 0
        s['cmd_marquee_set'] = 0
        s['cmd_img_sel'] = 0
        s['cmd_img_sel_set'] = 0
        s['cmd_filt'] = 0
        s['cmd_filt_set'] = 0
        s['cmd_font'] = 0
        s['cmd_font_set'] = 0
        s['cmd_audio'] = 0
        s['cmd_audio_set'] = 0
        # --- clk domain: brightness + override latch + toggle gen ---
        s['brightness'] = 2
        s['sw_c0'] = s['sw_c1'] = s['sw_c2'] = 7
        s['sw4_c0'] = s['sw4_c1'] = s['sw4_c2'] = 1
        s['mode_ovr_val'] = 0
        s['mode_ovr_en'] = 0
        s['marq_ovr_val'] = 1
        s['marq_ovr_en'] = 0
        s['filt_val'] = 0           # FILT latch, 0 = passthrough
        s['font_val'] = 0           # FONT latch, 0 = flat glyphs
        s['music_en'] = self.audio_default   # MUSC latch, no DIP reclaim
        s['next_tgl'] = 0
        s['auto_tgl'] = 0
        s['img_tgl'] = 0
        s['img_sel_lat'] = 0
        # --- sd_card_clk domain ---
        s['next_tgl_s0'] = s['next_tgl_s1'] = s['next_tgl_s2'] = 0
        s['auto_tgl_s0'] = s['auto_tgl_s1'] = s['auto_tgl_s2'] = 0
        s['img_tgl_s0'] = s['img_tgl_s1'] = s['img_tgl_s2'] = 0
        s['img_sel_s0'] = s['img_sel_s1'] = 0
        s['music_en_s0'] = s['music_en_s1'] = self.audio_default
        # --- video_clk domain ---
        s['sw_v0'] = s['sw_v1'] = 7
        s['sw4_v0'] = s['sw4_v1'] = 1
        s['mode_ovr_val_v0'] = s['mode_ovr_val_v1'] = 0
        s['mode_ovr_en_v0'] = s['mode_ovr_en_v1'] = 0
        s['marq_ovr_val_v0'] = s['marq_ovr_val_v1'] = 1
        s['marq_ovr_en_v0'] = s['marq_ovr_en_v1'] = 0
        s['filt_val_v0'] = s['filt_val_v1'] = 0
        s['font_val_v0'] = s['font_val_v1'] = 0
        s['music_en_v0'] = s['music_en_v1'] = self.audio_default
        s['filt_frame'] = 0         # frame-atomic: what the pixels actually see
        s['font_frame'] = 0
        s['vs_d'] = 0               # video_frame_start = vs_d & ~vs
        return s

    # -- combinational outputs -------------------------------------------
    def trans_mode(self):
        # 4 bits wide in the RTL now, but the physical branch is {1'b0, ~sw_v1}
        # so it still yields exactly 0..7: codes 8..15 can only arrive over the
        # serial port. That is deliberate -- the verified DIP path is unchanged.
        return self.st['mode_ovr_val_v1'] if self.st['mode_ovr_en_v1'] \
            else ((~self.st['sw_v1']) & 7)

    def marquee_en(self):
        return self.st['marq_ovr_val_v1'] if self.st['marq_ovr_en_v1'] \
            else self.st['sw4_v1']

    def filt_out(self):
        return self.st['filt_frame']

    def font_out(self):
        return self.st['font_frame']

    def audio_sel_v(self):
        """video_clk view: selects the audio output mux and drives OSD I_asrc."""
        return self.st['music_en_v1']

    def music_req_sd(self):
        """sd_card_clk view: sd_card_bmp's music_req, gates audio_phase arming."""
        return self.st['music_en_s1']

    def cmd_next_pulse_sd(self):
        return self.st['next_tgl_s1'] ^ self.st['next_tgl_s2']

    def cmd_auto_pulse_sd(self):
        return self.st['auto_tgl_s1'] ^ self.st['auto_tgl_s2']

    def cmd_img_sel_pulse_sd(self):
        return self.st['img_tgl_s1'] ^ self.st['img_tgl_s2']

    # -- clock-edge handlers (non-blocking: read old, write nxt) ---------
    def _do_clk(self):
        s = self.st
        n = self.clk_n
        rx_pin = self.rx_levels[n] if n < len(self.rx_levels) else 1
        nxt = {}

        # ---- RX FSM ----
        rx_in = (s['rx_sync'] >> 1) & 1
        nxt['rx_sync'] = ((s['rx_sync'] & 1) << 1) | rx_pin
        rx_state_n, rx_cnt_n = s['rx_state'], s['rx_cnt']
        rx_bit_n, rx_shift_n = s['rx_bit'], s['rx_shift']
        rx_byte_n, rx_valid_n = s['rx_byte'], 0
        cpb = self.cpb
        if s['rx_state'] == RX_IDLE:
            rx_cnt_n, rx_bit_n = 0, 0
            if rx_in == 0:
                rx_state_n = RX_START
        elif s['rx_state'] == RX_START:
            if s['rx_cnt'] == (cpb - 1) // 2:
                if rx_in == 0:
                    rx_cnt_n, rx_state_n = 0, RX_DATA
                else:
                    rx_state_n = RX_IDLE
            else:
                rx_cnt_n = s['rx_cnt'] + 1
        elif s['rx_state'] == RX_DATA:
            if s['rx_cnt'] == cpb - 1:
                rx_cnt_n = 0
                rx_shift_n = ((rx_in << 7) | (s['rx_shift'] >> 1)) & 0xFF
                rx_bit_n = (s['rx_bit'] + 1) & 7
                if s['rx_bit'] == 7:
                    rx_state_n = RX_STOP
            else:
                rx_cnt_n = s['rx_cnt'] + 1
        elif s['rx_state'] == RX_STOP:
            if s['rx_cnt'] == cpb - 1:
                rx_cnt_n, rx_state_n = 0, RX_IDLE
                if rx_in == 1:
                    rx_byte_n, rx_valid_n = s['rx_shift'], 1
            else:
                rx_cnt_n = s['rx_cnt'] + 1
        nxt['rx_state'] = rx_state_n
        nxt['rx_cnt'] = rx_cnt_n & 0xFFFF
        nxt['rx_bit'] = rx_bit_n
        nxt['rx_shift'] = rx_shift_n
        nxt['rx_byte'] = rx_byte_n
        nxt['rx_valid'] = rx_valid_n

        # ---- parser (consumes the OLD rx_valid / rx_byte) ----
        for k in ('cmd_next_pulse', 'cmd_auto_pulse', 'cmd_bright_cycle_pulse',
                  'cmd_bright_set_v', 'cmd_mode_set', 'cmd_marquee_set',
                  'cmd_img_sel_set', 'cmd_filt_set', 'cmd_font_set',
                  'cmd_audio_set'):
            nxt[k] = 0
        nxt['cmd_bright_set'] = s['cmd_bright_set']
        nxt['cmd_mode'] = s['cmd_mode']
        nxt['cmd_marquee'] = s['cmd_marquee']
        nxt['cmd_img_sel'] = s['cmd_img_sel']
        nxt['cmd_filt'] = s['cmd_filt']
        nxt['cmd_font'] = s['cmd_font']
        nxt['cmd_audio'] = s['cmd_audio']
        for k in ('c0', 'c1', 'c2', 'c3', 'c4', 'c5'):
            nxt[k] = s[k]
        nxt['clen'] = s['clen']
        nxt['ffc'] = s['ffc']
        nxt['dbg_rx_toggle'] = s['dbg_rx_toggle']
        nxt['dbg_rx_ff'] = s['dbg_rx_ff']

        if s['rx_valid']:
            b = s['rx_byte']
            # observation-only, mirrors the RTL: driven before the 0xFF test so it
            # cannot perturb ffc / clen / any cmd_* effect.
            nxt['dbg_rx_toggle'] = s['dbg_rx_toggle'] ^ 1
            nxt['dbg_rx_ff'] = 1 if b == 0xFF else 0
            if b == 0xFF:
                if s['ffc'] == 2:
                    nxt['ffc'] = 0
                    nxt['clen'] = 0
                    key = bytes([s['c0'], s['c1'], s['c2'], s['c3']])
                    clen, c4, c5 = s['clen'], s['c4'], s['c5']
                    # c5_is_hex / c5_hex mirror the RTL wires of the same name.
                    # Lowercase is deliberately outside the accepted set.
                    c5_is_hex = (0x30 <= c5 <= 0x39) or (0x41 <= c5 <= 0x46)
                    c5_hex = (c5 - 0x30) if c5 <= 0x39 else (c5 - 0x41) + 10
                    if key == b"NEXT" and clen == 4:
                        nxt['cmd_next_pulse'] = 1
                    elif key == b"AUTO" and clen == 4:
                        nxt['cmd_auto_pulse'] = 1
                    elif key == b"BRUP" and clen == 4:
                        nxt['cmd_bright_cycle_pulse'] = 1
                    elif key == b"BRGT" and clen == 6 and c4 == 0x20 \
                            and 0x30 <= c5 <= 0x34:
                        nxt['cmd_bright_set'] = c5 - 0x30
                        nxt['cmd_bright_set_v'] = 1
                    elif key == b"MODE" and clen == 6 and c4 == 0x20 and c5_is_hex:
                        nxt['cmd_mode'] = c5_hex
                        nxt['cmd_mode_set'] = 1
                    elif key == b"MARQ" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        nxt['cmd_marquee'] = 1 if c5 == 0x31 else 0
                        nxt['cmd_marquee_set'] = 1
                    elif key == b"IMGX" and clen == 6 and c4 == 0x20 \
                            and 0x31 <= c5 <= 0x34:
                        nxt['cmd_img_sel'] = (c5 - 0x30 - 1) & 3
                        nxt['cmd_img_sel_set'] = 1
                    elif key == b"FILT" and clen == 6 and c4 == 0x20 and c5_is_hex:
                        nxt['cmd_filt'] = c5_hex
                        nxt['cmd_filt_set'] = 1
                    elif key == b"FONT" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        nxt['cmd_font'] = 1 if c5 == 0x31 else 0
                        nxt['cmd_font_set'] = 1
                    elif key == b"MUSC" and clen == 6 and c4 == 0x20 \
                            and c5 in (0x30, 0x31):
                        nxt['cmd_audio'] = 1 if c5 == 0x31 else 0
                        nxt['cmd_audio_set'] = 1
                else:
                    nxt['ffc'] = (s['ffc'] + 1) & 3
            else:
                nxt['ffc'] = 0
                if s['clen'] < 6:
                    nxt['c%d' % s['clen']] = b
                if s['clen'] < 15:
                    nxt['clen'] = s['clen'] + 1

        # ---- override latch (OLD cmd_*_set, OLD sw_c*, live sw) ----
        nxt['sw_c0'] = self.sw & 7
        nxt['sw_c1'] = s['sw_c0']
        nxt['sw_c2'] = s['sw_c1']
        nxt['sw4_c0'] = (self.sw >> 3) & 1
        nxt['sw4_c1'] = s['sw4_c0']
        nxt['sw4_c2'] = s['sw4_c1']
        nxt['mode_ovr_val'] = s['mode_ovr_val']
        nxt['mode_ovr_en'] = s['mode_ovr_en']
        nxt['marq_ovr_val'] = s['marq_ovr_val']
        nxt['marq_ovr_en'] = s['marq_ovr_en']
        if s['cmd_mode_set']:
            nxt['mode_ovr_val'] = s['cmd_mode']
            nxt['mode_ovr_en'] = 1
        elif s['sw_c1'] != s['sw_c2']:
            nxt['mode_ovr_en'] = 0
        if s['cmd_marquee_set']:
            nxt['marq_ovr_val'] = s['cmd_marquee']
            nxt['marq_ovr_en'] = 1
        elif s['sw4_c1'] != s['sw4_c2']:
            nxt['marq_ovr_en'] = 0

        # ---- FILT/FONT/MUSC value latch: own block in the RTL, no ovr_en, no
        #      DIP-reclaim (no physical control is free for these three) ----
        nxt['filt_val'] = s['cmd_filt'] if s['cmd_filt_set'] else s['filt_val']
        nxt['font_val'] = s['cmd_font'] if s['cmd_font_set'] else s['font_val']
        nxt['music_en'] = s['cmd_audio'] if s['cmd_audio_set'] else s['music_en']

        # ---- toggle generator (OLD cmd pulses) ----
        nxt['next_tgl'] = s['next_tgl']
        nxt['auto_tgl'] = s['auto_tgl']
        nxt['img_tgl'] = s['img_tgl']
        nxt['img_sel_lat'] = s['img_sel_lat']
        if s['cmd_next_pulse']:
            nxt['next_tgl'] = 1 - s['next_tgl']
        if s['cmd_auto_pulse']:
            nxt['auto_tgl'] = 1 - s['auto_tgl']
        if s['cmd_img_sel_set']:
            nxt['img_sel_lat'] = s['cmd_img_sel']
            nxt['img_tgl'] = 1 - s['img_tgl']

        # ---- brightness merge (OLD strobes, live key3) ----
        bright_n = s['brightness']
        if s['cmd_bright_set_v']:
            bright_n = s['cmd_bright_set']
        elif self.key3_press or s['cmd_bright_cycle_pulse']:
            bright_n = 0 if s['brightness'] == 4 else s['brightness'] + 1
        nxt['brightness'] = bright_n

        # ---- commit ----
        s.update(nxt)
        self.key3_press = 0     # consume the one-shot

        # ---- record effects from the committed values ----
        if nxt['rx_valid']:
            self.log['rx'].append(nxt['rx_byte'])
        if nxt['cmd_next_pulse']:
            self.log['next'] += 1
        if nxt['cmd_auto_pulse']:
            self.log['auto'] += 1
        if nxt['cmd_bright_cycle_pulse']:
            self.log['bright_cycle'] += 1
        if nxt['cmd_bright_set_v']:
            self.log['bright_set'].append(nxt['cmd_bright_set'])
        if nxt['cmd_mode_set']:
            self.log['mode_set'].append(nxt['cmd_mode'])
        if nxt['cmd_marquee_set']:
            self.log['marquee_set'].append(nxt['cmd_marquee'])
        if nxt['cmd_img_sel_set']:
            self.log['img_set'].append(nxt['cmd_img_sel'])
        if nxt['cmd_filt_set']:
            self.log['filt_set'].append(nxt['cmd_filt'])
        if nxt['cmd_font_set']:
            self.log['font_set'].append(nxt['cmd_font'])
        if nxt['cmd_audio_set']:
            self.log['audio_set'].append(nxt['cmd_audio'])

    def _do_sd(self):
        s = self.st
        # detect pulses from the PRE-edge s1^s2 (what sd_card_bmp would see)
        if s['next_tgl_s1'] ^ s['next_tgl_s2']:
            self.sdlog['next'] += 1
        if s['auto_tgl_s1'] ^ s['auto_tgl_s2']:
            self.sdlog['auto'] += 1
        if s['img_tgl_s1'] ^ s['img_tgl_s2']:
            self.sdlog['img'] += 1
            self.sdlog['img_vals'].append(s['img_sel_s1'])
        nxt = {
            'next_tgl_s0': s['next_tgl'], 'next_tgl_s1': s['next_tgl_s0'],
            'next_tgl_s2': s['next_tgl_s1'],
            'auto_tgl_s0': s['auto_tgl'], 'auto_tgl_s1': s['auto_tgl_s0'],
            'auto_tgl_s2': s['auto_tgl_s1'],
            'img_tgl_s0': s['img_tgl'], 'img_tgl_s1': s['img_tgl_s0'],
            'img_tgl_s2': s['img_tgl_s1'],
            'img_sel_s0': s['img_sel_lat'], 'img_sel_s1': s['img_sel_s0'],
            'music_en_s0': s['music_en'], 'music_en_s1': s['music_en_s0'],
        }
        s.update(nxt)

    def _do_vid(self):
        s = self.st
        nxt = {
            'sw_v0': self.sw & 7, 'sw_v1': s['sw_v0'],
            'sw4_v0': (self.sw >> 3) & 1, 'sw4_v1': s['sw4_v0'],
            'mode_ovr_val_v0': s['mode_ovr_val'],
            'mode_ovr_val_v1': s['mode_ovr_val_v0'],
            'mode_ovr_en_v0': s['mode_ovr_en'],
            'mode_ovr_en_v1': s['mode_ovr_en_v0'],
            'marq_ovr_val_v0': s['marq_ovr_val'],
            'marq_ovr_val_v1': s['marq_ovr_val_v0'],
            'marq_ovr_en_v0': s['marq_ovr_en'],
            'marq_ovr_en_v1': s['marq_ovr_en_v0'],
            'filt_val_v0': s['filt_val'],
            'filt_val_v1': s['filt_val_v0'],
            'font_val_v0': s['font_val'],
            'font_val_v1': s['font_val_v0'],
            # music_en is a bare 2FF, NOT frame-atomic: both audio sources emit
            # a continuous audio_valid stream, so a switch costs at most one
            # sample of phase discontinuity and the ACR reference never breaks.
            'music_en_v0': s['music_en'],
            'music_en_v1': s['music_en_v0'],
            'vs_d': self.vs_pin,
            # frame-atomic: video_frame_start = OLD vs_d & ~live vs
            'filt_frame': s['filt_val_v1'] if (s['vs_d'] and not self.vs_pin)
                          else s['filt_frame'],
            'font_frame': s['font_val_v1'] if (s['vs_d'] and not self.vs_pin)
                          else s['font_frame'],
        }
        s.update(nxt)

    # -- time advance ----------------------------------------------------
    def run_clk(self, target_n):
        """Process edges in strict time order until clk_n reaches target_n."""
        while self.clk_n < target_n:
            tc = CLK_OFF + CLK_PERIOD * self.clk_n
            ts = SD_OFF + SD_PERIOD * self.sd_n
            tv = VID_OFF + VID_PERIOD * self.vid_n
            m = min(tc, ts, tv)
            if m == tc:
                self._do_clk(); self.clk_n += 1
            elif m == ts:
                self._do_sd(); self.sd_n += 1
            else:
                self._do_vid(); self.vid_n += 1

    def send(self, byte_list, cpb=None, extra=120):
        """Push a frame onto the wire and run enough cycles to fully drain it."""
        if cpb is None:
            cpb = self.cpb
        # rx_levels is indexed by absolute clk cycle. If settle()/a prior frame
        # already advanced clk_n past the end of the wire, pad with idle-high so
        # the new frame's start bit lands exactly on the next clk edge -- else
        # the RX FSM would sample the middle of the wave and miss it.
        while len(self.rx_levels) < self.clk_n:
            self.rx_levels.append(1)
        self.rx_levels.extend(build_rx_wave(byte_list, cpb))
        self.run_clk(len(self.rx_levels) + extra)

    def settle(self, cycles=40):
        """Run idle clk cycles (line stays high) to let CDC chains resolve."""
        self.run_clk(self.clk_n + cycles)

    def frame_boundary(self, low_cycles=4, high_cycles=4):
        """
        Drive one vsync low pulse so video_frame_start = vs_d & ~vs fires
        exactly once. FILT/FONT are frame-atomic in the RTL, so a pass that
        wants a new value to reach the pixels must cross a frame boundary.
        """
        self.vs_pin = 0
        self.run_clk(self.clk_n + low_cycles)
        self.vs_pin = 1
        self.run_clk(self.clk_n + high_cycles)


class Result(object):
    def __init__(self):
        self.rows = []
        self.failed = 0

    def add(self, ok, name, detail):
        self.rows.append((ok, name, detail))
        if not ok:
            self.failed += 1
        print("  [%s] %-46s %s" % ("PASS" if ok else "FAIL", name, detail))


# ---------------------------------------------------------------------------
# Pass A -- real divider byte decode
# ---------------------------------------------------------------------------
def pass_a(res, verbose):
    print("=" * 78)
    print("A. RX decode at the real divider CLKS_PER_BIT = %d (50 MHz / 9600)" % CPB_REAL)
    print("=" * 78)
    sysA = System(cpb=CPB_REAL)
    sysA.send(frame("NEXT"), extra=200)
    want = frame("NEXT")
    got = sysA.log['rx']
    res.add(got == want, "real-CPB byte stream",
            "decoded %d bytes %s" % (len(got), got))
    res.add(sysA.log['next'] == 1 and sysA.sdlog['next'] == 1,
            "real-CPB NEXT -> 1 clk + 1 sd pulse",
            "clk next=%d, sd next=%d" % (sysA.log['next'], sysA.sdlog['next']))
    if verbose:
        print("      rx bytes:", got)


# ---------------------------------------------------------------------------
# Pass B -- parser command effects and guards
# ---------------------------------------------------------------------------
def pass_b(res, verbose):
    print("=" * 78)
    print("B. Parser command effects (cpb=%d)" % CPB_FAST)
    print("=" * 78)

    # (frame, expected log key, expected value or None for pulse-count==1)
    pulse_cases = [
        ("NEXT", 'next'),
        ("AUTO", 'auto'),
        ("BRUP", 'bright_cycle'),
    ]
    for cmd, key in pulse_cases:
        s = System()
        s.send(frame(cmd))
        fired = s.log[key] == 1
        others = sum(v for k, v in s.log.items()
                     if k in ('next', 'auto', 'bright_cycle') and k != key)
        res.add(fired and others == 0, "%s fires once, nothing else" % cmd,
                "%s=%d, other pulses=%d" % (key, s.log[key], others))

    value_cases = [
        ("BRGT 3", 'bright_set', [3]),
        ("MARQ 0", 'marquee_set', [0]),
        ("MARQ 1", 'marquee_set', [1]),
        ("IMGX 2", 'img_set', [1]),      # picture 2 -> index 1
        ("IMGX 4", 'img_set', [3]),
    ]
    # MODE and FILT each span their whole accepted range now that the argument
    # is a hex character: 0 is the retreat (DIP path / passthrough) and F is the
    # top reserved code, so a mis-set bound or a wrong c5_hex decode shows up
    # here rather than on the board.
    for n in range(16):
        value_cases.append(("MODE %s" % HEX_DIGITS[n], 'mode_set', [n]))
    for n in range(16):
        value_cases.append(("FILT %s" % HEX_DIGITS[n], 'filt_set', [n]))
    value_cases += [
        ("FONT 0", 'font_set', [0]),
        ("FONT 1", 'font_set', [1]),
        ("MUSC 0", 'audio_set', [0]),
        ("MUSC 1", 'audio_set', [1]),
    ]
    for cmd, key, want in value_cases:
        s = System()
        s.send(frame(cmd))
        res.add(s.log[key] == want, "%s -> %s=%s" % (cmd, key, want),
                "got %s" % s.log[key])

    # guards: out-of-range digits, wrong lengths and wrong case must NOT fire
    guard_cases = [
        ("BRGT 5", 'bright_set', "digit >4 rejected"),
        ("IMGX 0", 'img_set', "digit <1 rejected"),
        ("IMGX 5", 'img_set', "digit >4 rejected"),
        ("MODE 33", 'mode_set', "two-digit arg -> clen==7 rejected"),
        ("FILT 33", 'filt_set', "two-digit arg -> clen==7 rejected"),
        ("FONT 11", 'font_set', "two-digit arg -> clen==7 rejected"),
        ("FONT 2", 'font_set', "digit >1 rejected"),
        ("MODE a", 'mode_set', "lowercase hex rejected"),
        ("MODE f", 'mode_set', "lowercase hex rejected"),
        ("FILT c", 'filt_set', "lowercase hex rejected"),
        ("MODE G", 'mode_set', "'G' is outside '0'-'9'/'A'-'F'"),
        ("FILT G", 'filt_set', "'G' is outside '0'-'9'/'A'-'F'"),
        ("MODE 10", 'mode_set', "decimal 10 as two chars -> clen==7 rejected"),
        ("FILT 10", 'filt_set', "decimal 10 as two chars -> clen==7 rejected"),
        ("MUSC 2", 'audio_set', "digit other than 0/1 rejected"),
        ("MUSC", 'audio_set', "no argument -> clen==4 rejected"),
        ("MUSCX 1", 'audio_set', "five-char keyword -> clen==7 rejected"),
        ("MUSC 11", 'audio_set', "two-digit arg -> clen==7 rejected"),
    ]
    for cmd, key, why in guard_cases:
        s = System()
        s.send(frame(cmd))
        res.add(len(s.log[key]) == 0, "%s silent (%s)" % (cmd, why),
                "%s=%s" % (key, s.log[key]))

    # brightness set actually lands in the register
    s = System()
    s.send(frame("BRGT 4"))
    res.add(s.st['brightness'] == 4, "BRGT 4 sets brightness_level",
            "brightness=%d" % s.st['brightness'])

    # A rejected frame must not wedge the parser: the next valid frame has to
    # dispatch normally, which is what makes a mistyped screen button harmless.
    s = System()
    s.send(frame("MODE a"))
    s.send(frame("MUSC 2"))
    s.settle(40)
    rejected = len(s.log['mode_set']) + len(s.log['audio_set'])
    s.send(frame("MODE C"))
    s.send(frame("MUSC 1"))
    s.settle(40)
    res.add(rejected == 0 and s.log['mode_set'] == [12]
            and s.log['audio_set'] == [1],
            "parser recovers after rejected frames",
            "rejected=%d then MODE C -> %s, MUSC 1 -> %s"
            % (rejected, s.log['mode_set'], s.log['audio_set']))
    if verbose:
        print("      guard cases all silent as expected")


# ---------------------------------------------------------------------------
# Pass C -- toggle-CDC: exactly one sd pulse per command, correct img value
# ---------------------------------------------------------------------------
def pass_c(res, verbose):
    print("=" * 78)
    print("C. Toggle-CDC clk -> sd_card_clk (exactly one pulse, no loss/double)")
    print("=" * 78)

    s = System()
    s.send(frame("NEXT"))
    s.send(frame("NEXT"))
    s.send(frame("NEXT"))
    res.add(s.log['next'] == 3 and s.sdlog['next'] == 3,
            "3x NEXT -> 3 clk, 3 sd pulses",
            "clk=%d sd=%d" % (s.log['next'], s.sdlog['next']))

    s = System()
    s.send(frame("AUTO"))
    s.send(frame("AUTO"))
    res.add(s.log['auto'] == 2 and s.sdlog['auto'] == 2,
            "2x AUTO -> 2 sd pulses",
            "clk=%d sd=%d" % (s.log['auto'], s.sdlog['auto']))

    s = System()
    for n, idx in (("IMGX 1", 0), ("IMGX 3", 2), ("IMGX 4", 3)):
        s.send(frame(n))
    res.add(s.sdlog['img'] == 3 and s.sdlog['img_vals'] == [0, 2, 3],
            "IMGX 1/3/4 -> sd img_vals [0,2,3]",
            "count=%d vals=%s" % (s.sdlog['img'], s.sdlog['img_vals']))

    # a command must never leak a pulse onto a different channel
    s = System()
    s.send(frame("MODE 2"))
    s.settle()
    res.add(s.sdlog['next'] == 0 and s.sdlog['auto'] == 0 and s.sdlog['img'] == 0,
            "MODE leaks no sd pulse",
            "next=%d auto=%d img=%d" % (s.sdlog['next'], s.sdlog['auto'], s.sdlog['img']))
    if verbose:
        print("      img_vals:", s.sdlog['img_vals'])


# ---------------------------------------------------------------------------
# Pass D -- override mux + brightness merge + physical reclaim
# ---------------------------------------------------------------------------
def pass_d(res, verbose):
    print("=" * 78)
    print("D. trans_mode / marquee_en override mux, brightness merge")
    print("=" * 78)

    # no command: trans_mode == ~sw across the whole switch range
    ok = True
    detail = []
    for v in range(8):
        s = System(sw=(v | 0b1000))    # sw[3]=1 (SW4 OFF), sw[2:0]=v
        s.settle(60)
        tm = s.trans_mode()
        want = (~v) & 7
        detail.append("%d->%d" % (v, tm))
        if tm != want:
            ok = False
    res.add(ok, "no cmd: trans_mode == ~sw (all 8)", " ".join(detail))

    # no command: marquee_en == sw4
    ok = True
    detail = []
    for sw4 in (0, 1):
        s = System(sw=(0b111 | (sw4 << 3)))
        s.settle(60)
        me = s.marquee_en()
        detail.append("sw4=%d->%d" % (sw4, me))
        if me != sw4:
            ok = False
    res.add(ok, "no cmd: marquee_en == sw4", " ".join(detail))

    # MODE n overrides, and survives while sw is held still
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MODE 3"))
    s.settle(60)
    res.add(s.trans_mode() == 3, "MODE 3 overrides ~sw",
            "trans_mode=%d (sw held 111, ~sw would be 0)" % s.trans_mode())

    # MARQ 0 overrides
    s = System(sw=0xF)          # sw4=1 -> marquee shown by default
    s.settle(40)
    s.send(frame("MARQ 0"))
    s.settle(60)
    res.add(s.marquee_en() == 0, "MARQ 0 hides banner over sw4=1",
            "marquee_en=%d" % s.marquee_en())

    # physical sw change reclaims control from the override
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MODE 3"))
    s.settle(60)
    pre = s.trans_mode()
    s.sw = 0b1001               # sw[2:0] 7 -> 1, a real DIP movement
    s.settle(60)
    post = s.trans_mode()
    res.add(pre == 3 and post == ((~1) & 7),
            "DIP move clears override, ~sw reclaims",
            "before=%d after=%d (want 6)" % (pre, post))

    # same for marquee, and prove the override is truly gone afterwards
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("MARQ 0"))
    s.settle(60)
    pre = s.marquee_en()                  # override hides banner while sw4=1
    en_before = s.st['marq_ovr_en_v1']
    s.sw = 0b0111                         # sw[3] 1 -> 0, physical SW4 flip
    s.settle(60)
    en_after = s.st['marq_ovr_en_v1']     # override must be cleared now
    s.sw = 0xF                            # flip SW4 back 0 -> 1
    s.settle(60)
    tracks = s.marquee_en()               # must follow physical (=1), not stale 0
    res.add(pre == 0 and en_before == 1 and en_after == 0 and tracks == 1,
            "SW4 flip clears marquee override, physical reclaims",
            "pre=%d ovr_en %d->%d, after flip-back marquee_en=%d (want 1)"
            % (pre, en_before, en_after, tracks))

    # ---- FILT/FONT: clk latch -> 2FF -> frame-atomic output ----
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("FILT 4"))
    s.settle(200)                      # plenty of video_clk edges, no vsync pulse
    lat, v1, frm = s.st['filt_val'], s.st['filt_val_v1'], s.filt_out()
    s.frame_boundary()
    after = s.filt_out()
    res.add(lat == 4 and v1 == 4 and frm == 0 and after == 4,
            "FILT 4 latches + 2FF settles, pixels wait for frame start",
            "clk=%d v1=%d filt_frame before=%d after boundary=%d"
            % (lat, v1, frm, after))

    # a mid-frame change must NOT reach the pixels: this is the tear the
    # frame-atomic stage exists to prevent
    s.send(frame("FILT 5"))
    s.settle(400)
    held = s.filt_out()
    s.frame_boundary()
    released = s.filt_out()
    res.add(held == 4 and released == 5,
            "mid-frame FILT 5 held until the next frame start",
            "held=%d (want 4) then released=%d (want 5)" % (held, released))

    # FONT 1/0 across frame boundaries
    s = System(sw=0xF)
    s.settle(40)
    s.send(frame("FONT 1"))
    s.settle(200)
    pre = s.font_out()
    s.frame_boundary()
    on = s.font_out()
    s.send(frame("FONT 0"))
    s.frame_boundary()
    off = s.font_out()
    res.add(pre == 0 and on == 1 and off == 0,
            "FONT 1 -> emboss on at frame start, FONT 0 -> flat again",
            "before=%d on=%d off=%d" % (pre, on, off))

    # ---- MODE codes above 7 survive the 3 -> 4 bit widening ----
    # The physical branch is {1'b0, ~sw_v1} and can never produce these, so if
    # anything between cmd_mode and trans_mode still truncates to 3 bits the
    # new transitions silently come out as the old ones.
    ok = True
    detail = []
    for n in (8, 11, 12, 14, 15):
        s = System(sw=0xF)
        s.settle(40)
        s.send(frame("MODE %s" % HEX_DIGITS[n]))
        s.settle(60)
        tm = s.trans_mode()
        detail.append("%X->%d" % (n, tm))
        if tm != n:
            ok = False
    res.add(ok, "MODE 8/B/C/E/F reach trans_mode untruncated",
            " ".join(detail))

    # ---- MUSC: clk latch -> bare 2FF into video_clk AND sd_card_clk ----
    # Deliberately not frame-atomic. Both audio sources emit a continuous
    # audio_valid stream, so the mux may switch at any video_clk edge; gating
    # it on video_frame_start would only delay sd_card_bmp's music_req
    # withdrawal by a whole frame and waste a sector read.
    s = System(sw=0xF)
    s.settle(60)
    tone_v, tone_sd = s.audio_sel_v(), s.music_req_sd()
    s.send(frame("MUSC 1"))
    s.settle(200)                  # many video_clk edges, but NO vsync pulse
    mus_v, mus_sd = s.audio_sel_v(), s.music_req_sd()
    s.frame_boundary()
    after_v = s.audio_sel_v()
    res.add(tone_v == 0 and tone_sd == 0 and mus_v == 1 and mus_sd == 1
            and after_v == 1,
            "MUSC 1 switches both domains with no frame boundary",
            "reset v/sd=%d/%d, after MUSC 1 (no vsync) v/sd=%d/%d, "
            "after boundary v=%d" % (tone_v, tone_sd, mus_v, mus_sd, after_v))

    s.send(frame("MUSC 0"))
    s.settle(200)
    res.add(s.audio_sel_v() == 0 and s.music_req_sd() == 0,
            "MUSC 0 returns to the test tone in both domains",
            "v=%d sd=%d" % (s.audio_sel_v(), s.music_req_sd()))

    # The one-line retreat: AUDIO_SRC_DEFAULT = 1 must power up in today's
    # already-board-verified "music from power-up" state with zero traffic.
    s = System(sw=0xF, audio_default=1)
    s.settle(120)
    res.add(s.st['music_en'] == 1 and s.audio_sel_v() == 1
            and s.music_req_sd() == 1 and len(s.log['audio_set']) == 0,
            "AUDIO_SRC_DEFAULT=1 retreat powers up on music, zero traffic",
            "music_en=%d v=%d sd=%d, MUSC frames seen=%d"
            % (s.st['music_en'], s.audio_sel_v(), s.music_req_sd(),
               len(s.log['audio_set'])))

    # A rejected MUSC must leave the audio source exactly where it was.
    s = System(sw=0xF, audio_default=1)
    s.settle(60)
    s.send(frame("MUSC 2"))
    s.send(frame("MUSCX 1"))
    s.settle(120)
    res.add(s.audio_sel_v() == 1 and len(s.log['audio_set']) == 0,
            "rejected MUSC leaves the audio source untouched",
            "still %d after MUSC 2 and MUSCX 1" % s.audio_sel_v())

    # contrast: trans_mode is deliberately NOT frame-gated (bare 2FF), and the
    # MODE 3 test above already settled it with no vsync pulse at all.
    res.add(s.trans_mode() == ((~7) & 7),
            "trans_mode stays bare-2FF (no frame gate) alongside",
            "trans_mode=%d with sw held 111" % s.trans_mode())

    # brightness: BRUP wraps 4->0, key3 OR-merges, BRGT priority
    s = System()
    s.send(frame("BRGT 4"))
    b4 = s.st['brightness']
    s.send(frame("BRUP"))
    bwrap = s.st['brightness']
    s.key3_press = 1
    s.settle(1)
    bkey = s.st['brightness']
    res.add(b4 == 4 and bwrap == 0 and bkey == 1,
            "brightness: set 4, BRUP wraps to 0, key3 -> 1",
            "after BRGT4=%d after BRUP=%d after key3=%d" % (b4, bwrap, bkey))
    if verbose:
        print("      override + reclaim verified")


# ---------------------------------------------------------------------------
# Pass E -- negative controls and full retreat
# ---------------------------------------------------------------------------
def pass_e(res, verbose):
    print("=" * 78)
    print("E. Negative controls + retreat (no command == baseline)")
    print("=" * 78)

    # only two 0xFF -> never dispatch
    s = System()
    s.send([ord(c) for c in "NEXT"] + [0xFF, 0xFF])
    s.settle(80)
    res.add(s.log['next'] == 0 and s.sdlog['next'] == 0,
            "two 0xFF only -> no dispatch",
            "clk next=%d sd next=%d" % (s.log['next'], s.sdlog['next']))

    # unknown keyword
    s = System()
    s.send(frame("ZZZZ"))
    s.settle(80)
    total = (s.log['next'] + s.log['auto'] + s.log['bright_cycle']
             + len(s.log['bright_set']) + len(s.log['mode_set'])
             + len(s.log['marquee_set']) + len(s.log['img_set'])
             + len(s.log['filt_set']) + len(s.log['font_set'])
             + len(s.log['audio_set']))
    res.add(total == 0, "unknown keyword ZZZZ -> nothing",
            "total effects=%d" % total)

    # over-long keyword-only frame "NEXTX" -> clen==5, rejected
    s = System()
    s.send(frame("NEXTX"))
    s.settle(80)
    res.add(s.log['next'] == 0, "NEXTX (clen==5) rejected by length guard",
            "next=%d" % s.log['next'])

    # half frame "MO" + terminator, then a valid NEXT: no false MODE, NEXT works
    s = System()
    s.send([ord('M'), ord('O')] + [0xFF, 0xFF, 0xFF])
    s.settle(40)
    false_mode = len(s.log['mode_set'])
    s.send(frame("NEXT"))
    s.settle(40)
    res.add(false_mode == 0 and s.log['next'] == 1,
            "partial 'MO'+FFF then valid NEXT",
            "false mode_set=%d, later next=%d" % (false_mode, s.log['next']))

    # full retreat: zero commands ever, outputs track sw, no spurious pulse
    ok_track = True
    detail = []
    for v in range(8):
        s = System(sw=(v | 0b1000))
        s.settle(80)
        if s.trans_mode() != ((~v) & 7):
            ok_track = False
        detail.append("%d->%d" % (v, s.trans_mode()))
    s = System(sw=0xF)
    s.settle(400)
    # cross real frame boundaries with zero traffic: the retreat must hold for
    # filt_frame/font_frame too, not just for the combinational muxes
    for _ in range(4):
        s.frame_boundary()
    spur = (s.log['next'] + s.log['auto'] + s.log['bright_cycle']
            + s.sdlog['next'] + s.sdlog['auto'] + s.sdlog['img']
            + len(s.log['filt_set']) + len(s.log['font_set'])
            + len(s.log['audio_set']))
    res.add(ok_track, "retreat: trans_mode tracks ~sw with zero traffic",
            " ".join(detail))
    res.add(spur == 0 and s.st['brightness'] == 2,
            "retreat: no spurious pulse, brightness stays at reset 2",
            "spurious=%d brightness=%d" % (spur, s.st['brightness']))
    res.add(s.filt_out() == 0 and s.font_out() == 0,
            "retreat: filt_frame=0 (passthrough), font_frame=0 (flat)",
            "filt_frame=%d font_frame=%d after 4 frame boundaries"
            % (s.filt_out(), s.font_out()))
    res.add(s.audio_sel_v() == 0 and s.music_req_sd() == 0,
            "retreat: audio source stays on the test tone",
            "video_clk=%d sd_card_clk=%d after 4 frame boundaries"
            % (s.audio_sel_v(), s.music_req_sd()))

    # The physical branch is {1'b0, ~sw_v1} on a now-4-bit wire. It must still
    # only ever produce 0..7 -- that is what keeps the seven new transitions
    # reachable from the serial port alone, leaving the verified DIP path
    # bit-for-bit as it was.
    phys = []
    for v in range(8):
        p = System(sw=(v | 0b1000))
        p.settle(80)
        phys.append(p.trans_mode())
    res.add(max(phys) <= 7 and sorted(phys) == list(range(8)),
            "retreat: physical trans_mode never exceeds 7 (4-bit wire)",
            "sw 0..7 -> %s" % phys)
    if verbose:
        print("      negative controls complete")


# Pass F -- link-debug LED registers, observation-only
def pass_f(res, verbose):
    print("=" * 78)
    print("F. Link-debug LEDs (dbg_rx_toggle / dbg_rx_ff are observation-only)")
    print("=" * 78)

    # negative control: idle line, zero traffic -> both indicators stay dark and
    # nothing is dispatched. Proves the debug registers cannot self-trigger.
    s = System()
    s.settle(200)
    res.add(s.st['dbg_rx_toggle'] == 0 and s.st['dbg_rx_ff'] == 0
            and len(s.log['rx']) == 0 and len(s.log['filt_set']) == 0,
            "no traffic -> both debug registers stay at reset",
            "tgl=%d ff=%d rx=%d" % (s.st['dbg_rx_toggle'],
                                    s.st['dbg_rx_ff'], len(s.log['rx'])))

    # a well-formed 9-byte frame: all three layers respond
    s = System()
    s.send(frame("FILT 1"))
    s.settle(80)
    res.add(len(s.log['rx']) == 9 and s.st['dbg_rx_toggle'] == 9 % 2
            and s.st['dbg_rx_ff'] == 1 and s.log['filt_set'] == [1],
            "clean frame -> byte parity odd, terminator seen, frame accepted",
            "rx=%d tgl=%d ff=%d filt_set=%s" % (len(s.log['rx']),
            s.st['dbg_rx_toggle'], s.st['dbg_rx_ff'], s.log['filt_set']))

    # signature 1: bytes arrive but the screen never sent `printh ff ff ff`.
    # Odd byte count so the toggle indicator ends lit, last byte is payload so
    # the terminator indicator is dark. On the board: LED0 blinks, LED1 never
    # lights, LED2 never blinks.
    s = System()
    s.send([ord(c) for c in "FILT 1"] + [ord("X")])
    s.settle(80)
    res.add(len(s.log['rx']) == 7 and s.st['dbg_rx_toggle'] == 1
            and s.st['dbg_rx_ff'] == 0 and len(s.log['filt_set']) == 0,
            "no terminator -> bytes seen, terminator flag dark, no dispatch",
            "rx=%d tgl=%d ff=%d filt_set=%d" % (len(s.log['rx']),
            s.st['dbg_rx_toggle'], s.st['dbg_rx_ff'], len(s.log['filt_set'])))

    # signature 2: exactly two 0xFF. The terminator flag is lit (the last byte
    # really was 0xFF) yet nothing dispatches, because ffc only reaches 2.
    # On the board: LED0 blinks, LED1 lights, LED2 stays dark.
    s = System()
    s.send([ord(c) for c in "FILT 1"] + [0xFF, 0xFF])
    s.settle(80)
    res.add(len(s.log['rx']) == 8 and s.st['dbg_rx_ff'] == 1
            and len(s.log['filt_set']) == 0,
            "two 0xFF -> terminator flag lit but still no dispatch",
            "rx=%d ff=%d filt_set=%d" % (len(s.log['rx']),
            s.st['dbg_rx_ff'], len(s.log['filt_set'])))

    # contiguous frames in ONE send() call, i.e. no idle gap at all between the
    # two terminators and the next start bit. Both must dispatch.
    s = System()
    s.send(frame("FILT 1") + frame("FILT 2"))
    s.settle(80)
    res.add(len(s.log['rx']) == 18 and s.log['filt_set'] == [1, 2]
            and s.st['cmd_filt'] == 2,
            "back-to-back frames with zero gap both dispatch",
            "rx=%d filt_set=%s cmd_filt=%d" % (len(s.log['rx']),
            s.log['filt_set'], s.st['cmd_filt']))

    # a single inter-frame 0x20 shifts the next frame's keyword into c1..c4, so
    # {c0,c1,c2,c3} == " FIL" and it is silently dropped. Dispatch clears clen
    # unconditionally, so the frame after that one works: exactly one command is
    # lost and the link self-heals.
    s = System()
    s.send(frame("FILT 1") + [0x20] + frame("FILT 2") + frame("FILT 3"))
    s.settle(80)
    res.add(len(s.log['rx']) == 28 and s.log['filt_set'] == [1, 3]
            and s.st['cmd_filt'] == 3,
            "inter-frame 0x20 kills exactly the next frame, then self-heals",
            "rx=%d filt_set=%s cmd_filt=%d" % (len(s.log['rx']),
            s.log['filt_set'], s.st['cmd_filt']))
    if verbose:
        print("      link-debug LED checks complete")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    res = Result()
    pass_a(res, args.verbose)
    pass_b(res, args.verbose)
    pass_c(res, args.verbose)
    pass_d(res, args.verbose)
    pass_e(res, args.verbose)
    pass_f(res, args.verbose)
    print("=" * 78)
    total = len(res.rows)
    print("%d/%d checks passed, %d failed" % (total - res.failed, total, res.failed))
    print("=" * 78)
    sys.exit(1 if res.failed else 0)


if __name__ == "__main__":
    main()
