#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_video_effect.py -- cycle-free model of video_effect.v (thirteen point operations).

Why this exists
---------------
There is no Verilog simulator on this machine (see the project memory
"no-verilog-simulator-use-python-models"), so the point-operation arithmetic is
proven here before it goes anywhere near the board. video_effect.v is purely
combinational -- no clock, no reset, no state -- which means "cycle-accurate"
degenerates to "bit-accurate": the model must reproduce every intermediate
width and every truncation the RTL produces, not just the mathematical intent.

That is the whole point of Pass B. The RTL reaches its results through
shift-add decompositions (77R = R<<6 + R<<3 + R<<2 + R, and the factored sepia
sum S = 12T + R + G where T = 2R + 4G + B). Those decompositions are where
transcription errors live, and a model written from the same intent would share
the bug. So Pass B recomputes each mode from plain integer multiplication and
demands exact equality -- it catches a wrong shift, a dropped term, and a
mis-sized concatenation alike. The seven filters added later (codes 6..12) are
held to the same standard: each is an exact identity, so each is checked by
equality rather than by tolerance, and the whole output mux is additionally
compared against a reference assembled only from those independent functions so
a helper wired to the wrong sel fails too.

What it verifies (passes A-F)
-----------------------------
  A. Retreat proof: sel==0 is bit-identical passthrough for every tested colour.
  B. Shift-add decomposition == direct integer arithmetic, exactly, all modes.
  C. Accuracy against the textbook references over the full 16.7M-colour space:
     grayscale vs BT.601 <=1.5 levels, sepia vs the classic matrix <=2/3/5
     levels per channel.
  D. Clamp and fixed-point invariants: every output in 0..255; contrast leaves
     0, 128 and 255 unmoved; sepia maps black to black and white to cream;
     SATU and MUTE leave neutral grey alone; POSTE emits exactly four levels.
  E. Intermediate widths do not overflow the bit widths declared in the RTL
     (gray sum 16 bit, sepia T 11 bit, S 15 bit, sw/gp/bp 9 bit, g_mul5 11 bit,
     g_mul3 10 bit, satu up/dn 10 bit, mute sum 9 bit), plus the proof that
     bit 9 of satu's dn is an exact sign test over all 65536 (ch, y) pairs --
     the property the dark-side clamp is built on.
  F. Negative controls -- fifteen in-memory mutants that MUST be caught.

Honest limits
-------------
  * This models the arithmetic only. It says nothing about where video_effect is
     instantiated, nor about the frame-atomic latch that feeds I_sel; those live
     in sim_uart_ctrl.py.
  * Pass C's grayscale reference is BT.601 (0.299/0.587/0.114). The RTL uses
     77/150/29 over 256, which is the nearest exact-power-of-two-denominator
     fit; the residual is what Pass C bounds.

Usage
-----
    python tools/sim_video_effect.py
Exit code is 0 on success, 1 if any check fails.
"""

import sys

MASK8 = 0xFF
MASK9 = 0x1FF
MASK10 = 0x3FF
MASK11 = 0x7FF
MASK15 = 0x7FFF
MASK16 = 0xFFFF


# ---------------------------------------------------------------------------
# reference model -- mirrors video_effect.v line for line
# ---------------------------------------------------------------------------
def gray_y(r, g, b):
    """(77R + 150G + 29B) >> 8, coefficients summing to exactly 256."""
    pr = ((r << 6) + (r << 3)) + ((r << 2) + r)
    pg = ((g << 7) + (g << 4)) + ((g << 2) + (g << 1))
    pb = ((b << 4) + (b << 3)) + ((b << 2) + b)
    total = ((pr + pg) + pb) & MASK16
    return total >> 8, total, pr, pg, pb


def sepia_rgb(r, g, b):
    """
    S = 25R + 49G + 12B reached as 12T + R + G with T = 2R + 4G + B.
    Channel gains are taken off the UNCLAMPED sw = S>>6, then each clamped,
    because the textbook matrix clamps its three rows independently.
    """
    t = ((r << 1) + (g << 2)) + b
    rg = r + g
    s = ((t << 3) + (t << 2)) + rg
    sw = (s >> 6) & MASK9
    gp = ((sw - ((sw >> 3) & MASK9)) + ((sw >> 6) & MASK9)) & MASK9
    bp = (((sw >> 1) & MASK9) + ((sw >> 3) & MASK9)
          + ((sw >> 4) & MASK9) + ((sw >> 7) & MASK9)) & MASK9
    s8 = 255 if sw > 255 else sw
    g8 = 255 if gp > 255 else gp
    b8 = bp & MASK8
    return (s8, g8, b8), {'t': t & MASK11, 'rg': rg & MASK9,
                          's': s & MASK15, 'sw': sw, 'gp': gp, 'bp': bp}


def contrast_ch(ch):
    """Gain 1.5 about the 128 pivot, unsigned on both sides."""
    if ch >= 128:
        half = ((ch - 128) & MASK8) >> 1
        total = (ch + half) & MASK9
        return 255 if total > 255 else total & MASK8
    half = ((128 - ch) & MASK8) >> 1
    dif = ch - half
    return 0 if dif < 0 else dif & MASK8


def satu_ch(ch, y):
    """
    Saturation boost: y + 2*(ch - y), split on the sign of (ch - y) so both
    sides stay unsigned, exactly as the RTL's two-branch function does.
    Above luma the sum can exceed 255 and is clamped; below it the difference
    can go negative and is clamped to 0.
    """
    if ch >= y:
        dup = (ch - y) & MASK9
        up = (y + (dup << 1)) & MASK10
        return 255 if up > 255 else up & MASK8
    ddn = (y - ch) & MASK9
    dn = y - (ddn << 1)
    return 0 if dn < 0 else dn & MASK8


def mute_ch(ch, y):
    """Half way blend toward luma: (ch + y) >> 1. Max 510 >> 1 = 255, no clamp."""
    return ((ch + y) & MASK9) >> 1


def solar_ch(ch):
    """Exposure inversion: the dark half is kept, the bright half is inverted."""
    return ch if ch < 128 else (~ch) & MASK8


def cast58(v):
    """Exact floor(5v/8) via 5v then a bit-select -- what the RTL's g_mul5[10:3] does."""
    return (((v << 2) + v) & MASK11) >> 3


def cast38(v):
    """Exact floor(3v/8) via 3v then a bit-select -- the RTL's g_mul3[9:3]."""
    return (((v << 1) + v) & MASK10) >> 3


def effect(rgb, sel):
    """Top-level mux of video_effect.v."""
    r, g, b = (rgb >> 16) & MASK8, (rgb >> 8) & MASK8, rgb & MASK8
    if sel == 1:
        y = gray_y(r, g, b)[0]
        return (y << 16) | (y << 8) | y
    if sel == 2:
        return (~rgb) & 0xFFFFFF
    if sel == 3:
        y = gray_y(r, g, b)[0]
        return 0xFFFFFF if y >= 128 else 0x000000
    if sel == 4:
        s8, g8, b8 = sepia_rgb(r, g, b)[0]
        return (s8 << 16) | (g8 << 8) | b8
    if sel == 5:
        return (contrast_ch(r) << 16) | (contrast_ch(g) << 8) | contrast_ch(b)
    if sel == 6:
        y = gray_y(r, g, b)[0]
        return ((satu_ch(r, y) << 16) | (satu_ch(g, y) << 8) | satu_ch(b, y))
    if sel == 7:
        y = gray_y(r, g, b)[0]
        return ((mute_ch(r, y) << 16) | (mute_ch(g, y) << 8) | mute_ch(b, y))
    if sel == 8:
        return (r << 16) | (cast58(g) << 8) | (b >> 2)
    if sel == 9:
        return ((r >> 2) << 16) | (cast38(g) << 8) | b
    if sel == 10:
        return ((solar_ch(r) << 16) | (solar_ch(g) << 8) | solar_ch(b))
    if sel == 11:
        return (((r >> 6) << 22) | ((g >> 6) << 14) | ((b >> 6) << 6))
    if sel == 12:
        y = (~gray_y(r, g, b)[0]) & MASK8
        return (y << 16) | (y << 8) | y
    return rgb


# ---------------------------------------------------------------------------
# textbook references, independent of the decompositions above
# ---------------------------------------------------------------------------
def ref_gray(r, g, b):
    return (77 * r + 150 * g + 29 * b) >> 8


def ref_sepia_s(r, g, b):
    return 25 * r + 49 * g + 12 * b


def float_gray(r, g, b):
    return 0.299 * r + 0.587 * g + 0.114 * b


SEPIA_M = ((0.393, 0.769, 0.189),
           (0.349, 0.686, 0.168),
           (0.272, 0.534, 0.131))


def float_sepia(r, g, b):
    return tuple(min(255.0, round(row[0] * r + row[1] * g + row[2] * b))
                 for row in SEPIA_M)


# Independent references for the seven new filters. Deliberately written with
# plain multiplication and floor division, sharing no shift-add structure with
# the helpers above, so Pass B's equality is a real cross-check rather than a
# restatement of the same decomposition.
def ref_satu(ch, y):
    return min(255, max(0, 2 * ch - y))


def ref_mute(ch, y):
    return (ch + y) // 2


def ref_gain(num, v):
    return (num * v) // 8


def ref_solar(ch):
    return ch if ch < 128 else 255 - ch


def ref_poste(ch):
    return (ch // 64) * 64


def ref_ginv(ch):
    return 255 - ch


def ref_new_effect(rgb, sel):
    """
    Whole-pixel reference for codes 6..12, assembled only from the ref_*
    functions above. Checking effect() against this catches a mis-wired output
    mux (say AMBER reaching for COOL's gain) as well as a wrong helper.
    """
    r, g, b = (rgb >> 16) & MASK8, (rgb >> 8) & MASK8, rgb & MASK8
    y = ref_gray(r, g, b)
    if sel == 6:
        ch = (ref_satu(r, y), ref_satu(g, y), ref_satu(b, y))
    elif sel == 7:
        ch = (ref_mute(r, y), ref_mute(g, y), ref_mute(b, y))
    elif sel == 8:
        ch = (r, ref_gain(5, g), b >> 2)
    elif sel == 9:
        ch = (r >> 2, ref_gain(3, g), b)
    elif sel == 10:
        ch = (ref_solar(r), ref_solar(g), ref_solar(b))
    elif sel == 11:
        ch = (ref_poste(r), ref_poste(g), ref_poste(b))
    elif sel == 12:
        ch = (ref_ginv(y),) * 3
    else:
        return rgb
    return (ch[0] << 16) | (ch[1] << 8) | ch[2]


def full_space():
    for r in range(256):
        for g in range(256):
            for b in range(256):
                yield r, g, b


def coarse_space(step=3):
    for r in range(0, 256, step):
        for g in range(0, 256, step):
            for b in range(0, 256, step):
                yield r, g, b


def sweep_full():
    """
    One pass over all 16777216 colours, collecting everything Pass B and Pass E
    need: transcription mismatches against plain integer arithmetic, and the
    observed peak of every intermediate the RTL declares a width for.
    """
    bad_gray, bad_s, bad_t = [], [], []
    caps = {'gray_total': 0, 'sepia_t': 0, 'sepia_rg': 0, 'sepia_s': 0,
            'sepia_sw': 0, 'sepia_gp': 0, 'sepia_bp': 0}
    for r, g, b in full_space():
        y, total, _, _, _ = gray_y(r, g, b)
        if y != ref_gray(r, g, b) or total != 77 * r + 150 * g + 29 * b:
            if len(bad_gray) < 3:
                bad_gray.append((r, g, b, y, ref_gray(r, g, b)))
        out, iw = sepia_rgb(r, g, b)
        if iw['s'] != ref_sepia_s(r, g, b):
            if len(bad_s) < 3:
                bad_s.append((r, g, b, iw['s'], ref_sepia_s(r, g, b)))
        if iw['t'] != 2 * r + 4 * g + b:
            if len(bad_t) < 3:
                bad_t.append((r, g, b, iw['t']))
        if total > caps['gray_total']:
            caps['gray_total'] = total
        for k in ('t', 'rg', 's', 'sw', 'gp', 'bp'):
            key = 'sepia_' + k
            if iw[k] > caps[key]:
                caps[key] = iw[k]
    return {'bad_gray': bad_gray, 'bad_s': bad_s, 'bad_t': bad_t,
            'caps': caps}


SWEEP = None


def sweep():
    global SWEEP
    if SWEEP is None:
        SWEEP = sweep_full()
    return SWEEP


# ---------------------------------------------------------------------------
# in-memory mutants for Pass F. Each MUST be caught by at least one check.
# ---------------------------------------------------------------------------
def mut_gray_coef76(r, g, b):
    """grayscale green coefficient mistyped as 76 -> sum 255, white goes dark."""
    return (77 * r + 76 * g + 29 * b) >> 8


def mut_sepia_noclamp(r, g, b):
    """sepia with the sw>255 clamp removed -> bright pixels wrap."""
    t = (r << 1) + (g << 2) + b
    s = (t << 3) + (t << 2) + (r + g)
    sw = (s >> 6) & MASK9
    gp = (sw - (sw >> 3) + (sw >> 6)) & MASK9
    bp = ((sw >> 1) + (sw >> 3) + (sw >> 4) + (sw >> 7)) & MASK9
    return (sw & MASK8, gp & MASK8, bp & MASK8)


def mut_sepia_gain_from_clamped(r, g, b):
    """the bug this design avoids: gains derived from the CLAMPED s8."""
    t = (r << 1) + (g << 2) + b
    s = (t << 3) + (t << 2) + (r + g)
    sw = (s >> 6) & MASK9
    s8 = 255 if sw > 255 else sw
    return (s8, (s8 - (s8 >> 3) + (s8 >> 6)) & MASK8,
            ((s8 >> 1) + (s8 >> 3) + (s8 >> 4) + (s8 >> 7)) & MASK8)


def mut_contrast_no_low_clamp(ch):
    """contrast low side using bare 8-bit unsigned subtraction -> wraps."""
    if ch >= 128:
        total = ch + ((ch - 128) >> 1)
        return 255 if total > 255 else total
    return (ch - ((128 - ch) >> 1)) & MASK8


def mut_thresh_strict(y):
    """threshold using > instead of >= -> luma exactly 128 goes black."""
    return 0xFFFFFF if y > 128 else 0x000000


def mut_sepia_forget_shift(r, g, b):
    """sepia forgetting the >>6 scaling -> everything saturates."""
    t = (r << 1) + (g << 2) + b
    s = (t << 3) + (t << 2) + (r + g)
    sw = s & MASK9
    return (min(sw, 255), min(sw - (sw >> 3) + (sw >> 6), 255),
            min((sw >> 1) + (sw >> 3) + (sw >> 4) + (sw >> 7), 255))


def mut_mute_no_shift(ch, y):
    """MUTE forgetting the >>1 -> the 9 bit sum is truncated to 8 bits."""
    return (ch + y) & MASK8


def mut_satu_no_high_clamp(ch, y):
    """SATU with the >255 clamp dropped on the bright side -> wraps down."""
    if ch >= y:
        return (y + ((ch - y) << 1)) & MASK8
    dn = y - ((y - ch) << 1)
    return 0 if dn < 0 else dn & MASK8


def mut_satu_no_low_clamp(ch, y):
    """SATU taking the dark side through bare 8 bit unsigned -> wraps up."""
    if ch >= y:
        up = y + ((ch - y) << 1)
        return 255 if up > 255 else up & MASK8
    return (y - ((y - ch) << 1)) & MASK8


def mut_amber_compounded_floors(g):
    """
    The exact bug this design avoids: (g>>1)+(g>>3) compounds two floors and
    so is NOT floor(5g/8). It first diverges at g=5, where the shift-add gives
    2 and the true quotient is 3.
    """
    return ((g >> 1) + (g >> 3)) & MASK8


def mut_amber_cool_swapped(rgb, sel):
    """AMBER and COOL reading each other's green gain."""
    r, g, b = (rgb >> 16) & MASK8, (rgb >> 8) & MASK8, rgb & MASK8
    if sel == 8:
        return (r << 16) | (cast38(g) << 8) | (b >> 2)
    return ((r >> 2) << 16) | (cast58(g) << 8) | b


def mut_amber_cool_shared(rgb, sel):
    """
    AMBER and COOL sharing one green gain, the shortcut the first draft took.
    It is arithmetically cheaper but makes the two filters differ only in which
    of red and blue is cut, which reads as the same colour temperature on a
    panel. Caught because COOL's green must be 3/8, not 5/8.
    """
    r, g, b = (rgb >> 16) & MASK8, (rgb >> 8) & MASK8, rgb & MASK8
    shared = cast58(g)
    if sel == 8:
        return (r << 16) | (shared << 8) | (b >> 2)
    return ((r >> 2) << 16) | (shared << 8) | b


def mut_poste_3bit(ch):
    """POSTE truncating [7:5] instead of [7:6] -> 8 levels, not 4."""
    return ((ch >> 5) << 5) & MASK8


def mut_solar_inclusive(ch):
    """SOLAR using <=128 so mid grey inverts instead of being the fixed point."""
    return ch if ch <= 128 else (~ch) & MASK8


def mut_ginv_per_channel(r, g, b):
    """GINV inverting each channel -> degenerates into INVT, losing the luma."""
    return ((~r) & MASK8, (~g) & MASK8, (~b) & MASK8)


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
def pass_a(res):
    print("=" * 78)
    print("A. Retreat proof -- sel==0 is bit-identical passthrough")
    print("=" * 78)
    samples = [0x000000, 0xFFFFFF, 0x123456, 0xFF0000, 0x00FF00, 0x0000FF,
               0x808080, 0xFEDCBA, 0x010101, 0xFEFEFE]
    samples += [(r << 16) | (g << 8) | b for r, g, b in coarse_space(37)]
    bad = [hex(v) for v in samples if effect(v, 0) != v]
    res.add(not bad, "sel=0 passthrough over %d colours" % len(samples),
            "mismatches: %s" % (bad[:4] if bad else "none"))


def pass_b(res):
    print("=" * 78)
    print("B. Shift-add decomposition == direct integer arithmetic (exact)")
    print("=" * 78)
    sw_res = sweep()
    bad_gray, bad_s, bad_t = (sw_res['bad_gray'], sw_res['bad_s'],
                              sw_res['bad_t'])
    res.add(not bad_gray, "gray shift-add == (77R+150G+29B)>>8",
            "all 16777216 colours" if not bad_gray else "bad: %s" % bad_gray)
    res.add(not bad_s, "sepia S == 25R+49G+12B (via 12T+R+G)",
            "all 16777216 colours" if not bad_s else "bad: %s" % bad_s)
    res.add(not bad_t, "sepia T == 2R+4G+B",
            "all 16777216 colours" if not bad_t else "bad: %s" % bad_t)

    # The two channel gains are ratios, so the shift tree truncates at a
    # different place than a single multiply would. Equality is impossible;
    # what must hold is that the tree realises the fraction it claims
    # (57/64 = 1 - 1/8 + 1/64, 89/128 = 1/2 + 1/8 + 1/16 + 1/128) to within
    # the accumulated floor error of its own terms.
    gp_e = bp_e = 0.0
    gp_arg = bp_arg = 0
    for sw in range(343):
        gp = sw - (sw >> 3) + (sw >> 6)
        bp = (sw >> 1) + (sw >> 3) + (sw >> 4) + (sw >> 7)
        eg = abs(gp - sw * 57 / 64)
        eb = abs(bp - sw * 89 / 128)
        if eg > gp_e:
            gp_e, gp_arg = eg, sw
        if eb > bp_e:
            bp_e, bp_arg = eb, sw
    res.add(gp_e < 1.0, "sepia G gain tree realises 57/64 = 0.890625",
            "max |err| = %.3f at sw=%d (floor accumulation bound 7/8)"
            % (gp_e, gp_arg))
    res.add(bp_e < 3.5, "sepia B gain tree realises 89/128 = 0.6953125",
            "max |err| = %.3f at sw=%d (four floors, bound 3.31)"
            % (bp_e, bp_arg))

    # contrast: bounded deviation from the clamped ideal, plus monotonicity.
    # The RTL truncates inside each branch, so it can sit half a level off the
    # ideal 128 + 1.5*(ch-128); it must never invert the ordering.
    outs = [contrast_ch(ch) for ch in range(256)]
    ce = 0.0
    ce_arg = 0
    for ch in range(256):
        ideal = min(255.0, max(0.0, 128 + 1.5 * (ch - 128)))
        e = abs(outs[ch] - ideal)
        if e > ce:
            ce, ce_arg = e, ch
    mono = all(outs[i] <= outs[i + 1] for i in range(255))
    res.add(ce <= 0.5 and mono, "contrast == 1.5x about pivot 128, monotone",
            "max |err| = %.3f at ch=%d, non-decreasing=%s"
            % (ce, ce_arg, mono))

    # The seven new filters. Every one is an exact identity against plain
    # integer arithmetic, so these are equality checks, not tolerance checks --
    # a single wrong shift or a missing clamp shows up as a hard mismatch.
    # SATU and MUTE take (ch, y), so they get the full 65536 pair sweep.
    bad = [(ch, y, satu_ch(ch, y), ref_satu(ch, y))
           for ch in range(256) for y in range(256)
           if satu_ch(ch, y) != ref_satu(ch, y)]
    res.add(not bad, "SATU == clamp(2*ch - y, 0, 255)",
            "all 65536 (ch,y) pairs" if not bad else "bad: %s" % (bad[:3],))

    bad = [(ch, y, mute_ch(ch, y), ref_mute(ch, y))
           for ch in range(256) for y in range(256)
           if mute_ch(ch, y) != ref_mute(ch, y)]
    res.add(not bad, "MUTE == (ch + y) >> 1",
            "all 65536 (ch,y) pairs" if not bad else "bad: %s" % (bad[:3],))

    for name, got, want in (
            ("AMBER green == (5*g)>>3", cast58, lambda v: ref_gain(5, v)),
            ("COOL green == (3*g)>>3", cast38, lambda v: ref_gain(3, v)),
            ("SOLAR == ch<128 ? ch : ~ch", solar_ch, ref_solar),
            ("POSTE == {ch[7:6], 6'b0}", lambda v: ((v >> 6) << 6), ref_poste)):
        bad = [(v, got(v), want(v)) for v in range(256) if got(v) != want(v)]
        res.add(not bad, name,
                "all 256 values" if not bad else "bad: %s" % (bad[:3],))

    # And the whole output mux, so a helper that is right on its own but wired
    # to the wrong sel still fails. Codes 13..15 are reserved and must pass
    # through untouched, which ref_new_effect reproduces by returning rgb.
    samples = [0x000000, 0xFFFFFF, 0x123456, 0xFF0000, 0x00FF00, 0x0000FF,
               0x808080, 0x7F7F7F, 0xFEDCBA, 0x070707, 0x010203]
    samples += [(r << 16) | (g << 8) | b for r, g, b in coarse_space(5)]
    for sel in range(6, 16):
        bad = [(hex(v), hex(effect(v, sel)), hex(ref_new_effect(v, sel)))
               for v in samples if effect(v, sel) != ref_new_effect(v, sel)]
        res.add(not bad, "sel=%d output mux == independent reference" % sel,
                "all %d colours" % len(samples) if not bad
                else "bad: %s" % (bad[:2],))

    c = sw_res['caps']
    print("      (max gray sum=%d, max S=%d, max T=%d)"
          % (c['gray_total'], c['sepia_s'], c['sepia_t']))


def pass_c(res):
    print("=" * 78)
    print("C. Accuracy vs textbook references")
    print("=" * 78)
    try:
        import numpy as np
    except ImportError:
        np = None

    if np is not None:
        idx = np.arange(256, dtype=np.int64)
        R, G, B = np.meshgrid(idx, idx, idx, indexing='ij')
        R, G, B = R.ravel(), G.ravel(), B.ravel()
        gy = (77 * R + 150 * G + 29 * B) >> 8
        ge = np.abs(gy - (0.299 * R + 0.587 * G + 0.114 * B))
        res.add(ge.max() <= 1.5, "grayscale vs BT.601 float",
                "max |err| = %.3f levels over all 16777216 colours" % ge.max())

        T = (R << 1) + (G << 2) + B
        S = (T << 3) + (T << 2) + (R + G)
        SW = S >> 6
        S8 = np.minimum(SW, 255)
        G8 = np.minimum(SW - (SW >> 3) + (SW >> 6), 255)
        B8 = np.minimum((SW >> 1) + (SW >> 3) + (SW >> 4) + (SW >> 7), 255)
        errs = []
        for i, mine in enumerate((S8, G8, B8)):
            ref = np.minimum(255.0, np.round(SEPIA_M[i][0] * R
                                             + SEPIA_M[i][1] * G
                                             + SEPIA_M[i][2] * B))
            errs.append(int(np.abs(mine - ref).max()))
        res.add(errs[0] <= 2 and errs[1] <= 3 and errs[2] <= 5,
                "sepia vs classic matrix, full gamut",
                "max |err| per channel = %s (bound 2/3/5)" % errs)
    else:
        wg = max(abs(gray_y(r, g, b)[0] - float_gray(r, g, b))
                 for r, g, b in coarse_space())
        ws = [0, 0, 0]
        for r, g, b in coarse_space():
            out = sepia_rgb(r, g, b)[0]
            ref = float_sepia(r, g, b)
            for i in range(3):
                ws[i] = max(ws[i], abs(out[i] - ref[i]))
        res.add(wg <= 1.5, "grayscale vs BT.601 float (coarse, no numpy)",
                "max |err| = %.3f" % wg)
        res.add(ws[0] <= 2 and ws[1] <= 3 and ws[2] <= 5,
                "sepia vs classic matrix (coarse, no numpy)",
                "max |err| per channel = %s" % ws)


def pass_d(res):
    print("=" * 78)
    print("D. Range, clamp and fixed-point invariants")
    print("=" * 78)
    oor = []
    for sel in range(16):
        for r, g, b in coarse_space(5):
            out = effect((r << 16) | (g << 8) | b, sel)
            if not 0 <= out <= 0xFFFFFF:
                oor.append((sel, r, g, b, hex(out)))
    bad_ch = [ch for ch in range(256) if not 0 <= contrast_ch(ch) <= 255]
    bad_y = [(r, g, b) for r, g, b in coarse_space(17)
             if not 0 <= gray_y(r, g, b)[0] <= 255]
    bad_sp = [(r, g, b) for r, g, b in coarse_space(17)
              if any(not 0 <= v <= 255 for v in sepia_rgb(r, g, b)[0])]
    res.add(not oor and not bad_ch and not bad_y and not bad_sp,
            "every channel helper and every mode stays in 0..255",
            "sel 0..15; contrast 0..255; gray+sepia coarse")

    pts = [(0, 0, 0), (255, 255, 255), (128, 128, 128)]
    fp = {0: 0, 64: 32, 128: 128, 192: 224, 255: 255}
    bad_fp = [(ch, contrast_ch(ch), want) for ch, want in fp.items()
              if contrast_ch(ch) != want]
    res.add(not bad_fp, "contrast fixed points 0/64/128/192/255",
            "-> %s" % [contrast_ch(c) for c in (0, 64, 128, 192, 255)]
            if not bad_fp else "bad (ch, got, want): %s" % bad_fp)

    checks = []
    for rgb in pts:
        v = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
        checks.append((rgb, "%06X" % effect(v, 4)))
    res.add(effect(0x000000, 4) == 0x000000 and effect(0xFFFFFF, 4) == 0xFFFFEC,
            "sepia black->black, white->cream FFFFFFEC",
            " ".join("%s->%s" % c for c in checks))

    res.add(effect(0x808080, 3) == 0xFFFFFF,
            "threshold: luma exactly 128 rounds to white (>= not >)",
            "(128,128,128) luma=%d -> %06X" % (gray_y(128, 128, 128)[0],
                                               effect(0x808080, 3)))
    res.add(effect(0x7F7F7F, 3) == 0x000000,
            "threshold: luma 127 rounds to black",
            "(127,127,127) luma=%d" % gray_y(127, 127, 127)[0])
    res.add(effect(0x123456, 2) == 0xEDCBA9, "invert is exact bitwise NOT",
            "123456 -> %06X" % effect(0x123456, 2))

    # A saturation knob must leave an already-neutral pixel alone, otherwise it
    # is adding a colour cast rather than changing saturation. 77+150+29 == 256
    # makes luma(v,v,v) == v exactly, so both ends of the knob are fixed there.
    greys = [(v << 16) | (v << 8) | v for v in range(0, 256, 7)] + [0x808080]
    bad = [hex(v) for v in greys
           if effect(v, 6) != v or effect(v, 7) != v]
    res.add(not bad, "SATU and MUTE both fix every neutral grey",
            "%d greys, no shift" % len(greys) if not bad else "bad: %s" % bad[:3])

    outs = [solar_ch(ch) for ch in range(256)]
    res.add(max(outs) == 127 and outs[127] == 127 and outs[128] == 127
            and outs[0] == 0 and outs[255] == 0,
            "SOLAR is a triangle peaking at 127",
            "0->%d 127->%d 128->%d 255->%d, max=%d"
            % (outs[0], outs[127], outs[128], outs[255], max(outs)))

    levels = sorted(set(((ch >> 6) << 6) for ch in range(256)))
    res.add(levels == [0, 64, 128, 192], "POSTE emits exactly four levels",
            "-> %s" % levels)

    res.add(effect(0xFFFFFF, 12) == 0x000000
            and effect(0x000000, 12) == 0xFFFFFF,
            "GINV maps white to black and black to white",
            "and is not INVT: GINV(0x123456) = %06X vs INVT %06X"
            % (effect(0x123456, 12), effect(0x123456, 2)))

    res.add(all(effect(0x123456, s) == 0x123456 for s in (13, 14, 15)),
            "reserved sel 13/14/15 fall through to passthrough",
            "the output mux's final I_rgb arm")


def pass_e(res):
    print("=" * 78)
    print("E. Intermediate widths stay inside the RTL's declared bit widths")
    print("=" * 78)
    declared = {'gray_total': 16, 'sepia_t': 11, 'sepia_rg': 9,
                'sepia_s': 15, 'sepia_sw': 9, 'sepia_gp': 9, 'sepia_bp': 9}
    caps = sweep()['caps']
    ok = True
    detail = []
    for k, bits in declared.items():
        mx = caps[k]
        if mx > (1 << bits) - 1:
            ok = False
        detail.append("%s %d/%db" % (k, mx, bits))
    res.add(ok, "no intermediate exceeds its declared width",
            " ".join(detail))
    res.add(caps['gray_total'] == 65280 and caps['sepia_s'] == 21930
            and caps['sepia_gp'] == 305 and caps['sepia_bp'] == 236,
            "peaks match the hand-derived bounds exactly",
            "gray 65280 (=255*256, no clamp needed), S 21930 (=255*86), "
            "gp 305 (clamped), bp 236 (fits 8 bits, no clamp needed)")

    # The seven new filters. Peaks are measured exhaustively, not asserted from
    # a hand-derived bound, so a wrong declaration fails here rather than on
    # the board.
    g5 = max((g << 2) + g for g in range(256))
    g3 = max((g << 1) + g for g in range(256))
    dup = ddn = up = 0
    dn_lo = dn_hi = 0
    sign_bad = []
    for ch in range(256):
        for y in range(256):
            if ch >= y:
                dup = max(dup, ch - y)
                up = max(up, y + ((ch - y) << 1))
            else:
                ddn = max(ddn, y - ch)
                dn = y - ((y - ch) << 1)
                dn_lo = min(dn_lo, dn)
                dn_hi = max(dn_hi, dn)
                # The RTL tests dn[9] on a 10 bit reg, so the whole dark-side
                # clamp rests on bit 9 being an exact sign test over the range
                # dn actually reaches, and on every non-negative dn fitting the
                # 8 bits that get returned.
                bits = dn & MASK10
                if ((bits >> 9) & 1) != (1 if dn < 0 else 0):
                    if len(sign_bad) < 3:
                        sign_bad.append((ch, y, dn, hex(bits)))
                elif dn > 255 and len(sign_bad) < 3:
                    sign_bad.append((ch, y, dn, "non-negative but >8 bits"))
    msum = max(ch + y for ch in range(256) for y in range(256))

    widths = [('g_mul5', g5, 11), ('g_mul3', g3, 10), ('satu dup', dup, 9),
              ('satu ddn', ddn, 9), ('satu up', up, 10), ('mute sum', msum, 9)]
    over = ["%s %d>%db" % (n, v, b) for n, v, b in widths if v > (1 << b) - 1]
    res.add(not over, "new-filter intermediates fit their declared widths",
            " ".join("%s %d/%db" % (n, v, b) for n, v, b in widths))

    res.add(dn_lo >= -512 and dn_hi <= 511 and not sign_bad,
            "satu dn[9] is an exact sign test over all 65536 (ch,y) pairs",
            "dn range [%d, %d] inside 10 bit two's complement" % (dn_lo, dn_hi)
            if not sign_bad else "bad: %s" % (sign_bad,))

    res.add(g5 == 1275 and g3 == 765 and up == 510 and msum == 510,
            "new-filter peaks match the hand-derived bounds",
            "g_mul5 1275 (=5*255), g_mul3 765 (=3*255), "
            "satu up 510 (=2*255, clamped), mute sum 510 (>>1, no clamp)")


def pass_f(res):
    print("=" * 78)
    print("F. Negative controls -- every mutant MUST be caught")
    print("=" * 78)

    caught = any(mut_gray_coef76(r, g, b) != gray_y(r, g, b)[0]
                 for r, g, b in coarse_space(11))
    res.add(caught, "MUT gray coefficient 76 instead of 77",
            "caught by Pass B exact-equality vs 77R+150G+29B")

    caught = any(min(mut_sepia_noclamp(r, g, b)) != min(sepia_rgb(r, g, b)[0])
                 or mut_sepia_noclamp(r, g, b) != sepia_rgb(r, g, b)[0]
                 for r, g, b in coarse_space(11))
    res.add(caught, "MUT sepia sw clamp removed",
            "white: mutant %s vs correct %s"
            % (mut_sepia_noclamp(255, 255, 255), sepia_rgb(255, 255, 255)[0]))

    worst = max(abs(mut_sepia_gain_from_clamped(r, g, b)[i] - float_sepia(r, g, b)[i])
                for r, g, b in coarse_space(7) for i in range(3))
    res.add(worst > 5, "MUT sepia gains taken from CLAMPED s8",
            "full-gamut max |err| vs matrix = %d levels (correct design: <=5)"
            % worst)

    caught = any(mut_contrast_no_low_clamp(ch) != contrast_ch(ch)
                 for ch in range(256))
    res.add(caught, "MUT contrast low side without the underflow clamp",
            "ch=0: mutant %d vs correct %d"
            % (mut_contrast_no_low_clamp(0), contrast_ch(0)))

    caught = mut_thresh_strict(gray_y(128, 128, 128)[0]) != effect(0x808080, 3)
    res.add(caught, "MUT threshold using > instead of >=",
            "luma 128: mutant %06X vs correct %06X"
            % (mut_thresh_strict(128), effect(0x808080, 3)))

    caught = any(mut_sepia_forget_shift(r, g, b) != sepia_rgb(r, g, b)[0]
                 for r, g, b in coarse_space(11))
    res.add(caught, "MUT sepia forgetting the >>6 scaling",
            "mid grey: mutant %s vs correct %s"
            % (mut_sepia_forget_shift(128, 128, 128),
               sepia_rgb(128, 128, 128)[0]))

    # --- the seven new filters -------------------------------------------
    wit = next(((ch, y) for ch in range(256) for y in range(256)
                if mut_mute_no_shift(ch, y) != mute_ch(ch, y)), None)
    res.add(wit is not None, "MUT MUTE forgetting the >>1",
            "(ch=%d,y=%d): mutant %d vs correct %d"
            % (wit[0], wit[1], mut_mute_no_shift(*wit), mute_ch(*wit))
            if wit else "not caught")

    wit = next(((ch, y) for ch in range(256) for y in range(256)
                if mut_satu_no_high_clamp(ch, y) != satu_ch(ch, y)), None)
    res.add(wit is not None, "MUT SATU without the overflow clamp",
            "(ch=%d,y=%d): mutant %d vs correct %d"
            % (wit[0], wit[1], mut_satu_no_high_clamp(*wit), satu_ch(*wit))
            if wit else "not caught")

    wit = next(((ch, y) for ch in range(256) for y in range(256)
                if mut_satu_no_low_clamp(ch, y) != satu_ch(ch, y)), None)
    res.add(wit is not None, "MUT SATU without the underflow clamp",
            "(ch=%d,y=%d): mutant %d vs correct %d"
            % (wit[0], wit[1], mut_satu_no_low_clamp(*wit), satu_ch(*wit))
            if wit else "not caught")

    wit = next((g for g in range(256)
                if mut_amber_compounded_floors(g) != cast58(g)), None)
    res.add(wit is not None,
            "MUT AMBER green as (g>>1)+(g>>3), compounded floors",
            "first diverges at g=%d: mutant %d vs exact %d (=floor(5g/8))"
            % (wit, mut_amber_compounded_floors(wit), cast58(wit))
            if wit is not None else "not caught")

    cs = [(r << 16) | (g << 8) | b for r, g, b in coarse_space(11)]
    wit = next(((v, s) for v in cs for s in (8, 9)
                if mut_amber_cool_swapped(v, s) != effect(v, s)), None)
    res.add(wit is not None, "MUT AMBER and COOL gain vectors swapped",
            "sel=%d on %06X: mutant %06X vs correct %06X"
            % (wit[1], wit[0], mut_amber_cool_swapped(*wit), effect(*wit))
            if wit else "not caught")

    wit = next((v for v in cs if mut_amber_cool_shared(v, 9) != effect(v, 9)), None)
    res.add(wit is not None, "MUT AMBER and COOL sharing one green gain",
            "COOL on %06X: mutant %06X vs correct %06X (3/8, not 5/8)"
            % (wit, mut_amber_cool_shared(wit, 9), effect(wit, 9))
            if wit else "not caught")

    wit = next((ch for ch in range(256)
                if mut_poste_3bit(ch) != ((ch >> 6) << 6)), None)
    res.add(wit is not None, "MUT POSTE truncating [7:5] instead of [7:6]",
            "ch=%d: mutant %d (8 levels) vs correct %d (4 levels)"
            % (wit, mut_poste_3bit(wit), (wit >> 6) << 6)
            if wit is not None else "not caught")

    wit = next((ch for ch in range(256)
                if mut_solar_inclusive(ch) != solar_ch(ch)), None)
    res.add(wit is not None, "MUT SOLAR using <=128 instead of <128",
            "ch=%d: mutant %d vs correct %d"
            % (wit, mut_solar_inclusive(wit), solar_ch(wit))
            if wit is not None else "not caught")

    wit = None
    for v in cs:
        r, g, b = (v >> 16) & MASK8, (v >> 8) & MASK8, v & MASK8
        mut = mut_ginv_per_channel(r, g, b)
        got = tuple((effect(v, 12) >> s) & MASK8 for s in (16, 8, 0))
        if mut != got:
            wit = (v, mut, got)
            break
    res.add(wit is not None, "MUT GINV inverting per channel instead of luma",
            "%06X: mutant %02X%02X%02X (=INVT) vs correct %02X%02X%02X"
            % ((wit[0],) + wit[1] + wit[2]) if wit else "not caught")


def main():
    res = Result()
    pass_a(res)
    pass_b(res)
    pass_c(res)
    pass_d(res)
    pass_e(res)
    pass_f(res)
    print("=" * 78)
    total = len(res.rows)
    print("%d/%d checks passed, %d failed" % (total - res.failed, total,
                                              res.failed))
    print("=" * 78)
    sys.exit(1 if res.failed else 0)


if __name__ == "__main__":
    main()
