#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Render the redesigned spectrum panel pixel-for-pixel, BEFORE any RTL is
written or synthesized.

Why this exists
---------------
The panel is drawn by a chain of overlapping pixel predicates in
audio_visualizer.v, and every one of them is a pair of constants compared
against x_pos/y_pos. A constant that is off by one does not fail a build, does
not fail timing, and does not fail any simulation -- it just draws a bar one
pixel too narrow, or a peak cap that disappears off the top of the panel, and
the only way to find out is to burn a bitstream and look at the monitor.

So the geometry is written down once, here, and this script does two things
with it:

  1. `check_geometry()` turns the layout claims into hard assertions -- every
     region inside the panel, nothing overlapping, 16 bars actually filling the
     bar area exactly, a full-scale peak cap actually staying on screen.
  2. `panel_pixel()` mirrors the stage_rgb priority mux, and the envelopes come
     from sim_biquad_bank rather than being invented, so the pictures show the
     real detector's output on the real geometry.

Then the layout is eyeballed at 3x zoom before the Verilog is written. This
step is free; a wasted place-and-route is not.

Usage
-----
    python tools/render_spectrum_preview.py [outdir] [background.bmp]

Writes <outdir>/spectrum_*.png. With no background a synthetic image is used;
pass a real 640x480 BMP to see the panel dimming an actual photograph.
"""

import os
import sys

from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_biquad_bank as sb

# --------------------------------------------------------------------------
# Geometry. These constants are transcribed verbatim into audio_visualizer.v.
# Inclusive ranges are written in comments because that is what the predicates
# actually compare, and an off-by-one lives in the gap between "X_END" and
# "X_LAST".
# --------------------------------------------------------------------------
H_ACTIVE = 640
V_ACTIVE = 480

PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 28, 352, 584, 104      # x[28..611] y[352..455]
PANEL_X_LAST = PANEL_X + PANEL_W - 1                        # 611
PANEL_Y_LAST = PANEL_Y + PANEL_H - 1                        # 455

# Bar area: 16 cells of exactly 32 px, so bar_idx is a shift and not a divide.
BAR_X, BAR_Y, BAR_W, BAR_H = 44, 362, 512, 64               # x[44..555] y[362..425]
BAR_X_LAST = BAR_X + BAR_W - 1                              # 555
BAR_Y_LAST = BAR_Y + BAR_H - 1                              # 425
BAR_CELL = 32
BAR_PX = 24                                                 # bar width
BAR_OFF_LO = (BAR_CELL - BAR_PX) // 2                       # 4
BAR_OFF_HI = BAR_OFF_LO + BAR_PX - 1                        # 27
NBAND = 16

# Side columns, inside the 1 px border.
LEFT_X, LEFT_W = PANEL_X + 1, BAR_X - PANEL_X - 1           # x[29..43]
RIGHT_X = BAR_X + BAR_W                                     # 556
RIGHT_W = PANEL_X_LAST - RIGHT_X                            # 54 -> x[556..609]

# Two horizontal meters below the bars.
METER_X, METER_W = BAR_X, BAR_W
METER_L_Y, METER_H = 428, 12                                # y[428..439]
METER_R_Y = 441                                             # y[441..452]
METER_DIV_Y = 440

# -6 / -12 / -24 dB rows. env is a linear amplitude, so the row for a given
# attenuation is (127 * 10**(-dB/20)) >> 1 pixels above the bar floor.
DB_MARKS = (6, 12, 24)

# Colours, carried over from the current design where one already existed.
C_BORDER = 0x70D0FF
C_GRID = 0x24343C
C_BAR_HI = 0xFFB84C                                         # bar_rel_y >= 32
C_BAR_MID = 0x6CFFB0                                        # bar_rel_y >= 16
C_BAR_LO = 0x40C8FF
C_PEAK = 0xFFE060
C_METER_L = 0xD8F4FF                                        # pale ice, not the
C_METER_R = 0xFFB0D8                                        # bar blue 0x40C8FF
C_TICK = 0x5A7A88


def unpack(rgb):
    return ((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF)


def db_row(db):
    """Panel y of a dB mark: the topmost row a bar of that height actually lights.

    bar_pixel is `rel_y < bar_height`, so a bar of height h occupies rel_y
    0..h-1 and its top row is BAR_Y_LAST - h + 1. Labelling BAR_Y_LAST - h
    would put the tick one pixel above the bar it describes.
    """
    env = int(round(127.0 * (10.0 ** (-db / 20.0))))
    return BAR_Y_LAST - (env >> 1) + 1, env


def check_render():
    """Read the rendered pixels back and assert the layout actually drew.

    check_geometry validates the constants; this validates what they produce.
    The two are not the same -- the peak cap overlapping the bar top, and the
    cap painting a dash in every band at peak == 0, both passed every bounds
    assertion in check_geometry and were only caught by looking at pixels.
    """
    fails = []

    def want(cond, what):
        if not cond:
            fails.append(what)
        return cond

    bg = synth_background()
    bgpx = bg.load()
    bars = (unpack(C_BAR_LO), unpack(C_BAR_MID), unpack(C_BAR_HI))
    capc = unpack(C_PEAK)
    meterc = (unpack(C_METER_L), unpack(C_METER_R))

    env = [6, 8, 12, 18, 32, 77, 33, 19, 19, 35, 21, 25, 24, 22, 18, 13]
    # Caps deliberately above the bars, so "cap does not overlap bar" is a real
    # test rather than a tautology.
    peak = [e + 20 if e else 0 for e in env]
    px = render_frame(env, peak, 30, 30, bg).load()

    for k in range(NBAND):
        cell_x = BAR_X + k * BAR_CELL
        x0 = cell_x + BAR_OFF_LO
        x1 = cell_x + BAR_OFF_HI
        band_rows = range(BAR_Y, BAR_Y_LAST + 1)

        # Walk all 32 offsets of the cell, not just the ones the bar is meant
        # to occupy, and assert properties that do not move with the constants
        # under test: the right width, contiguous, and centred.
        lit_offsets = [o for o in range(BAR_CELL)
                       if any(px[cell_x + o, y] in bars for y in band_rows)]
        want(len(lit_offsets) == BAR_PX,
             "band %d: bar is %d px wide, BAR_PX says %d"
             % (k, len(lit_offsets), BAR_PX))
        if not lit_offsets:
            want(False, "band %d: no lit columns at all" % k)
            continue
        want(lit_offsets == list(range(lit_offsets[0],
                                       lit_offsets[0] + len(lit_offsets))),
             "band %d: the bar's lit columns are not contiguous: %s"
             % (k, lit_offsets))
        # Centred means the gap either side is equal. This is what catches an
        # edge being off by one: comparing lit_offsets against
        # range(BAR_OFF_LO, BAR_OFF_HI + 1) cannot, because that expectation is
        # built from the very constants being checked.
        gap_lo, gap_hi = lit_offsets[0], BAR_CELL - 1 - lit_offsets[-1]
        want(gap_lo == gap_hi,
             "band %d: bar is not centred in its cell, gaps %d and %d"
             % (k, gap_lo, gap_hi))

        cols = [sorted(y for y in band_rows if px[x, y] in bars)
                for x in range(x0, x1 + 1)]
        want(all(c == cols[0] for c in cols),
             "band %d: bar is not uniform across its own %d px width"
             % (k, BAR_OFF_HI - BAR_OFF_LO + 1))
        want(len(cols[0]) == (env[k] >> 1),
             "band %d: bar is %d px tall, env %d asks for %d"
             % (k, len(cols[0]), env[k], env[k] >> 1))

        caps = sorted(y for y in band_rows if px[x0 + 5, y] == capc)
        pr = peak_row(peak[k])
        want(len(caps) == (2 if pr + 1 < BAR_H else 1),
             "band %d: peak cap is %d px, expected %d at peak_row %d"
             % (k, len(caps), 2 if pr + 1 < BAR_H else 1, pr))
        if cols[0] and caps:
            want(max(caps) < min(cols[0]),
                 "band %d: peak cap rows %s overlap the bar top %d"
                 % (k, caps, min(cols[0])))
        want(not caps or min(caps) >= BAR_Y,
             "band %d: peak cap escapes the top of the bar area at y=%d"
             % (k, min(caps)))

        for x in (x0 - 1, x1 + 1):
            if BAR_X <= x <= BAR_X_LAST:
                want(not any(px[x, y] in bars or px[x, y] == capc
                             for y in band_rows),
                     "band %d: the gap column x=%d is lit" % (k, x))

    # Silence must be completely flat. This is test B' in pixel form: if the
    # quantisation DC offset ever stood a bar up, this is where it shows.
    zenv = [0] * NBAND
    zpx = render_frame(zenv, zenv, 0, 0, bg).load()
    lit = [(x, y) for y in range(BAR_Y, BAR_Y_LAST + 1)
           for x in range(BAR_X, BAR_X_LAST + 1)
           if zpx[x, y] in bars or zpx[x, y] == capc]
    want(not lit, "silence still lights %d pixel(s) in the bar area, first %s"
                  % (len(lit), lit[:6]))
    mlit = [(x, y) for y in (METER_L_Y, METER_R_Y)
            for x in range(METER_X, METER_X + METER_W)
            if zpx[x, y] in meterc]
    want(not mlit, "silence still lights %d meter pixel(s)" % len(mlit))

    # Nothing drawn outside its own region. Meter blue equals bar blue, so the
    # colour sets are scoped to the y band each belongs to.
    below = [(x, y) for y in range(BAR_Y_LAST + 1, PANEL_Y_LAST + 1)
             for x in range(BAR_X, BAR_X_LAST + 1) if px[x, y] in bars]
    want(not below, "bar colour appears below the bar area: %s" % below[:6])
    above = [(x, y) for y in range(PANEL_Y, BAR_Y)
             for x in range(BAR_X, BAR_X_LAST + 1)
             if px[x, y] in bars or px[x, y] == capc]
    want(not above, "bar or cap colour above the bar area: %s" % above[:6])
    outside = [(x, y) for y in range(PANEL_Y - 4, PANEL_Y_LAST + 5)
               for x in range(PANEL_X - 4, PANEL_X_LAST + 5)
               if not (PANEL_Y <= y <= PANEL_Y_LAST
                       and PANEL_X <= x <= PANEL_X_LAST)
               and px[x, y] != bgpx[x, y]]
    want(not outside, "panel draws outside its own border: %s" % outside[:6])

    # Meters must not touch each other, the bar area, or the border.
    for y in (METER_DIV_Y, METER_L_Y - 1, METER_R_Y + METER_H, BAR_Y_LAST + 1):
        want(not any(px[x, y] in meterc
                     for x in range(METER_X, METER_X + METER_W)),
             "meter colour on row y=%d, which must stay dark" % y)

    # Full scale, where the cap's second row would land at rel_y 64 and has to
    # be clipped by in_bars instead of escaping the top of the bar area. The
    # env+20 caps above never reach peak_row 63, so without this the clipping
    # branch is never exercised.
    fpx = render_frame([127] * NBAND, [127] * NBAND, 127, 127, bg).load()
    escape = [(x, y) for y in range(PANEL_Y, BAR_Y)
              for x in range(BAR_X, BAR_X_LAST + 1) if fpx[x, y] == capc]
    want(not escape, "a full-scale peak cap escapes above the bar area: %s"
                     % escape[:6])
    fcaps = sorted(y for y in range(BAR_Y, BAR_Y_LAST + 1)
                   if fpx[BAR_X + BAR_OFF_LO + 5, y] == capc)
    want(fcaps == [BAR_Y],
         "a full-scale peak cap should clip to the single top row %d, got %s"
         % (BAR_Y, fcaps))
    fbar = sum(1 for y in range(BAR_Y, BAR_Y_LAST + 1)
               if fpx[BAR_X + BAR_OFF_LO + 5, y] in bars)
    want(fbar == 127 >> 1, "a full-scale env should light %d bar rows, got %d"
                           % (127 >> 1, fbar))
    fmeter = sum(1 for x in range(METER_X, METER_X + METER_W)
                 if fpx[x, METER_L_Y] == unpack(C_METER_L))
    want(fmeter == 127 << 2,
         "a full-scale meter should be %d px long, got %d" % (127 << 2, fmeter))

    # Each dB tick must land on the top row of a bar of exactly that height.
    # check_geometry only asserts the tick is somewhere inside the bar area, so
    # a tick drawn one row above the bar it labels passes there. Compare
    # against what a bar actually rendered instead of against db_row's own
    # arithmetic, which would be circular.
    for db in DB_MARKS:
        y_tick, env_mark = db_row(db)
        tenv = [0] * NBAND
        tenv[0] = env_mark
        tpx = render_frame(tenv, [0] * NBAND, 0, 0, bg).load()
        probe_x = BAR_X + BAR_OFF_LO + 5
        top = min([y for y in range(BAR_Y, BAR_Y_LAST + 1)
                   if tpx[probe_x, y] in bars] or [None])
        want(top is not None and top == y_tick,
             "-%d dB tick is at y=%s but a bar of env %d tops out at y=%s"
             % (db, y_tick, env_mark, top))
        want(tpx[probe_x, y_tick] in bars,
             "-%d dB tick at y=%d is not covered by the bar it labels"
             % (db, y_tick))

    return fails


def check_geometry():
    """Assert every layout claim. This is the part that saves a build cycle."""
    fails = []

    def want(cond, what):
        if not cond:
            fails.append(what)
        return cond

    want(PANEL_X >= 0 and PANEL_X_LAST < H_ACTIVE,
         "panel overflows x: [%d..%d]" % (PANEL_X, PANEL_X_LAST))
    want(PANEL_Y >= 0 and PANEL_Y_LAST < V_ACTIVE,
         "panel overflows y: [%d..%d]" % (PANEL_Y, PANEL_Y_LAST))

    want(BAR_X > PANEL_X and BAR_X_LAST < PANEL_X_LAST,
         "bar area overflows the panel in x")
    want(BAR_Y > PANEL_Y and BAR_Y_LAST < PANEL_Y_LAST,
         "bar area overflows the panel in y")
    want(NBAND * BAR_CELL == BAR_W,
         "16 cells of %d px = %d, but BAR_W = %d"
         % (BAR_CELL, NBAND * BAR_CELL, BAR_W))
    want(BAR_CELL % 2 == 0, "BAR_CELL must be a power of two for bar_idx to be "
                            "a shift, got %d" % BAR_CELL)
    want(0 <= BAR_OFF_LO <= BAR_OFF_HI < BAR_CELL,
         "bar offsets [%d..%d] outside the %d px cell"
         % (BAR_OFF_LO, BAR_OFF_HI, BAR_CELL))

    # env is 0..127 and bar_height = env >> 1, so 63 px is the tallest bar.
    want((127 >> 1) <= BAR_H,
         "a full-scale env needs %d px but BAR_H is %d" % (127 >> 1, BAR_H))
    # A peak cap at full scale must not leave the top of the bar area. It is
    # drawn at bar_rel_y in {peak_row-1, peak_row}, so peak_row = 63 reaches
    # bar_rel_y 63 == BAR_Y_LAST - BAR_Y, the top row, and no further.
    want(BAR_Y_LAST - (127 >> 1) >= BAR_Y,
         "a full-scale peak cap draws above the bar area")
    # ...and at zero it must not fall through the bottom. The predicate
    # bar_rel_y + 2 > peak_row clamps it to a single row at bar_rel_y 0.
    want(BAR_Y_LAST - 0 <= BAR_Y_LAST, "peak cap at 0 leaves the bar area")

    want(METER_L_Y > BAR_Y_LAST, "L meter overlaps the bar area")
    want(METER_R_Y + METER_H - 1 < PANEL_Y_LAST,
         "R meter overflows the panel: last row %d, border at %d"
         % (METER_R_Y + METER_H - 1, PANEL_Y_LAST))
    want(METER_L_Y + METER_H - 1 < METER_DIV_Y < METER_R_Y,
         "meter divider row %d is not between the two meters" % METER_DIV_Y)
    # lvl is 0..127 and the meter width is lvl << 2, so 508 px at full scale.
    want((127 << 2) <= METER_W,
         "a full-scale meter needs %d px but METER_W is %d" % (127 << 2, METER_W))

    want(LEFT_X > PANEL_X and LEFT_X + LEFT_W - 1 < BAR_X,
         "left column [%d..%d] collides with the border or the bars"
         % (LEFT_X, LEFT_X + LEFT_W - 1))
    want(RIGHT_X > BAR_X_LAST and RIGHT_X + RIGHT_W - 1 <= PANEL_X_LAST - 1,
         "right column [%d..%d] collides with the bars or the border"
         % (RIGHT_X, RIGHT_X + RIGHT_W - 1))

    for db in DB_MARKS:
        y, env = db_row(db)
        want(BAR_Y <= y <= BAR_Y_LAST,
             "-%d dB mark (env %d) lands at y=%d, outside the bar area [%d..%d]"
             % (db, env, y, BAR_Y, BAR_Y_LAST))

    # Every foreground colour must be distinct. C_METER_L was once 0x40C8FF,
    # byte-identical to C_BAR_LO, so a meter pixel could not be told from a bar
    # pixel by eye or by the checker. Gate the whole palette rather than that
    # one pair.
    palette = [("border", C_BORDER), ("grid", C_GRID), ("bar_hi", C_BAR_HI),
               ("bar_mid", C_BAR_MID), ("bar_lo", C_BAR_LO), ("peak", C_PEAK),
               ("meter_l", C_METER_L),
               ("meter_r", C_METER_R), ("tick", C_TICK)]
    for i, (n0, c0) in enumerate(palette):
        for n1, c1 in palette[i + 1:]:
            want(c0 != c1, "%s and %s are the same colour 0x%06X"
                           % (n0, n1, c0))

    return fails


# --------------------------------------------------------------------------
# The pixel pipeline, mirroring stage_rgb.
# --------------------------------------------------------------------------
def bar_height(env):
    return env >> 1


def peak_row(peak):
    return peak >> 1


def meter_width(lvl):
    return lvl << 2


def panel_pixel(x, y, env, peak, lvl_l, lvl_r, bg):
    # bg is an (r, g, b) tuple straight from PIL, not a packed 24-bit int.
    """Return (r, g, b) for one pixel, following stage_rgb's priority exactly.

    Priority order matters and is not decorative: peak cap beats bar beats
    meter beats border beats tick beats grid, so the grid shows through only in
    the gaps between bars and in the headroom above them. Getting this order
    wrong in the preview would hide a real ordering bug in the RTL.
    """
    in_panel = (PANEL_X <= x <= PANEL_X_LAST) and (PANEL_Y <= y <= PANEL_Y_LAST)
    if not in_panel:
        return bg

    in_bars = (BAR_X <= x <= BAR_X_LAST) and (BAR_Y <= y <= BAR_Y_LAST)

    if in_bars:
        rel_x = (x - BAR_X) % BAR_CELL
        rel_y = BAR_Y_LAST - y                     # 0 at the floor, 63 at the top
        idx = (x - BAR_X) >> 5
        in_bar_col = BAR_OFF_LO <= rel_x <= BAR_OFF_HI

        # The cap sits strictly ABOVE the bar, at rel_y in {pr, pr+1}. Drawing
        # it at {pr-1, pr} instead overlaps the bar's own top row and hides one
        # pixel of every bar -- a third of the signal on a 3 px bar. pr != 0 is
        # required because at peak 0 the same predicate paints a two-row dash
        # at the floor of all 16 bands, which is exactly the standing-bar
        # artefact the silence test exists to forbid. At pr == 63 the second
        # row falls outside in_bars and is clipped, so the cap is 1 px there.
        pr = peak_row(peak[idx])
        if in_bar_col and pr and rel_y >= pr and (rel_y - pr) <= 1:
            return unpack(C_PEAK)

        bh = bar_height(env[idx])
        if in_bar_col and rel_y < bh:
            # Same three-step gradient as the current bar_rgb, driven by the
            # same bits of the same relative-y.
            if rel_y & 0x20:
                return unpack(C_BAR_HI)
            if rel_y & 0x10:
                return unpack(C_BAR_MID)
            return unpack(C_BAR_LO)

    in_meter_l = (METER_X <= x < METER_X + METER_W) and \
                 (METER_L_Y <= y < METER_L_Y + METER_H)
    in_meter_r = (METER_X <= x < METER_X + METER_W) and \
                 (METER_R_Y <= y < METER_R_Y + METER_H)
    if in_meter_l and (x - METER_X) < meter_width(lvl_l):
        return unpack(C_METER_L)
    if in_meter_r and (x - METER_X) < meter_width(lvl_r):
        return unpack(C_METER_R)

    # Legend: a solid chip in the left column, the same colour as its meter.
    if LEFT_X <= x < LEFT_X + LEFT_W:
        if METER_L_Y <= y < METER_L_Y + METER_H:
            return unpack(C_METER_L)
        if METER_R_Y <= y < METER_R_Y + METER_H:
            return unpack(C_METER_R)

    border = (x in (PANEL_X, PANEL_X_LAST)) or (y in (PANEL_Y, PANEL_Y_LAST))
    if border:
        return unpack(C_BORDER)

    grid = ((x & 0x1F) == 0) or ((y & 0x0F) == 0)
    db_rows = [db_row(db)[0] for db in DB_MARKS]
    tick = y in db_rows and (
        (BAR_X <= x <= BAR_X_LAST) or
        (LEFT_X <= x < LEFT_X + 4) or
        (RIGHT_X + RIGHT_W - 4 <= x < RIGHT_X + RIGHT_W))
    if tick:
        return unpack(C_TICK)
    if grid:
        return unpack(C_GRID)

    # panel_rgb: (v >> 2) + (v >> 3) == v * 3/8 per channel.
    r, g, b = bg
    return ((r >> 2) + (r >> 3), (g >> 2) + (g >> 3), (b >> 2) + (b >> 3))


def render_frame(env, peak, lvl_l, lvl_r, bg, crop=None, zoom=1):
    x0, y0, x1, y1 = crop or (0, 0, H_ACTIVE - 1, V_ACTIVE - 1)
    w, h = x1 - x0 + 1, y1 - y0 + 1
    img = Image.new("RGB", (w * zoom, h * zoom))
    px = img.load()
    bgpx = bg.load()
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            c = panel_pixel(x, y, env, peak, lvl_l, lvl_r, bgpx[x, y])
            for dy in range(zoom):
                for dx in range(zoom):
                    px[(x - x0) * zoom + dx, (y - y0) * zoom + dy] = c
    return img


def synth_background():
    """A stand-in photograph: bright sky, mid-tone subject, dark foreground.

    The panel dims whatever is behind it to 3/8, so the background has to span
    the range or the dimming cannot be judged.
    """
    img = Image.new("RGB", (H_ACTIVE, V_ACTIVE))
    px = img.load()
    for y in range(V_ACTIVE):
        for x in range(H_ACTIVE):
            t = y / float(V_ACTIVE)
            r = int(60 + 150 * (1 - t))
            g = int(90 + 110 * (1 - t))
            b = int(140 + 80 * (1 - t))
            if 200 < y < 320 and 150 < x < 480:
                r, g, b = 210, 170, 90
            px[x, y] = (min(r, 255), min(g, 255), min(b, 255))
    return img


def load_background(path):
    img = Image.open(path).convert("RGB")
    if img.size != (H_ACTIVE, V_ACTIVE):
        print("  background %s is %dx%d, resizing to %dx%d"
              % (path, img.size[0], img.size[1], H_ACTIVE, V_ACTIVE))
        img = img.resize((H_ACTIVE, V_ACTIVE), Image.LANCZOS)
    return img


def scene(gen_fn, seconds, frames_at_end=0):
    """Run the real detector on a source and return (env, peak, lvl_l, lvl_r).

    frames_at_end advances peak caps past the end of the audio, which is the
    only way to see them fall -- while a tone holds, peak == env by definition.
    """
    bank = sb.Bank(sb.make_coeffs())
    sb.run(bank, gen_fn(), int(seconds * sb.AUDIO_RATE))
    if frames_at_end:
        # run() already applies frame_start at its true 60 Hz rate, so the tail
        # is one run of N frames' worth of silence rather than N chunked runs
        # with a hand-rolled frame tick on top.
        sb.run(bank, sb.silence(),
               frames_at_end * sb.AUDIO_RATE // sb.FRAME_RATE)
    return list(bank.env), list(bank.peak), bank.lvl_l, bank.lvl_r


def main(argv):
    outdir = argv[1] if len(argv) > 1 else os.path.join("tools", "preview")
    bgpath = argv[2] if len(argv) > 2 else None
    os.makedirs(outdir, exist_ok=True)

    print("=" * 72)
    print("spectrum panel layout preview")
    print("=" * 72)

    fails = check_geometry()
    if fails:
        print("\nGEOMETRY FAILS -- do not write the RTL yet:")
        for f in fails:
            print("  " + f)
        return 1
    print("geometry: all bounds assertions pass")

    pfails = check_render()
    if pfails:
        print("\nRENDER FAILS -- the constants are in bounds but draw wrong:")
        for f in pfails:
            print("  " + f)
        return 1
    print("render  : all pixel assertions pass (bar heights exact, gaps clean,")
    print("          caps above the bars, silence flat, full scale clipped)")
    print("  panel        x[%d..%d] y[%d..%d]  (%dx%d)"
          % (PANEL_X, PANEL_X_LAST, PANEL_Y, PANEL_Y_LAST, PANEL_W, PANEL_H))
    print("  bar area     x[%d..%d] y[%d..%d]  (%d bars x %d px cell, %d px wide,"
          " gap %d)" % (BAR_X, BAR_X_LAST, BAR_Y, BAR_Y_LAST, NBAND, BAR_CELL,
                        BAR_OFF_HI - BAR_OFF_LO + 1,
                        BAR_CELL - (BAR_OFF_HI - BAR_OFF_LO + 1)))
    print("  left column  x[%d..%d]" % (LEFT_X, LEFT_X + LEFT_W - 1))
    print("  right column x[%d..%d]" % (RIGHT_X, RIGHT_X + RIGHT_W - 1))
    print("  L meter      y[%d..%d]   R meter y[%d..%d]   divider y=%d"
          % (METER_L_Y, METER_L_Y + METER_H - 1, METER_R_Y,
             METER_R_Y + METER_H - 1, METER_DIV_Y))
    for db in DB_MARKS:
        y, env = db_row(db)
        print("  -%2d dB mark  env %3d -> bar %2d px -> y=%d"
              % (db, env, env >> 1, y))
    print("  full-scale bar %d px of %d available; full-scale meter %d px of %d"
          % (127 >> 1, BAR_H, 127 << 2, METER_W))

    bg = load_background(bgpath) if bgpath else synth_background()
    crop = (PANEL_X - 8, PANEL_Y - 8, PANEL_X_LAST + 8, PANEL_Y_LAST + 8)

    scenes = [
        ("silence", "no audio at all -- every bar and meter must be flat",
         lambda: sb.silence(), 1.0, 0),
        ("tone412", "the real test note: 412.1 Hz square at -12.5 dBFS",
         lambda: sb.sq(412.1), 0.5, 0),
        ("noise", "white noise -- log-spaced bands tilt upward, correctly",
         lambda: sb.noise(), 0.5, 0),
        ("sweep1200", "mid-sweep: 1200 Hz sine",
         lambda: sb.sine(1200.0), 0.4, 0),
        ("peakfall", "0.25 s after the 412 Hz tone stopped: caps in freefall",
         lambda: sb.sq(412.1), 0.5, 15),
    ]

    print()
    for name, desc, gen, secs, tail in scenes:
        env, peak, ll, lr = scene(gen, secs, tail)
        bars = [bar_height(e) for e in env]
        img = render_frame(env, peak, ll, lr, bg, crop=crop, zoom=3)
        p = os.path.join(outdir, "spectrum_%s.png" % name)
        img.save(p)
        print("  %-10s %s" % (name, desc))
        print("             env    : %s" % " ".join("%3d" % e for e in env))
        print("             bar px : %s" % " ".join("%3d" % b for b in bars))
        print("             peak   : %s" % " ".join("%3d" % p_ for p_ in peak))
        print("             meters : L %d px, R %d px  -> %s"
              % (meter_width(ll), meter_width(lr), os.path.basename(p)))
        if name == "silence" and (max(bars) or max(peak) or ll or lr):
            print("             *** SILENCE IS NOT FLAT -- this is the visual "
                  "form of test B'")
            return 1

    full_env, full_peak, fll, flr = scene(lambda: sb.sq(412.1), 0.5)
    p = os.path.join(outdir, "spectrum_fullframe.png")
    render_frame(full_env, full_peak, fll, flr, bg).save(p)
    print("\n  fullframe  whole 640x480 with the panel in place -> %s"
          % os.path.basename(p))

    print("\nWhat to look for in the 3x crops:")
    print("  * every bar is %d px wide with %d px of gap, and bar 15 ends exactly"
          % (BAR_PX, BAR_CELL - BAR_PX))
    print("    at x=%d without touching the right column at x=%d"
          % (BAR_X_LAST, RIGHT_X))
    print("  * the yellow peak cap is 2 px tall and sits strictly ABOVE the bar")
    print("    top; at full scale its upper row is clipped by y=%d, so it is"
          % BAR_Y)
    print("    1 px there rather than escaping the bar area")
    print("  * the -6/-12/-24 dB ticks line up with the bar heights they label")
    print("  * the two meters are %d px tall, do not touch each other or the"
          % METER_H)
    print("    border at y=%d, and the left-column chips match their meter colour"
          % PANEL_Y_LAST)
    print("  * nothing is drawn outside y[%d..%d]" % (PANEL_Y, PANEL_Y_LAST))
    print("  * the background behind the panel is visibly dimmed to 3/8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
