#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the 24x24 dot-matrix glyphs for the scrolling marquee banner.

Emits `src/user_source/hdl_source/marquee_font.vh`, which `marquee_overlay.v`
pulls in with a bare `include inside the module body. A naked `function` cannot
be compiled standalone, so the .vh is deliberately NOT registered in the .al.

Also writes previews into tools/preview/ so the glyphs can be eyeballed before
burning a build:
  marquee_font_sheet.png   every cell at 4x, with the 32 px pitch grid marked
  marquee_band_preview_*.png  the real 640x32 band over demo pictures

check_marquee_transcription.py re-imports build_font() and compares it against
the .vh on disk bit for bit, so this module is the golden reference.
"""

from __future__ import annotations

import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Geometry. Mirrored as localparams in marquee_overlay.v and cross-checked by
# tools/check_marquee_transcription.py. Change them here first.
# ---------------------------------------------------------------------------
SLOGAN = "FPGA创新设计竞赛国赛十周年"

CELL = 24          # glyph box, rows and columns
PITCH = 32         # advance per cell; must stay a power of two so that cell and
                   # col are bit slices of u rather than a division
GUTTER = (PITCH - CELL) // 2   # 4 px of letter spacing each side
N_CELLS = len(SLOGAN)          # 15

H_ACTIVE = 640
V_ACTIVE = 480
TEXT_W = N_CELLS * PITCH       # 480
TRAVEL = H_ACTIVE + TEXT_W     # 1120: fully off right -> fully off left
POS_BITS = 11                  # width of marq_pos / of x_pos + marq_pos

BAND_H = 32
BAND_Y = (V_ACTIVE - BAND_H) // 2   # 224, so the band centre is y = 240
BAND_TEXT_Y = BAND_Y + 4            # 228: 1 px edge line + 3 px padding

FONT_PATH = "C:/Windows/Fonts/simhei.ttf"
FONT_SIZE = 24      # Largest size whose natural ink bbox (24x24 on 新/竞) still
                    # fits CELL without a LANCZOS downsample. Bumping it re-enables
                    # resampling, which is what makes dense Chinese strokes muddy;
                    # main() prints [RESAMPLED] per glyph if that ever happens.
THRESHOLD = 110                # greyscale cut, 0..255

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VH_PATH = os.path.join(REPO, "src", "user_source", "hdl_source", "marquee_font.vh")
PREVIEW_DIR = os.path.join(REPO, "tools", "preview")

TEXT_RGB = (0xFF, 0xE8, 0x78)   # same warm yellow as osd_overlay's text_pixel
EDGE_RGB = (0x60, 0xD8, 0xFF)   # same cyan as osd_overlay's panel_border
DIM_SHIFT = 2                   # band background keeps 1/4 of the picture. A 1/2
                                # dim was tried first and, over a bright busy
                                # background, the band dissolved into the texture;
                                # see tools/preview/marquee_band_preview_*.png.


# ---------------------------------------------------------------------------
# Glyph rendering
# ---------------------------------------------------------------------------
def render_glyph(ch, font_path=FONT_PATH, font_size=FONT_SIZE, cell=CELL,
                 threshold=THRESHOLD, center_bias=0, max_ink=None):
    """Return (rows, info): rows is cell x cell of 0/1, ordered left to right.

    The glyph is drawn large, cropped to its SOLID ink extent, and only then
    centred in the cell. Cropping on the greyscale bbox instead would be wrong:
    thresholding eats the faint anti-aliased columns asymmetrically, so the ink
    that actually survives would land up to a pixel off centre.

    max_ink defaults to `cell`; setting it below the natural ink size forces a
    LANCZOS downsample, which `info` reports because resampled strokes are the
    usual cause of muddy Chinese glyphs.
    """
    if max_ink is None:
        max_ink = cell

    font = ImageFont.truetype(font_path, font_size)
    pad = 4 * font_size
    big = Image.new("L", (pad, pad), 0)
    ImageDraw.Draw(big).text((pad // 4, pad // 4), ch, font=font, fill=255)

    cut = lambda img: img.point(lambda p: 255 if p > threshold else 0)  # noqa: E731

    solid = cut(big).getbbox()
    if solid is None:
        return ([[0] * cell for _ in range(cell)],
                {"ink": (0, 0), "resampled": False, "clipped": False})

    crop = big.crop(solid)
    resampled = False
    if crop.width > max_ink or crop.height > max_ink:
        scale = min(max_ink / crop.width, max_ink / crop.height)
        crop = crop.resize((max(1, round(crop.width * scale)),
                            max(1, round(crop.height * scale))), Image.LANCZOS)
        resampled = True

    # Re-crop after thresholding: the resize can leave a fringe of sub-threshold
    # pixels, and the centring below must use the extent that really gets drawn.
    final = cut(crop)
    fb = final.getbbox()
    if fb is not None:
        final = final.crop(fb)

    out = Image.new("L", (cell, cell), 0)
    off_x = (cell - final.width) // 2 + center_bias
    off_y = (cell - final.height) // 2 + center_bias
    out.paste(final, (off_x, off_y))

    # PIL silently truncates whatever falls outside the destination, and a
    # truncated glyph still measures as "inside the cell", so record the intent.
    clipped = (off_x < 0 or off_y < 0
               or off_x + final.width > cell or off_y + final.height > cell)

    rows = [[1 if out.getpixel((x, y)) else 0 for x in range(cell)] for y in range(cell)]
    return rows, {"ink": (final.width, final.height), "resampled": resampled,
                  "clipped": clipped}


def build_glyphs(**kwargs):
    """Render the whole slogan. Returns a list of (rows, info) pairs."""
    return [render_glyph(ch, **kwargs) for ch in SLOGAN]


def build_font(**kwargs):
    """Render every cell of the slogan. Returns N_CELLS row-lists."""
    return [rows for rows, _ in build_glyphs(**kwargs)]


def row_bits(row):
    """Pack one glyph row: bit CELL-1 is the leftmost column."""
    value = 0
    for x, on in enumerate(row):
        if on:
            value |= 1 << (CELL - 1 - x)
    return value


def table_from_cells(cells):
    """{cell: {row: bits}} -- the canonical form both emitters compare in."""
    return {i: {y: row_bits(r) for y, r in enumerate(rows)} for i, rows in enumerate(cells)}


# ---------------------------------------------------------------------------
# Self-checks
# ---------------------------------------------------------------------------
def check_font(cells, infos=None):
    """Return a list of human-readable failures. Empty list means pass."""
    fails = []
    centre = (CELL - 1) / 2.0
    first_at = {}

    for idx, rows in enumerate(cells):
        ch = SLOGAN[idx]
        if infos is not None and infos[idx]["clipped"]:
            fails.append("cell %d %r: solid ink was clipped by the cell edge" % (idx, ch))
        xs = [x for y in range(CELL) for x in range(CELL) if rows[y][x]]
        ys = [y for y in range(CELL) for x in range(CELL) if rows[y][x]]
        if not xs:
            fails.append("cell %d %r: no ink at all" % (idx, ch))
            continue

        # Nothing may touch or cross the cell border: the band only reserves
        # CELL rows, so overflow would bleed into the padding.
        if min(xs) < 0 or max(xs) > CELL - 1 or min(ys) < 0 or max(ys) > CELL - 1:
            fails.append("cell %d %r: ink escapes the %dx%d cell" % (idx, ch, CELL, CELL))

        # Centring. Tolerance is half a pixel so a 1 px bias is caught; odd ink
        # widths legitimately land half a pixel off centre.
        for axis, lo, hi in (("x", min(xs), max(xs)), ("y", min(ys), max(ys))):
            got = (lo + hi) / 2.0
            if abs(got - centre) > 0.5:
                fails.append("cell %d %r: ink centre %s = %.1f, expected %.1f"
                             % (idx, ch, axis, got, centre))

        # Identical characters must render identically, otherwise the table is
        # not reproducible and the RTL cannot be regenerated.
        packed = tuple(row_bits(r) for r in rows)
        if ch in first_at:
            if first_at[ch][1] != packed:
                fails.append("char %r at cell %d differs from cell %d"
                             % (ch, idx, first_at[ch][0]))
        else:
            first_at[ch] = (idx, packed)

    # Distinct characters must not collide: a threshold that is too aggressive
    # collapses everything into a blob and this is what notices.
    by_bits = {}
    for idx, rows in enumerate(cells):
        key = tuple(row_bits(r) for r in rows)
        if key in by_bits and SLOGAN[by_bits[key]] != SLOGAN[idx]:
            fails.append("cells %d (%r) and %d (%r) rendered identically"
                         % (by_bits[key], SLOGAN[by_bits[key]], idx, SLOGAN[idx]))
        by_bits.setdefault(key, idx)

    return fails


def check_geometry():
    """The geometry must stay consistent with the bit-slice addressing."""
    fails = []
    if PITCH & (PITCH - 1):
        fails.append("PITCH %d is not a power of two; cell/col would need a divider" % PITCH)
    if H_ACTIVE % PITCH:
        fails.append("H_ACTIVE %d is not a multiple of PITCH %d; u would be misaligned"
                     % (H_ACTIVE, PITCH))
    if CELL > PITCH:
        fails.append("CELL %d exceeds PITCH %d; glyphs would overlap" % (CELL, PITCH))
    if BAND_Y + BAND_H // 2 != V_ACTIVE // 2:
        fails.append("band centre y=%d is not the frame middle y=%d"
                     % (BAND_Y + BAND_H // 2, V_ACTIVE // 2))
    if BAND_TEXT_Y + CELL > BAND_Y + BAND_H - 1:
        fails.append("glyph rows y%d..%d overflow the band y%d..%d"
                     % (BAND_TEXT_Y, BAND_TEXT_Y + CELL - 1, BAND_Y, BAND_Y + BAND_H - 1))
    if TRAVEL > (1 << POS_BITS) - 1:
        fails.append("TRAVEL %d overflows the %d bit marq_pos" % (TRAVEL, POS_BITS))
    if H_ACTIVE - 1 + TRAVEL - 1 > (1 << POS_BITS) - 1:
        fails.append("x_pos + marq_pos peaks at %d, overflows %d bits"
                     % (H_ACTIVE - 1 + TRAVEL - 1, POS_BITS))
    return fails


def check_negative_controls():
    """The checks above must have teeth. Returns a list of failures."""
    fails = []
    good, good_info = zip(*build_glyphs())
    good, good_info = list(good), list(good_info)

    baseline = check_font(good, good_info)
    if baseline:
        fails.append("baseline font fails its own checks: %s" % baseline)

    # A 1 px bias trips the half-pixel centring tolerance on even-width glyphs
    # and the clip check on full-width ones (新 / 竞), so it cannot slip through.
    biased, biased_info = zip(*build_glyphs(center_bias=1))
    if not check_font(list(biased), list(biased_info)):
        fails.append("negative control: center_bias=1 was NOT caught")

    blanked = [list(r) for r in good]
    blanked[3] = [[0] * CELL for _ in range(CELL)]
    if not check_font(blanked, good_info):
        fails.append("negative control: blanked cell was NOT caught")

    collided = [list(r) for r in good]
    collided[5] = [list(r) for r in good[6]]
    if not check_font(collided, good_info):
        fails.append("negative control: char collision was NOT caught")

    # check_geometry reads PITCH out of the module globals at call time, so
    # perturbing it here proves the power-of-two guard actually fires.
    good_pitch = globals()["PITCH"]
    if check_geometry():
        fails.append("baseline geometry fails its own checks")
    globals()["PITCH"] = 30
    if not check_geometry():
        fails.append("negative control: PITCH=30 was NOT caught")
    globals()["PITCH"] = good_pitch

    return fails


# ---------------------------------------------------------------------------
# Verilog emission
# ---------------------------------------------------------------------------
def emit_vh(cells):
    out = [
        "// Generated by tools/gen_marquee_font.py -- DO NOT EDIT BY HAND.",
        "// Re-run the generator instead; tools/check_marquee_transcription.py",
        "// compares this file against a fresh render, bit for bit.",
        "//",
        "// Included from inside marquee_overlay.v's module body, so this file holds",
        "// a bare function and must NOT be added to the .al file list.",
        "//",
        "// Slogan    : %s" % SLOGAN,
        "// Font      : %s @ %d px, threshold %d" % (FONT_PATH, FONT_SIZE, THRESHOLD),
        "// Cells     : %d x %d px, pitch %d px (%d px gutter each side)"
        % (N_CELLS, CELL, PITCH, GUTTER),
        "// Bit order : bit %d is the leftmost column of the row." % (CELL - 1),
        "// Blank rows are folded into the default arm to keep the file short.",
        "// The argument is cell_idx, not cell: `cell` is a Verilog-2001 reserved",
        "// word and TD rejects it with HDL-8007.",
        "",
        "function [23:0] marquee_glyph;",
        "    input [3:0] cell_idx;",
        "    input [4:0] row;",
        "    begin",
        "        case (cell_idx)",
    ]
    for idx, rows in enumerate(cells):
        out.append("            4'd%d: case (row)   // %s" % (idx, SLOGAN[idx]))
        for y in range(CELL):
            bits = row_bits(rows[y])
            if bits == 0:
                continue
            out.append("                5'd%d: marquee_glyph = 24'h%06X;" % (y, bits))
        out.append("                default: marquee_glyph = 24'h000000;")
        out.append("            endcase")
    out.append("            default: marquee_glyph = 24'h000000;")
    out.append("        endcase")
    out.append("    end")
    out.append("endfunction")
    out.append("")
    return "\n".join(out)


_ROW_RE = re.compile(r"5'd(\d+):\s*marquee_glyph\s*=\s*24'h([0-9A-Fa-f]{1,6});")
_CELL_RE = re.compile(r"4'd(\d+):\s*case \(row\)")


def parse_vh(text):
    """Read a .vh back into {cell: {row: bits}}. Used by the transcription check."""
    table = {}
    cell = None
    for line in text.splitlines():
        line = line.strip()
        m = _CELL_RE.match(line)
        if m:
            cell = int(m.group(1))
            table[cell] = {}
            continue
        m = _ROW_RE.match(line)
        if m and cell is not None:
            table[cell][int(m.group(1))] = int(m.group(2), 16)
    return table


def tables_equal(parsed, expected):
    """Compare, treating rows the emitter folded into `default` as zero."""
    diffs = []
    if set(parsed) != set(expected):
        return ["cell set differs: parsed %s vs expected %s"
                % (sorted(parsed), sorted(expected))]
    for cell in sorted(expected):
        for row in range(CELL):
            got = parsed[cell].get(row, 0)
            want = expected[cell].get(row, 0)
            if got != want:
                diffs.append("cell %d row %d: .vh has 24'h%06X, generator has 24'h%06X"
                             % (cell, row, got, want))
    return diffs


# ---------------------------------------------------------------------------
# Preview: a pixel-exact model of marquee_overlay.v's output mux
# ---------------------------------------------------------------------------
def band_pixel(x, y, marq_pos, bg_rgb, packed):
    if not (BAND_Y <= y < BAND_Y + BAND_H):
        return bg_rgb
    if y == BAND_Y or y == BAND_Y + BAND_H - 1:
        return EDGE_RGB

    dim = tuple(c >> DIM_SHIFT for c in bg_rgb)

    if not (BAND_TEXT_Y <= y < BAND_TEXT_Y + CELL):
        return dim

    # u = x_pos + marq_pos - H_ACTIVE, taken modulo 2**POS_BITS exactly as the
    # 11 bit wire wraps; in_region is the single unsigned compare u < TEXT_W.
    u = (x + marq_pos - H_ACTIVE) & ((1 << POS_BITS) - 1)
    if u >= TEXT_W:
        return dim

    cell = (u >> 5) & 0x1F
    col = u & 0x1F
    if cell >= N_CELLS or not (GUTTER <= col < GUTTER + CELL):
        return dim

    gcol = col - GUTTER
    row = y - BAND_TEXT_Y
    if (packed[cell][row] >> (CELL - 1 - gcol)) & 1:
        return TEXT_RGB
    return dim


def render_strip(background, marq_pos, packed, pad=12):
    """Render only the rows around the band -- the rest is untouched background."""
    y0, y1 = BAND_Y - pad, BAND_Y + BAND_H + pad
    img = Image.new("RGB", (H_ACTIVE, y1 - y0))
    px = img.load()
    src = background.load()
    for y in range(y0, y1):
        for x in range(H_ACTIVE):
            px[x, y - y0] = band_pixel(x, y, marq_pos, src[x, y], packed)
    return img


def write_previews(packed):
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    paths = []

    # --- contact sheet -----------------------------------------------------
    zoom = 4
    grid_w = N_CELLS * PITCH * zoom + 40
    sheet = Image.new("RGB", (grid_w, CELL * zoom + 96), (16, 16, 24))
    d = ImageDraw.Draw(sheet)
    top = 44
    for idx, rows in enumerate(_rows_of(packed)):
        cx = 20 + idx * PITCH * zoom
        d.rectangle([cx, top - 6, cx, top + CELL * zoom], outline=None, fill=(90, 130, 170))
        d.rectangle([cx + GUTTER * zoom, top, cx + (GUTTER + CELL) * zoom - 1,
                     top + CELL * zoom - 1], outline=(50, 70, 95))
        for y in range(CELL):
            for x in range(CELL):
                if rows[y][x]:
                    ox = cx + (GUTTER + x) * zoom
                    d.rectangle([ox, top + y * zoom, ox + zoom - 1, top + y * zoom + zoom - 1],
                                fill=TEXT_RGB)
        d.text((cx + 3, 8), "%d %s" % (idx, SLOGAN[idx]), fill=(200, 220, 255))
    d.text((20, top + CELL * zoom + 12), SLOGAN, fill=(255, 232, 120))
    d.text((20, top + CELL * zoom + 34),
           "%d cells, glyph %dx%d, pitch %d (gutter %d), %s @ %d px, threshold %d"
           % (N_CELLS, CELL, CELL, PITCH, GUTTER,
              os.path.basename(FONT_PATH), FONT_SIZE, THRESHOLD),
           fill=(190, 190, 190))
    p = os.path.join(PREVIEW_DIR, "marquee_font_sheet.png")
    sheet.save(p)
    paths.append(p)

    # --- band over real demo pictures --------------------------------------
    backgrounds = [
        os.path.join(REPO, "doc", "TF卡图片", "西瓜_640x480_24bit_显示正常_工程适配版.bmp"),
        os.path.join(REPO, "doc", "convert", "3.png"),
    ]
    offsets = [700, 820, 940]
    strip_h = BAND_H + 24
    for bpath in backgrounds:
        if not os.path.exists(bpath):
            print("  (skip preview background, missing: %s)" % bpath)
            continue
        bg = Image.open(bpath).convert("RGB")
        if bg.size != (H_ACTIVE, V_ACTIVE):
            bg = bg.resize((H_ACTIVE, V_ACTIVE))
        canvas = Image.new("RGB", (H_ACTIVE, strip_h * len(offsets) + 8), (0, 0, 0))
        cd = ImageDraw.Draw(canvas)
        for i, off in enumerate(offsets):
            canvas.paste(render_strip(bg, off, packed), (0, i * strip_h))
            cd.text((4, i * strip_h + 1), "marq_pos=%d" % off, fill=(255, 255, 255))
        stem = os.path.splitext(os.path.basename(bpath))[0]
        p = os.path.join(PREVIEW_DIR, "marquee_band_preview_%s.png" % stem)
        canvas.save(p)
        paths.append(p)
    return paths


def _rows_of(packed):
    """Unpack {cell:{row:bits}} back to 0/1 rows, for drawing only."""
    out = []
    for cell in range(N_CELLS):
        rows = []
        for y in range(CELL):
            bits = packed[cell].get(y, 0)
            rows.append([(bits >> (CELL - 1 - x)) & 1 for x in range(CELL)])
        out.append(rows)
    return out


# ---------------------------------------------------------------------------
def main():
    failures = 0

    print("=== rendering %d glyphs: %s" % (N_CELLS, SLOGAN))
    rendered = build_glyphs()
    cells = [rows for rows, _ in rendered]
    infos = [info for _, info in rendered]
    for idx, (ch, info) in enumerate(zip(SLOGAN, infos)):
        flags = ""
        if info["resampled"]:
            flags += "   [RESAMPLED]"
        if info["clipped"]:
            flags += "   [CLIPPED]"
        print("  cell %2d %s  ink %2dx%-2d  pixels %4d%s"
              % (idx, ch, info["ink"][0], info["ink"][1],
                 sum(sum(r) for r in cells[idx]), flags))

    if any(i["resampled"] for i in infos):
        msg = ("at least one glyph was downsampled to fit the %d px cell; "
               "resampled strokes are what makes dense Chinese glyphs muddy" % CELL)
        if "--allow-resample" in sys.argv:
            print("  WARN: " + msg)
        else:
            print("  FAIL: " + msg)
            print("        lower FONT_SIZE, or re-run with --allow-resample to override")
            failures += 1
    if any(i["clipped"] for i in infos):
        print("  FAIL: at least one glyph's solid ink was clipped by the cell edge")
        failures += 1

    packed = table_from_cells(cells)

    print("=== geometry")
    print("  TEXT_W = %d * %d = %d px,  TRAVEL = %d + %d = %d px"
          % (N_CELLS, PITCH, TEXT_W, H_ACTIVE, TEXT_W, TRAVEL))
    print("  band   = y %d..%d (centre %d),  glyph rows y %d..%d"
          % (BAND_Y, BAND_Y + BAND_H - 1, BAND_Y + BAND_H // 2,
             BAND_TEXT_Y, BAND_TEXT_Y + CELL - 1))
    for f in check_geometry():
        print("  FAIL: " + f)
        failures += 1

    print("=== glyph self-checks")
    glyph_fails = check_font(cells, infos)
    for f in glyph_fails:
        print("  FAIL: " + f)
    failures += len(glyph_fails)
    if not glyph_fails:
        print("  pass: ink non-empty, inside the cell, centred, duplicates identical, no collisions")

    print("=== negative controls")
    nc_fails = check_negative_controls()
    for f in nc_fails:
        print("  FAIL: " + f)
    failures += len(nc_fails)
    if not nc_fails:
        print("  pass: center_bias / blank cell / char collision / PITCH=30 all caught")

    if failures:
        print("\n%d failure(s); not writing the .vh" % failures)
        return 1

    text = emit_vh(cells)
    os.makedirs(os.path.dirname(VH_PATH), exist_ok=True)
    with open(VH_PATH, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("=== wrote %s (%d bytes, %d lines)" % (VH_PATH, len(text), text.count("\n")))

    diffs = tables_equal(parse_vh(text), packed)
    if diffs:
        print("=== FAIL: round-trip through the .vh lost data")
        for d in diffs[:10]:
            print("  " + d)
        return 1
    print("=== round-trip parse of the .vh matches the render bit for bit")

    print("=== previews")
    for p in write_previews(packed):
        print("  " + p)

    print("\nOK -- inspect the previews before building.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
