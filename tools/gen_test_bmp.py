#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the BMP acceptance sets for the stage 3 nearest-neighbour scaler.

Why this script exists
  The stage 3 acceptance list in the plan is 640x480, 800x600, 1920x1080 and
  320x240. Every one of those maps to tw=640, th=480, off=(0,0) and therefore
  fills the screen exactly, so the list as written never shows a black border,
  never shows centring, and never builds up the elastic-buffer backlog that the
  border rows cause. It would have passed with the 256 entry buffer that
  silently dropped source pixels. Set B covers that gap.

  SCAN_TARGET_COUNT in sd_card_bmp.v is 4, matching the four SDRAM frame
  buffers, so only the first four BMPs found in the root directory are ever
  loaded. The two sets therefore have to go on the card one at a time.

What the pattern is designed to expose
  corner blocks   red TL / green TR / blue BL / white BR. A vertical flip swaps
                  red and blue, a horizontal flip swaps red and green, and a
                  BGR-vs-RGB mistake in the pixel unpacking shows up as blue
                  where red belongs.
  1 px black edge the outermost source pixel on all four sides. Proves no
                  off-by-one clipping in the fill or the drain.
  centred box     80% x 60% of the source. Asymmetric margins on screen mean
                  the mapping is shifted.
  label           the source resolution, so a carousel of four images can be
                  told apart on a panel with no other readout.
  plain field     a distinct mid-dark colour per resolution, so even a badly
                  corrupted frame is identifiable.

Output format, matched to what bmp_read.v accepts
  24 bpp, BI_RGB (compression 0), 40 byte BITMAPINFOHEADER, pixel data at
  offset 54, and a POSITIVE height, i.e. bottom-up row order. Positive height
  is not a stylistic choice: height_ok_r in bmp_read.v requires
  height[31:16] == 0, so a top-down BMP with a negative height is rejected
  outright and never displayed. PIL's BMP writer produces exactly this.

Card preparation, and this part matters
  ST_LOAD_DATA advances with sd_sec_read_addr + 1 and never follows the FAT
  chain, so each file has to be physically contiguous on the card. Format the
  card FAT32 first, then copy one set in a single operation, and do not add or
  delete files afterwards. A fragmented file reads as garbage no matter how
  correct the RTL is.

Usage:
    python tools/gen_test_bmp.py            # writes both sets under doc/convert
    python tools/gen_test_bmp.py --out DIR  # override the destination root
"""

import argparse
import os
import struct
import sys

from PIL import Image, ImageDraw

DST_W = 640
DST_H = 480
MAX_UPSCALE = 4
SRC_W_MIN, SRC_W_MAX = 64, 1920
SRC_H_MIN, SRC_H_MAX = 64, 1080

# 5x7 glyphs, top row first. Only the characters the resolution label needs.
FONT = {
    '0': "01110 10001 10011 10101 11001 10001 01110",
    '1': "00100 01100 00100 00100 00100 00100 01110",
    '2': "01110 10001 00001 00010 00100 01000 11111",
    '3': "11111 00010 00100 00010 00001 10001 01110",
    '4': "00010 00110 01010 10010 11111 00010 00010",
    '5': "11111 10000 11110 00001 00001 10001 01110",
    '6': "00110 01000 10000 11110 10001 10001 01110",
    '7': "11111 00001 00010 00100 01000 01000 01000",
    '8': "01110 10001 10001 01110 10001 10001 01110",
    '9': "01110 10001 10001 01111 00001 00010 01100",
    'x': "00000 10001 01010 00100 01010 10001 00000",
}

# name, width, height, background colour
SET_A = [
    ("setA_fill", [
        ("640x480",   640,  480,  (128, 128, 128)),
        ("800x600",   800,  600,  (0,   96,  96)),
        ("1920x1080", 1920, 1080, (64,  48,  112)),
        ("320x240",   320,  240,  (48,  96,  48)),
    ]),
]

SET_B = [
    ("setB_border", [
        ("64x64",    64,   64,  (112, 48, 48)),
        ("100x100",  100,  100, (96,  96, 32)),
        ("159x119",  159,  119, (96,  48, 96)),
        ("1920x64",  1920, 64,  (112, 80, 32)),
    ]),
]


def expected_geometry(src_w, src_h):
    """The scaler's own formulas, copied from scaler_nn.v."""
    tw = min(src_w * MAX_UPSCALE, DST_W)
    th = min(src_h * MAX_UPSCALE, DST_H)
    return tw, th, (DST_W - tw) >> 1, (DST_H - th) >> 1


def draw_text(img, text, cx, cy, scale, colour):
    """Centred 5x7 block text. Drawn as filled rectangles, not strokes, so it
    survives nearest-neighbour downscaling without turning to dust."""
    d = ImageDraw.Draw(img)
    advance = 6 * scale
    total = len(text) * advance - scale
    x = cx - total // 2
    y = cy - (7 * scale) // 2
    for ch in text:
        rows = FONT[ch].split()
        for ry, row in enumerate(rows):
            for rx, bit in enumerate(row):
                if bit == '1':
                    d.rectangle([x + rx * scale, y + ry * scale,
                                 x + rx * scale + scale - 1,
                                 y + ry * scale + scale - 1], fill=colour)
        x += advance


def make_image(src_w, src_h, bg):
    img = Image.new('RGB', (src_w, src_h), bg)
    d = ImageDraw.Draw(img)

    u = min(src_w, src_h)                       # the unit everything scales on
    corner = max(2, int(round(u * 0.14)))
    thick = max(1, int(round(u * 0.02)))
    scale = max(1, int(round(u / 48.0)))

    # Corner blocks: TL red, TR green, BL blue, BR white.
    d.rectangle([0, 0, corner - 1, corner - 1], fill=(255, 0, 0))
    d.rectangle([src_w - corner, 0, src_w - 1, corner - 1], fill=(0, 255, 0))
    d.rectangle([0, src_h - corner, corner - 1, src_h - 1], fill=(0, 0, 255))
    d.rectangle([src_w - corner, src_h - corner,
                 src_w - 1, src_h - 1], fill=(255, 255, 255))

    # Centred box, 80% x 60% of the source.
    bw = int(round(src_w * 0.80))
    bh = int(round(src_h * 0.60))
    x0 = (src_w - bw) // 2
    y0 = (src_h - bh) // 2
    for t in range(thick):
        d.rectangle([x0 + t, y0 + t,
                     x0 + bw - 1 - t, y0 + bh - 1 - t], outline=(255, 255, 255))

    # Label, centred inside the box.
    draw_text(img, "%dx%d" % (src_w, src_h), src_w // 2, src_h // 2,
              scale, (255, 255, 255))

    # One pixel of black on all four source edges, drawn last so nothing
    # overwrites it: this is the outermost pixel the scaler must reproduce.
    d.rectangle([0, 0, src_w - 1, 0], fill=(0, 0, 0))
    d.rectangle([0, src_h - 1, src_w - 1, src_h - 1], fill=(0, 0, 0))
    d.rectangle([0, 0, 0, src_h - 1], fill=(0, 0, 0))
    d.rectangle([src_w - 1, 0, src_w - 1, src_h - 1], fill=(0, 0, 0))

    return img


def verify_bmp(path, src_w, src_h):
    """Read the header back and assert everything bmp_read.v checks for."""
    with open(path, 'rb') as f:
        b = f.read(54)
    problems = []
    if b[0:2] != b'BM':
        problems.append("signature is %r, not b'BM'" % b[0:2])
    pix_off, hdr_size = struct.unpack_from('<II', b, 10)
    w, h = struct.unpack_from('<ii', b, 18)
    planes, bpp = struct.unpack_from('<HH', b, 26)
    comp, img_size = struct.unpack_from('<II', b, 30)
    if pix_off != 54:
        problems.append("pixel offset %d, expected 54" % pix_off)
    if hdr_size != 40:
        problems.append("header size %d, expected 40 (BITMAPINFOHEADER)"
                        % hdr_size)
    if w != src_w:
        problems.append("width %d, expected %d" % (w, src_w))
    if h != src_h:
        problems.append("height %d, expected +%d (bottom-up); a negative "
                        "height is rejected by height_ok_r" % (h, src_h))
    if bpp != 24:
        problems.append("bpp %d, expected 24" % bpp)
    if comp != 0:
        problems.append("compression %d, expected 0 (BI_RGB)" % comp)
    stride = (src_w * 3 + 3) & ~3
    expect_size = 54 + stride * src_h
    actual = os.path.getsize(path)
    if actual != expect_size:
        problems.append("file size %d, expected %d (stride %d)"
                        % (actual, expect_size, stride))
    if planes != 1:
        problems.append("planes %d, expected 1" % planes)
    return problems, stride


def render_expected_screen(src_path, src_w, src_h):
    """What the panel should show, computed with scaler_nn.v's own mapping.

    sx = ((dx - off_x) * src_w) // tw and sy = ((dy - off_y) * src_h) // th,
    black outside the active region. This is the same expression the reference
    model in tools/sim_scaler_nn.py checks every emitted pixel against, so the
    preview and the RTL agree by construction rather than by hope. It is
    deliberately NOT produced with PIL's resize, whose nearest-neighbour
    sampling does not use the same floor formula and would differ by a pixel
    here and there.
    """
    tw, th, off_x, off_y = expected_geometry(src_w, src_h)
    src = Image.open(src_path).convert('RGB')
    sp = src.load()
    out = Image.new('RGB', (DST_W, DST_H), (0, 0, 0))
    buf = out.load()
    for dy in range(off_y, off_y + th):
        sy = ((dy - off_y) * src_h) // th
        for dx in range(off_x, off_x + tw):
            buf[dx, dy] = sp[((dx - off_x) * src_w) // tw, sy]
    return out


def write_previews(preview_dir, set_dir, entries):
    """PNG previews go OUTSIDE the set directories on purpose: each set dir must
    hold exactly four BMPs, because that is all the card may contain for
    SCAN_TARGET_COUNT = 4 to pick up the intended files."""
    os.makedirs(preview_dir, exist_ok=True)
    made = 0
    for (ename, w, h, _bg) in entries:
        src = os.path.join(set_dir, ename + ".bmp")
        if not os.path.exists(src):
            print("  ! preview skipped, %s is missing" % src)
            continue
        png = os.path.join(preview_dir, ename + "_expected.png")
        render_expected_screen(src, w, h).save(png, 'PNG')
        print("  preview %-22s %s" % (ename + "_expected.png",
                                      "640x480, what the panel should show"))
        made += 1
    return made


def verify_previews(preview_dir, sets):
    """Check the previews against the sources with the mapping formula.

    The probe coordinate is derived from the mapping rather than hard-coded.
    Hard-coding it is how the first version of this check reported a false
    MISMATCH on 1920x64: the corner block is only max(2, round(64 * 0.14)) = 9
    source pixels wide and the horizontal downscale is 3x, so screen column 4
    maps to source column 12 and is already past the block. Any fixed guess
    has that problem on some geometry.
    """
    failures = 0
    for (set_name, entries) in sets:
        for (name, w, h, _bg) in entries:
            src_path = os.path.join(preview_dir, os.pardir, set_name,
                                    name + ".bmp")
            png_path = os.path.join(preview_dir, name + "_expected.png")
            if not (os.path.exists(src_path) and os.path.exists(png_path)):
                continue
            tw, th, off_x, off_y = expected_geometry(w, h)
            corner = max(2, int(round(min(w, h) * 0.14)))
            sp = Image.open(src_path).convert('RGB').load()
            pp = Image.open(png_path).convert('RGB').load()

            probe = None
            for dy in range(off_y, off_y + th):
                sy = ((dy - off_y) * h) // th
                if sy < 1 or sy >= corner - 1:
                    continue
                for dx in range(off_x, off_x + tw):
                    sx = ((dx - off_x) * w) // tw
                    if 1 <= sx < corner - 1:
                        probe = (dx, dy, sx, sy)
                        break
                if probe:
                    break

            problems = []
            if probe is None:
                problems.append("no probe pixel: the corner block is too small "
                                "to survive this downscale")
            else:
                dx, dy, sx, sy = probe
                if pp[dx, dy] != sp[sx, sy]:
                    problems.append("screen(%d,%d) is %s but source(%d,%d) is "
                                    "%s" % (dx, dy, pp[dx, dy], sx, sy,
                                             sp[sx, sy]))
                if sp[sx, sy] != (255, 0, 0):
                    problems.append("probe landed on %s, expected the red "
                                    "corner block" % (sp[sx, sy],))
            if pp[off_x, off_y] != (0, 0, 0):
                problems.append("first active pixel is %s, expected the source "
                                "1 px black edge" % (pp[off_x, off_y],))
            for x in range(0, off_x):
                if pp[x, off_y] != (0, 0, 0):
                    problems.append("left border not black at x=%d" % x)
                    break
            for x in range(off_x + tw, DST_W):
                if pp[x, off_y] != (0, 0, 0):
                    problems.append("right border not black at x=%d" % x)
                    break
            for y in range(0, off_y):
                if pp[off_x, y] != (0, 0, 0):
                    problems.append("top border not black at y=%d" % y)
                    break
            for y in range(off_y + th, DST_H):
                if pp[off_x, y] != (0, 0, 0):
                    problems.append("bottom border not black at y=%d" % y)
                    break

            if problems:
                failures += 1
            print("  %-4s %-14s tw=%3d th=%3d off=(%3d,%3d)"
                  % ("ok" if not problems else "FAIL", name + "_expected.png",
                     tw, th, off_x, off_y))
            for p in problems:
                print("        ! %s" % p)
    return failures


def verify_passthrough(set_dir, preview_dir):
    """640x480 maps 1:1, so its preview must equal its source pixel for pixel.
    Any difference means the mapping formula and the generator disagree."""
    src = os.path.join(set_dir, "640x480.bmp")
    png = os.path.join(preview_dir, "640x480_expected.png")
    if not (os.path.exists(src) and os.path.exists(png)):
        return 0
    a = Image.open(src).convert('RGB')
    b = Image.open(png).convert('RGB')
    # tobytes rather than getdata: same comparison, faster, and getdata is
    # slated for removal in Pillow 14.
    same = (a.size == b.size) and (a.tobytes() == b.tobytes())
    print("  %-4s 640x480 preview is byte-identical to its source (1:1 "
          "passthrough)" % ("ok" if same else "FAIL"))
    return 0 if same else 1


def write_set(dirname, entries):
    os.makedirs(dirname, exist_ok=True)
    lines = []
    lines.append("BMP acceptance set: %s" % os.path.basename(dirname))
    lines.append("")
    lines.append("Copy the .bmp files of THIS DIRECTORY ONLY to the root of a")
    lines.append("freshly FAT32 formatted TF card, in one operation. Do not add")
    lines.append("or delete files afterwards: ST_LOAD_DATA walks sectors with")
    lines.append("sd_sec_read_addr + 1 and never follows the FAT chain, so a")
    lines.append("fragmented file reads as garbage.")
    lines.append("")
    lines.append("bmp_read.v scans the root directory in entry order and stops")
    lines.append("at SCAN_TARGET_COUNT = 4, so exactly these four files must be")
    lines.append("the only BMPs on the card.")
    lines.append("")
    lines.append("Compare the panel against ../expected_screen/<name>_expected.png,")
    lines.append("NOT against the source BMPs. Those previews are 640x480 and are")
    lines.append("computed with scaler_nn.v's own mapping formula, so they show")
    lines.append("what the screen should actually look like after the stretch and")
    lines.append("the centring. A strong downscale is meant to look squashed.")
    lines.append("")
    hdr = ("%-14s %-7s %-9s %-13s %-11s %s"
           % ("file", "src", "stride", "expected tw/th", "expected off",
              "on screen"))
    lines.append(hdr)
    lines.append("-" * len(hdr))

    failures = 0
    for (name, w, h, bg) in entries:
        if not (SRC_W_MIN <= w <= SRC_W_MAX):
            print("  ! %s: width %d outside bmp_read's [%d, %d]"
                  % (name, w, SRC_W_MIN, SRC_W_MAX))
            failures += 1
            continue
        if not (SRC_H_MIN <= h <= SRC_H_MAX):
            print("  ! %s: height %d outside bmp_read's [%d, %d]"
                  % (name, h, SRC_H_MIN, SRC_H_MAX))
            failures += 1
            continue

        path = os.path.join(dirname, name + ".bmp")
        make_image(w, h, bg).save(path, 'BMP')
        problems, stride = verify_bmp(path, w, h)
        tw, th, off_x, off_y = expected_geometry(w, h)

        fills = (tw == DST_W and th == DST_H)
        if fills:
            screen = "fills the panel, no border"
        else:
            screen = "%dx%d image centred, black border %d px each side, %d px top/bottom" % (
                tw, th, off_x, off_y)

        status = "ok"
        if problems:
            status = "BAD"
            failures += 1
        print("  %-4s %-14s %5dx%-5d stride=%-5d tw=%3d th=%3d "
              "off=(%3d,%3d)  %d bytes"
              % (status, name + ".bmp", w, h, stride, tw, th, off_x, off_y,
                 os.path.getsize(path)))
        for p in problems:
            print("        ! %s" % p)

        pad = "  (row padded, %d of %d bytes are pixels)" % (w * 3, stride) \
            if stride != w * 3 else ""
        lines.append("%-14s %-7s %-9s %-13s %-11s %s"
                     % (name + ".bmp", "%dx%d" % (w, h),
                        "%d%s" % (stride, "*" if pad else ""),
                        "%dx%d" % (tw, th), "%d,%d" % (off_x, off_y), screen))
        if pad:
            lines.append("%-14s %s" % ("", pad.strip()))

    lines.append("")
    lines.append("(*) a padded row is what exercises the row_byte_cnt < src_w3")
    lines.append("    gate in bmp_read.v: a 24-bit row is padded up to a")
    lines.append("    multiple of four bytes and those pad bytes must not be")
    lines.append("    taken for pixels. Every multiple-of-four width skips that")
    lines.append("    path entirely, which is how the 640x480 only test set got")
    lines.append("    away without it.")
    lines.append("")
    lines.append("Pattern key, same for every file in this set:")
    lines.append("    corner blocks   top-left RED, top-right GREEN,")
    lines.append("                    bottom-left BLUE, bottom-right WHITE")
    lines.append("    1 px black edge on all four sides of the source")
    lines.append("    centred white box, 80% x 60% of the source")
    lines.append("    white label with the source resolution, centred")
    lines.append("")
    lines.append("How to read the result:")
    lines.append("    red and blue swapped      image is flipped vertically")
    lines.append("    red and green swapped     image is flipped horizontally")
    lines.append("    red corner looks blue     BGR/RGB order wrong in unpacking")
    lines.append("    box margins unequal       mapping is shifted, off-by-one")
    lines.append("    black edge missing        outermost source pixel is clipped")
    lines.append("    label unreadable but      expected on a strong downscale;")
    lines.append("      colours and box ok        nearest neighbour aliases fine")
    lines.append("                              detail away by design")

    with open(os.path.join(dirname, "manifest.txt"), 'w',
              encoding='utf-8', newline='\n') as f:
        f.write("\n".join(lines) + "\n")
    return failures


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_out = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "doc", "convert")
    ap.add_argument("--out", default=default_out,
                    help="destination root (default: doc/convert)")
    ap.add_argument("--no-preview", action="store_true",
                    help="skip rendering the expected-on-screen PNG previews")
    args = ap.parse_args()

    print("writing BMP acceptance sets under %s" % args.out)
    failures = 0
    previews = os.path.join(args.out, "expected_screen")
    for group in (SET_A + SET_B):
        (set_name, entries) = group
        print("[%s]" % set_name)
        set_dir = os.path.join(args.out, set_name)
        failures += write_set(set_dir, entries)
        if not args.no_preview:
            write_previews(previews, set_dir, entries)
    print()
    if not args.no_preview:
        print("[verify previews against sources]")
        failures += verify_previews(previews, SET_A + SET_B)
        failures += verify_passthrough(os.path.join(args.out, "setA_fill"),
                                       previews)
        print()
        print("expected-on-screen previews are in %s" % previews)
        print("compare the panel against those, not against the source BMPs:")
        print("a 1920x1080 source is meant to look squashed, because the")
        print("scaler stretches to fill 640x480 rather than letterboxing.")
        print()
    if failures:
        print("%d problem(s), the set is NOT usable" % failures)
        return 1
    print("all BMPs verified against bmp_read.v's header checks"
          + ("" if args.no_preview else ", all previews verified against the "
             "mapping formula"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
