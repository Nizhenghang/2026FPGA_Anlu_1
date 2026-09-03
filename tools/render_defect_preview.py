#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Render the TF card's BMPs as thumbnails, plus a prediction of what the panel
showed BEFORE the scaler_nn fill watchdog fix.

Why render a prediction at all
------------------------------
tools/check_sd_card.py proves the card is clean, so the corruption has to come
from the RTL, and the diagnosis is that scaler_nn's S_FILL_W watchdog fired on
the gap between two CMD17 single block reads. That diagnosis makes a specific,
checkable visual claim:

  * the row's line buffer is only filled up to column `dx` before the bail-out
    declares the row complete, and
  * everything past it is drained from BSRAM nobody wrote, whose value is
    arbitrary but FIXED PER ADDRESS, and the drain address is a function of the
    column alone (lb_raddr = dx_out - off_x).

So the predicted picture is: real image on the left, and on the right a field
of vertical stripes that is IDENTICAL on every row. Not noise, not blocks --
stripes that repeat exactly down the screen. That is what distinguishes this
failure from every other candidate, and it is what the photograph has to be
compared against.

The first bad column of row 0 is predicted exactly: the first sector holds 512
bytes, 54 of which are the BMP header, so (512 - 54) // 3 = 152 pixels arrive
before the reader goes silent waiting for sector 1. 152 / 640 = 23.8% of the
width.

The garbage values themselves cannot be predicted -- they are whatever the
uninitialised BSRAM powers up to on this particular chip -- so they are drawn
as pseudo-random saturated colours. Only the STRUCTURE is a prediction.

Usage
-----
    python tools/render_defect_preview.py F: tools/card_preview
"""

import os
import struct
import sys

from PIL import Image

SECTOR = 512


def read_bmp(path):
    """Decode a 24 bit bottom-up BMP into a top-down RGB PIL image.

    Bottom-up is what bmp_read.v requires (height_ok_r rejects a negative
    biHeight), and frame_fifo_write applies WRITE_V_FLIP to cancel it, so the
    panel shows the file flipped relative to this raw decode. Flip here so the
    thumbnail matches the panel.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    off, = struct.unpack_from("<I", data, 10)
    w, h = struct.unpack_from("<ii", data, 18)
    bpp, = struct.unpack_from("<H", data, 28)
    if bpp != 24 or h <= 0:
        raise ValueError("%s is not a 24 bit bottom-up BMP" % path)
    stride = (w * 3 + 3) & ~3
    rows = []
    for y in range(h):
        base = off + y * stride
        row = bytearray(data[base:base + w * 3])
        # BMP stores BGR
        row[0::3], row[2::3] = row[2::3], row[0::3]
        rows.append(bytes(row))
    rows.reverse()                       # bottom-up -> top-down
    return Image.frombytes("RGB", (w, h), b"".join(rows))


def garbage_column(x):
    """Stand-in for the power-up content of an uninitialised line_buf entry.

    Only the property that matters is modelled: the value depends on the
    address, i.e. on the column, and NOT on the row.
    """
    v = (x * 2654435761) ^ 0xC0FFEE
    r = (v >> 16) & 0xFF
    g = (v >> 8) & 0xFF
    b = v & 0xFF
    # Push towards saturated colours: uninitialised BSRAM on a real device
    # reads back as vivid garbage, and a dull grey field would be hard to tell
    # apart from a dim image in a phone photograph.
    mx, mn = max(r, g, b), min(r, g, b)
    if mx == 0:
        return (255, 0, 255)
    scale = 255.0 / mx
    return (int(r * scale), int(g * scale * 0.35), int(b * scale * 0.6))


def render_defect(img, first_bad, jag=None):
    """Left of first_bad: the real image. Right: one fixed colour per column,
    the same on every row, which is the signature of draining a line buffer
    nobody finished writing."""
    w, h = img.size
    out = img.copy()
    px = out.load()
    stripe = [garbage_column(x) for x in range(w)]
    for y in range(h):
        cut = first_bad if jag is None else jag[y % len(jag)]
        for x in range(cut, w):
            px[x, y] = stripe[x]
    return out


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    root = argv[1].rstrip("\\")
    if len(root) == 2 and root[1] == ":":
        root += "\\"
    outdir = argv[2]
    os.makedirs(outdir, exist_ok=True)

    bmps = sorted(e for e in os.listdir(root) if e.lower().endswith(".bmp"))
    if not bmps:
        print("no BMP files in %s" % root)
        return 2

    thumb_w = 320
    print("rendering %d file(s) from %s into %s" % (len(bmps), root, outdir))

    for name in bmps:
        img = read_bmp(os.path.join(root, name))
        w, h = img.size
        stem = os.path.splitext(name)[0]

        th = img.copy()
        th.thumbnail((thumb_w, thumb_w), Image.LANCZOS)
        p_thumb = os.path.join(outdir, "%s_thumb.png" % stem)
        th.save(p_thumb)
        print("  %-14s %4dx%-4d -> %s" % (name, w, h, os.path.basename(p_thumb)))

        # 1:1 geometry: tw = w, th = h, so off_x = off_y = 0 for 640x480 and
        # the whole row is active. The predicted first bad column of row 0 is
        # set by the first sector alone.
        first_bad = (SECTOR - 54) // 3
        if first_bad >= w:
            print("      (first sector already covers the whole row, "
                  "no defect preview for this width)")
            continue
        bad = render_defect(img, first_bad)
        bad.thumbnail((thumb_w, thumb_w), Image.LANCZOS)
        p_bad = os.path.join(outdir, "%s_predamage.png" % stem)
        bad.save(p_bad)
        print("      predicted damage: column %d..%d of %d (%.1f%% clean on the "
              "left) -> %s" % (first_bad, w - 1, w,
                               100.0 * first_bad / w, os.path.basename(p_bad)))

    print("\nCompare the *_thumb.png files against the panel photograph to find "
          "which image was on screen, then compare that image's "
          "*_predamage.png against the photographed corruption. The prediction "
          "is about STRUCTURE -- real picture on the left, vertical stripes on "
          "the right that are identical on every row -- not about the exact "
          "colours, which are whatever this chip's BSRAM powers up to.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
