#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the labeled landscape demo set for the "arbitrary resolution" card.

Why this tool exists
--------------------
The on-chip scaler (SD/scaler_nn.v) maps each axis INDEPENDENTLY:

    tw = min(src_w * 4, 640)      th = min(src_h * 4, 480)
    off_x = (640 - tw) // 2       off_y = (480 - th) // 2

So an image is displayed WITHOUT distortion only when the source is 4:3 (both
axes then clamp proportionally onto the 4:3 panel) or when both axes are under
the 4x cap (src_w < 160 AND src_h < 120 -> uniform x4, letterboxed). Every
target below is 4:3, which keeps the demo honest: the FPGA scales, it does not
stretch.

The two photo sources are 3:2 (1536x1024), so each is CENTER-CROPPED to an exact
4:3 frame before resizing; the two 640x480 BMP sources are already 4:3 and pass
through the crop unchanged. Cropping (not squashing) is what preserves aspect.

Each output BMP is labeled with its OWN stored resolution -- that is the source
resolution the scaler receives -- so on the panel you can read e.g. "320x240"
and know the FPGA upscaled it to fill 640x480.

Outputs are 24-bit BI_RGB bottom-up BMPs, the exact container bmp_read.v parses.
Filenames carry an order prefix (1_, 2_, ...) because sync_to_sd.py sorts the
staging dir lexicographically; the prefix fixes the on-screen cycle order.
"""
import os
import sys
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_CANDIDATES = [
    r"C:\Windows\Fonts\arialbd.ttf",
    r"C:\Windows\Fonts\arial.ttf",
    r"C:\Windows\Fonts\segoeui.ttf",
]

# (order_prefix, source, target_w, target_h, friendly_name)
# All targets are 4:3 -> displayed undistorted. Ascending resolution so the
# cycle walks small -> large: x4 letterbox, x2 fill, 1:1 native, downscale.
# Four DISTINCT pictures: the earlier set reused forest and mountain twice.
# Every source is at least as large as its target, so nothing is upscaled.
# 2026-09-05: the 128x96 letterbox slot is retired. It failed to load on
# hardware for two different pictures (watermelon, apple) at two different
# directory positions while 320x240/640x480/800x600 always load, so the
# black-border path (off_x/off_y != 0, the only geometry with src_h < 120)
# is suspect in RTL. Until that is investigated, position 3 uses 480x360:
# src_h >= 120 -> off_y = 0, and 480->640 / 360->480 is a non-integer
# downscale, a path the other three targets do not exercise.
SET = [
    ("1", "3.png",
     640, 480,  "forest"),     # 640x480 -> 1:1 native
    ("2", "output_bmp/4.bmp",
     320, 240,  "rose"),       # 320x240 -> x2, fills 640x480
    ("3", "../TF卡图片/西瓜_640x480_24bit_显示正常_工程适配版.bmp",
     480, 360,  "watermelon"), # 480x360 -> non-integer downscale to 640x480
    ("4", "1.png",
     800, 600,  "mountain"),   # 800x600 -> downscaled to 640x480
]


def load_font(size):
    for path in FONT_CANDIDATES:
        if os.path.isfile(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


DST_W, DST_H, MAX_UPSCALE = 640, 480, 4


def panel_sim(src):
    """Software twin of scaler_nn.v: what the 640x480 panel will actually show.

    Index mapping is copied from the RTL (sx = dx * src_w / tw, integer divide),
    NOT from PIL's resampler, whose rounding differs by a pixel here and there.
    The point of this image is to be trustworthy next to the hardware, so it has
    to make the same choices the hardware makes: per-axis nearest neighbour, 4x
    cap, black borders centered by (DST - t) >> 1.
    """
    src_w, src_h = src.size
    tw = min(src_w * MAX_UPSCALE, DST_W)
    th = min(src_h * MAX_UPSCALE, DST_H)
    off_x = (DST_W - tw) // 2
    off_y = (DST_H - th) // 2

    px = src.convert("RGB").load()
    out = Image.new("RGB", (DST_W, DST_H), (0, 0, 0))
    op = out.load()
    for dy in range(th):
        sy = dy * src_h // th
        for dx in range(tw):
            op[off_x + dx, off_y + dy] = px[dx * src_w // tw, sy]
    return out, (tw, th, off_x, off_y)


def render_preview(items, out_png):
    """One row per image: left the labeled source as stored, right the panel."""
    row_h = DST_H + 26
    left_w = DST_W - 10
    gap = 20
    canvas = Image.new("RGB", (left_w + gap + DST_W + 20, row_h * len(items) + 30),
                       (32, 32, 36))
    d = ImageDraw.Draw(canvas)
    small = load_font(13)
    d.text((10, 6), "left: BMP as stored on the card   right: simulated 640x480 "
                    "panel output (scaler_nn.v geometry)", font=small,
           fill=(210, 210, 210))

    for i, (path, w, h) in enumerate(items):
        y0 = 30 + i * row_h
        src = Image.open(path).convert("RGB")
        box = Image.new("RGB", (left_w, DST_H), (16, 16, 18))
        box.paste(src, ((left_w - w) // 2, (DST_H - h) // 2))
        canvas.paste(box, (10, y0))

        panel, geo = panel_sim(src)
        tw, th, off_x, off_y = geo
        canvas.paste(panel, (10 + left_w + gap, y0))

        d.rectangle([10, y0, 10 + left_w, y0 + DST_H], outline=(90, 90, 96))
        d.rectangle([10 + left_w + gap, y0, 10 + left_w + gap + DST_W, y0 + DST_H],
                    outline=(90, 90, 96))
        d.text((12, y0 + DST_H + 5),
               "%s   stored %dx%d   ->  box %dx%d at off (%d,%d)"
               % (os.path.basename(path), w, h, tw, th, off_x, off_y),
               font=small, fill=(190, 190, 196))

    canvas.save(out_png)
    return canvas.size


def center_crop_4_3(img):
    """Crop the largest centered 4:3 frame from img (no resample, no distortion)."""
    w, h = img.size
    unit = min(w // 4, h // 3)
    cw, ch = unit * 4, unit * 3
    left = (w - cw) // 2
    top = (h - ch) // 2
    return img.crop((left, top, left + cw, top + ch))


def draw_label(img, w, h):
    """Overlay 'WxH' top-right on a small semi-transparent banner."""
    base = img.convert("RGBA")
    overlay = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)

    text = "%dx%d" % (w, h)
    font_size = max(9, min(16, h // 20))
    font = load_font(font_size)
    stroke = max(1, font_size // 12)

    l, t, r, b = d.textbbox((0, 0), text, font=font, stroke_width=stroke)
    tw, th = r - l, b - t
    pad_x = max(3, font_size // 3)
    pad_y = max(2, font_size // 5)
    margin = max(2, min(w, h) // 60)

    bw = tw + 2 * pad_x
    bh = th + 2 * pad_y
    x1 = w - margin
    x0 = x1 - bw
    y0 = margin
    y1 = y0 + bh

    d.rectangle([x0, y0, x1, y1], fill=(0, 0, 0, 150))
    d.text((x0 + pad_x - l, y0 + pad_y - t), text, font=font,
           fill=(255, 255, 255, 255), stroke_width=stroke,
           stroke_fill=(0, 0, 0, 255))
    return Image.alpha_composite(base, overlay).convert("RGB")


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    print("source photos: %s" % ", ".join(sorted({s[1] for s in SET})))
    print("target dir   : %s\n" % outdir)
    made = []
    for prefix, src, tw, th, name in SET:
        sp = os.path.join(HERE, src)
        if not os.path.isfile(sp):
            print("!! missing source %s" % sp)
            return 2
        img = Image.open(sp).convert("RGB")
        cropped = center_crop_4_3(img)
        resized = cropped.resize((tw, th), Image.LANCZOS)
        labeled = draw_label(resized, tw, th)
        out = os.path.join(outdir, "%s_%s_%dx%d.bmp" % (prefix, name, tw, th))
        labeled.save(out, "BMP")
        sz = os.path.getsize(out)
        made.append((out, tw, th, sz))
        print("  %dx%d  <- %s (%s)  ->  %s  [%d bytes]"
              % (tw, th, src, name, os.path.basename(out), sz))

    prev = os.path.join(HERE, "demo_labeled_preview.png")
    cw, ch = render_preview([(m[0], m[1], m[2]) for m in made], prev)
    print("\nwrote %d BMPs + preview %s (%dx%d)"
          % (len(made), os.path.basename(prev), cw, ch))

    wrote = {m[0] for m in made}
    stale = sorted(f for f in os.listdir(outdir)
                   if f.lower().endswith(".bmp")
                   and os.path.join(outdir, f) not in wrote)
    if stale:
        print("\n!! %d stale BMP(s) still in %s:" % (len(stale), outdir))
        for f in stale:
            print("     %s" % f)
        print("   sync_to_sd.py copies the first 4 files by name, so a leftover")
        print("   from an older set silently displaces one of these. Delete them.")
        return 3

    repo_root = os.path.dirname(os.path.dirname(HERE))
    print("\nnext: sync_to_sd.py F: -s %s -n 4 --wav <backup of MUSIC.WAV>"
          % os.path.relpath(outdir, repo_root).replace("\\", "/"))
    print("then: check_sd_card.py F: / sim_dir_scan.py F: / check_wav_on_card.py F:")
    return 0


if __name__ == "__main__":
    default_out = os.path.join(HERE, "demo_labeled_stage")
    dest = sys.argv[1] if len(sys.argv) > 1 else default_out
    sys.exit(main(dest))
