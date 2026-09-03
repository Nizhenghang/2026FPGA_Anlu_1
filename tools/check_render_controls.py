"""Perturbation harness for render_spectrum_preview.py.

Every geometry / pixel assertion is only worth as much as the number of ways it
can fail. This rewrites the preview source 15 ways, each time breaking one
specific thing, and asserts the checker reports a failure. A perturbation that
passes means the corresponding assertion is dead weight. An anchor string that
no longer matches the source is reported as a failure too, not skipped -- two of
these anchors were wrong on first run and silently tested nothing.

Run from anywhere:  python tools/check_render_controls.py
"""
import contextlib
import importlib.util
import io
import os
import shutil
import sys
import tempfile

sys.dont_write_bytecode = True

TOOLS = os.path.dirname(os.path.abspath(__file__))
# the perturbed copies are written outside tools/, so they need it on the path to
# import their sibling sim_biquad_bank
sys.path.insert(0, TOOLS)

TMP = tempfile.mkdtemp(prefix="spectrum_controls_")

SRC = os.path.join(TOOLS, "render_spectrum_preview.py")
with open(SRC, "r", encoding="utf-8") as fh:
    ORIG = fh.read()

# (label, old_text, new_text, why it must fail)
PERTURB = [
    ("panel overflows the bottom of the screen",
     "PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 28, 352, 584, 104",
     "PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 28, 352, 584, 130",
     "PANEL_Y_LAST becomes 481 > 479"),
    ("panel overflows the right of the screen",
     "PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 28, 352, 584, 104",
     "PANEL_X, PANEL_Y, PANEL_W, PANEL_H = 28, 352, 620, 104",
     "PANEL_X_LAST becomes 647 > 639"),
    ("bar area starts above the panel interior",
     "BAR_X, BAR_Y, BAR_W, BAR_H = 44, 362, 512, 64",
     "BAR_X, BAR_Y, BAR_W, BAR_H = 44, 350, 512, 64",
     "BAR_Y 350 is above PANEL_Y + 1 = 353"),
    ("bar area overruns the panel bottom",
     "BAR_X, BAR_Y, BAR_W, BAR_H = 44, 362, 512, 64",
     "BAR_X, BAR_Y, BAR_W, BAR_H = 44, 362, 512, 100",
     "BAR_Y_LAST becomes 461, past PANEL_Y_LAST and into the meters"),
    ("bar widened until it touches its neighbours",
     "BAR_PX = 24", "BAR_PX = 32",
     "a full-cell bar still reads as 32 px wide, contiguous and centred (both "
     "gaps zero), so only the neighbouring-gap-column check can catch it"),
    ("bar width that cannot be centred",
     "BAR_PX = 24", "BAR_PX = 21",
     "(32-21)//2 leaves 5 px on the low side and 6 on the high side"),
    ("cell width no longer tiles the bar area",
     "BAR_CELL = 32", "BAR_CELL = 30",
     "NBAND * BAR_CELL != BAR_W, and bar_idx = x>>5 stops matching"),
    ("L meter overlapping the bar area",
     "METER_L_Y, METER_H = 428, 12", "METER_L_Y, METER_H = 420, 12",
     "METER_L_Y must be below BAR_Y_LAST"),
    ("meters stacked on top of each other",
     "METER_R_Y = 441", "METER_R_Y = 438",
     "divider row 440 no longer sits between the two meters"),
    ("meter colour colliding with the bar base colour",
     "C_METER_L = 0xD8F4FF", "C_METER_L = 0x40C8FF",
     "byte-identical to C_BAR_LO; the palette gate must catch it"),
    ("peak cap drawn over the bar's own top row",
     "if in_bar_col and pr and rel_y >= pr and (rel_y - pr) <= 1:",
     "if in_bar_col and pr and rel_y <= pr and pr - rel_y <= 1:",
     "caps land at {pr-1, pr}, hiding one pixel of every bar"),
    ("peak cap drawn at peak == 0",
     "if in_bar_col and pr and rel_y >= pr and (rel_y - pr) <= 1:",
     "if in_bar_col and rel_y >= pr and (rel_y - pr) <= 1:",
     "a 2 px dash at the floor of all 16 bands during silence"),
    ("dB tick drawn one row above the bar it labels",
     "return BAR_Y_LAST - (env >> 1) + 1, env",
     "return BAR_Y_LAST - (env >> 1), env",
     "bar_pixel is rel_y < h, so the top lit row is BAR_Y_LAST - h + 1"),
    ("bar height mapping losing a bit",
     "def bar_height(env):\n    return env >> 1",
     "def bar_height(env):\n    return env >> 2",
     "full scale lights 31 of 64 rows and the dB ticks no longer match"),
    ("meter width mapping losing a bit",
     "def meter_width(lvl):\n    return lvl << 2",
     "def meter_width(lvl):\n    return lvl << 1",
     "full scale fills 254 of 512 px"),
]


def run_checks(tag, source):
    path = os.path.join(TMP, "_perturb_%s.py" % tag)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(source)
    name = "_perturb_%s" % tag
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            spec.loader.exec_module(mod)
            # short-circuit exactly like main(): if geometry fails, render never
            # runs, so a geometry failure must not be masked by a render crash
            fails = mod.check_geometry()
            if not fails:
                fails = mod.check_render()
    except Exception as exc:            # noqa: BLE001 - any crash counts as a bite
        fails = ["%s: %s" % (type(exc).__name__, exc)]
    finally:
        os.remove(path)
        sys.modules.pop(name, None)
    return fails


def main():
    print("=" * 72)
    print("render_spectrum_preview perturbation controls")
    print("=" * 72)

    base = run_checks("base", ORIG)
    if base:
        print("\nthe UNPERTURBED source already fails -- fix that first:")
        for f in base:
            print("  " + f)
        return 1
    print("baseline: unmodified source passes every assertion\n")

    missed = []
    for i, (label, old, new, why) in enumerate(PERTURB):
        if old not in ORIG:
            print("  ?? control %2d: anchor text not found -- harness is stale" % i)
            print("     %r" % old)
            missed.append(label)
            continue
        fails = run_checks("p%02d" % i, ORIG.replace(old, new, 1))
        if fails:
            print("  ok control %2d bites: %s" % (i, label))
            print("       -> %s" % fails[0])
        else:
            print("  FAIL control %2d does NOT bite: %s" % (i, label))
            print("       expected: %s" % why)
            missed.append(label)

    print("\n%d/%d controls bite" % (len(PERTURB) - len(missed), len(PERTURB)))
    if missed:
        print("assertions with no teeth:")
        for m in missed:
            print("  " + m)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        shutil.rmtree(TMP, ignore_errors=True)
