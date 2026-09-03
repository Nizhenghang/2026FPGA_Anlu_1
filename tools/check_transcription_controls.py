"""Perturbation harness for check_rtl_transcription.py.

The transcription gate is regex-based, and a regex that silently matches nothing
reports success just as loudly as one that matched everything. This perturbs a
copy of audio_visualizer.v eighteen ways, each breaking one specific thing the
gate claims to compare, and asserts the gate reports a failure.

The RTL itself is never modified; the copies live in a temp directory.

Controls 11 and 12 are not hypothetical. They are the two hardware-fatal bugs
this project has actually shipped -- the ST_IDLE case arm whose absence left the
analyser dead, and the envelope target that took one shift instead of two and
pinned all sixteen bars at full height. Both were found by hand-tracing after the
board misbehaved, both were invisible to every constant comparison in the gate,
and both are now reproduced here so the checks that catch them can be shown to
have teeth. The rest guard against copy errors that have not happened yet.

Run from anywhere:  python tools/check_transcription_controls.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

sys.dont_write_bytecode = True

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

import check_rtl_transcription as gate    # noqa: E402

RTL = gate.RTL
with open(RTL, "r", encoding="utf-8") as fh:
    ORIG = fh.read()

TMP = tempfile.mkdtemp(prefix="rtl_controls_")

# (label, old_text, new_text, which check must catch it)
PERTURB = [
    ("one coefficient transposed",
     "4'd5 : band_b0 = 18'sd870;", "4'd5 : band_b0 = 18'sd780;",
     "the regenerated coefficient comparison"),
    ("a coefficient sign flipped",
     "4'd15: band_na1 = -18'sd30269;", "4'd15: band_na1 = 18'sd30269;",
     "the regenerated coefficient comparison"),
    ("coefficient width narrowed",
     "localparam COEFF_W     = 18;", "localparam COEFF_W     = 16;",
     "the fixed-point format comparison against the generator"),
    ("state fraction bits removed, reintroducing the DC offset",
     "localparam STATE_EXTRA = 8;", "localparam STATE_EXTRA = 0;",
     "the fixed-point format comparison against the generator"),
    ("accumulator narrowed",
     "localparam ACC_BITS    = 44;", "localparam ACC_BITS    = 32;",
     "the fixed-point format comparison against the generator"),
    ("panel pushed off the bottom of the screen",
     "localparam PANEL_H = 104;", "localparam PANEL_H = 130;",
     "the geometry comparison against render_spectrum_preview"),
    ("bar widened into its own gap",
     "localparam BAR_PX     = 24;", "localparam BAR_PX     = 32;",
     "the geometry comparison, via BAR_OFF_LO/BAR_OFF_HI"),
    ("dB tick drawn one row above the bar it labels",
     "localparam DB6_Y    = BAR_Y_LAST - (DB6_ENV  >> 1) + 1;",
     "localparam DB6_Y    = BAR_Y_LAST - (DB6_ENV  >> 1);",
     "the comparison against db_row(), which the preview verifies by rendering"),
    ("meter colour colliding with the bar base colour",
     "meter_l_pixel ? 24'hD8F4FF :", "meter_l_pixel ? 24'h40C8FF :",
     "the palette comparison against the preview's nine colours"),
    ("peak cap demoted below the bar it caps",
     "assign stage_rgb = peak_pixel    ? 24'hFFE060 :\n"
     "                   bar_pixel     ? bar_rgb :",
     "assign stage_rgb = bar_pixel     ? bar_rgb :\n"
     "                   peak_pixel    ? 24'hFFE060 :",
     "the stage_rgb priority-order comparison"),
    ("a deleted waveform register resurrected",
     "reg [9:0] x_pos;", "reg [5:0] write_idx;\nreg [9:0] x_pos;",
     "the structural check that the waveform really is gone"),
    # This is the one hardware-fatal bug that no constant comparison can see,
    # and the reason the FSM arm coverage check exists. Deleting the arm makes
    # ST_IDLE fall to `default`, whose mac_state <= ST_IDLE is the later
    # non-blocking assignment and cancels the launch every cycle.
    ("the ST_IDLE case arm deleted, so the analyser never starts",
     "            ST_IDLE: begin\n"
     "                // Deliberately empty. This case executes after the launch above\n"
     "                // in source order, so if ST_IDLE fell through to `default` its\n"
     "                // `mac_state <= ST_IDLE` would be the later non-blocking\n"
     "                // assignment and would cancel the launch every time.\n"
     "            end\n"
     "            ST_LAUNCH0: begin",
     "            ST_LAUNCH0: begin",
     "the check that every declared ST_* state has an explicit case arm"),

    # Controls 12..17 cover section [5], arithmetic scaling. Control 12 is the
    # second hardware-fatal bug this project has shipped, and like control 11 it
    # is not hypothetical: the board showed sixteen bars pinned at full height,
    # every colour tier lit, while the L/R meters -- which never touch this path
    # -- looked fine. Every constant, geometry, palette and structure check in
    # the gate passed it, because both 8s were individually correct and what was
    # wrong was the slice between them.
    ("the envelope target taking one shift instead of two, 256x too big",
     "wire [15:0] target = {8'd0, y_abs[23:16]};",
     "wire [15:0] target = {8'd0, y_abs[23:8]};",
     "the target slice comparison against state_extra + env_shift"),
    ("the dx8 state-format pre-shift halved",
     "wire signed [24:0] dx8_w = {dx_w, 8'b00};",
     "wire signed [24:0] dx8_w = {dx_w, 4'b00};",
     "the dx8 pad comparison against state_extra"),
    ("the L/R meter slice taken one bit low, halving the meters",
     "wire [7:0]  abs_l8 = (abs_l[23:16] == 8'd128) ? 8'd127 : abs_l[23:16];",
     "wire [7:0]  abs_l8 = (abs_l[23:15] == 8'd128) ? 8'd127 : abs_l[23:15];",
     "the meter slice comparison against the model's abs(left) >> 16"),
    ("the mix sign bit dropped, folding negative half-cycles positive",
     "wire signed [16:0] l_hi    = {I_audio_left[23],  I_audio_left[23:8]};",
     "wire signed [16:0] l_hi    = {1'b0,              I_audio_left[23:8]};",
     "the check that each mix term re-attaches its own sign bit"),
    ("the bar height shifted twice, so bars read half their envelope",
     "assign bar_h    = bar_env[7:1];",
     "assign bar_h    = bar_env[7:2];",
     "the bar_h comparison against the preview's bar_height()"),
    ("the meter width shift halved, so meters span a quarter of the panel",
     "assign meter_l_w   = {2'd0, lvl_l} << 2;",
     "assign meter_l_w   = {2'd0, lvl_l} << 1;",
     "the meter width comparison against the preview's meter_width()"),
]


def run_gate(tag, source):
    path = os.path.join(TMP, "av_%s.v" % tag)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(source)
    gate.RTL = path
    gate.FAILURES = []
    gate.CHECKS = [0]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = gate.main()
        return rc, gate.FAILURES
    except Exception as exc:            # noqa: BLE001 - a crash is a bite too
        return 1, ["%s: %s" % (type(exc).__name__, exc)]
    finally:
        os.remove(path)


def main():
    print("=" * 72)
    print("check_rtl_transcription perturbation controls")
    print("=" * 72)

    rc, fails = run_gate("base", ORIG)
    if rc != 0:
        print("\nthe UNMODIFIED RTL already fails the gate -- fix that first:")
        for f in fails:
            print("  " + f)
        return 1
    print("baseline: audio_visualizer.v passes all %d checks\n" % gate.CHECKS[0])

    missed = []
    for i, (label, old, new, which) in enumerate(PERTURB):
        if old not in ORIG:
            print("  ?? control %2d: anchor text not found -- harness is stale" % i)
            print("     %r" % old)
            missed.append(label)
            continue
        rc, fails = run_gate("p%02d" % i, ORIG.replace(old, new, 1))
        if rc != 0:
            print("  ok control %2d bites: %s" % (i, label))
            print("       -> %s" % fails[0])
        else:
            print("  FAIL control %2d does NOT bite: %s" % (i, label))
            print("       expected it to be caught by: %s" % which)
            missed.append(label)

    print("\n%d/%d controls bite" % (len(PERTURB) - len(missed), len(PERTURB)))
    if missed:
        print("gate checks with no teeth:")
        for m in missed:
            print("  " + m)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        gate.RTL = RTL
        shutil.rmtree(TMP, ignore_errors=True)
