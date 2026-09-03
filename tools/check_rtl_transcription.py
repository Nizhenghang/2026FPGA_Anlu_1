"""Transcription gate between audio_visualizer.v and the verified models.

There is no Verilog simulator on this machine, so the RTL cannot be executed.
What can be checked is that the RTL says what the models say. The three models
were each validated with their own negative controls; this script closes the
remaining gap, which is a hand-copy error between them and the .v file. It is
the gap that produced the original scrolling-waveform bug: nothing was
internally inconsistent, the drawing was just wrong.

Five independent comparisons, none of them against a value this script invents:

  coefficients  audio_visualizer.v  vs  gen_biquad_coeffs.py recomputed from
                scratch (deliberately NOT vs tools/biquad_coeffs.vh, so a bad
                paste into the header is caught too)
  geometry      audio_visualizer.v  vs  render_spectrum_preview.py
  palette       audio_visualizer.v  vs  render_spectrum_preview.py, including
                stage_rgb's priority order, which is a bug surface of its own
  arithmetic    audio_visualizer.v  vs  the shift amounts sim_biquad_bank.py and
                render_spectrum_preview.py actually use

Plus a structural check that the deleted waveform really is gone and that the
module's port list still matches its instantiation in top_tf_hdmi_audio.v.

The first three are constant comparisons, and constants are not where the danger
is. A Verilog `case` with a missing arm is silent: the state falls to `default`,
and if `default` assigns the same register as an earlier block its non-blocking
assignment wins. That killed the analyser's launch once and no constant check
could see it, so the FSM's arm coverage is checked too.

Arithmetic is the third such blind spot, and the expensive one. The envelope
target shipped reading `y_abs[23:8]` where the model applies two shifts totalling
`>> 16`, so every band's target was 256x too big and the whole analyser pinned at
full height -- a static colour wall on real hardware. Both 8s were correct in
isolation, the coefficients were correct, the geometry was correct, the palette
was correct and the FSM was correct. Every gate in this file passed. What was
wrong was a bit-slice, and nothing here compared one, so section 5 now derives
each shift from the model that owns it and checks the RTL's slice or concatenation
against it.

Run from anywhere:  python tools/check_rtl_transcription.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

import gen_biquad_coeffs as gc          # noqa: E402
import render_spectrum_preview as rp    # noqa: E402
import sim_biquad_bank as sb            # noqa: E402

# Derived from the model, not asserted from memory: five states run per band
# (LAUNCH0/1/2, ROUND, WRITE) and ST_IDLE is the sixth.
MAC_STATES = sb.MAC_STATES_PER_BAND + 1

RTL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source",
                   "audio_visualizer.v")
TOP = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source",
                   "top_tf_hdmi_audio.v")

FAILURES = []
CHECKS = [0]


def check(cond, what, detail=""):
    CHECKS[0] += 1
    if cond:
        print("    ok    %s" % what)
        return True
    FAILURES.append("%s%s" % (what, (" -- " + detail) if detail else ""))
    print("    FAIL  %s%s" % (what, (" -- " + detail) if detail else ""))
    return False


# --------------------------------------------------------------------------
# Localparam evaluation.
# --------------------------------------------------------------------------
LOCALPARAM_RE = re.compile(
    r"^\s*localparam\s+(?:\[\s*\d+\s*:\s*\d+\s*\]\s*)?(\w+)\s*=\s*([^;]+);",
    re.MULTILINE)

# Only plain integer geometry is evaluated. A sized literal, a concatenation, a
# bit-select or a bitwise/reduction operator is arithmetic the RTL does at run
# time, not a constant this script can compare. Shifts are handled below.
OPAQUE = ("'", "{", "[", ">", "<", "&", "|", "^", "~")

# Verilog >> and Python >> agree on these non-negative constants, so a shift is
# evaluated rather than skipped. It has to be removed before the probe for > and
# < or every dB tick row is silently dropped from the comparison.
SHIFTS = (">>>", "<<<", ">>", "<<")


def eval_localparams(text):
    """Return {name: int} for every localparam whose value is plain integer
    arithmetic over already-defined localparams."""
    out = {}
    skipped = []
    for name, expr in LOCALPARAM_RE.findall(text):
        expr = expr.strip()
        probe = expr
        for sh in SHIFTS:
            probe = probe.replace(sh, "")
        if any(tok in probe for tok in OPAQUE):
            skipped.append(name)
            continue
        # Every division in the geometry is exact and on integers; Python 3's /
        # would silently turn BAR_OFF_LO into a float and 27/2 into 13.5.
        py = expr.replace("/", "//")
        try:
            val = eval(py, {"__builtins__": {}}, dict(out))   # noqa: S307
        except Exception:                                        # noqa: BLE001
            skipped.append(name)
            continue
        out[name] = int(val)
    return out, skipped


# --------------------------------------------------------------------------
# 1. Coefficients.
# --------------------------------------------------------------------------
COEFF_RE = re.compile(
    r"4'd(\d+)\s*:\s*(band_b0|band_na1|band_na2)\s*=\s*(-?)18'sd(\d+)\s*;")


def check_coefficients(text):
    print("\n[1] coefficient ROM vs gen_biquad_coeffs.py recomputed")

    width = 18

    # The fixed-point format is the generator's decision, not the RTL's. These
    # are the widths gen_biquad_coeffs.py searched for and proved, so the RTL
    # declaring anything else silently invalidates every assertion behind them.
    lp, _ = eval_localparams(text)
    for name, want in (("COEFF_W", width),
                       ("COEFF_FRAC", width - 2),
                       ("STATE_BITS", gc.STATE_BITS),
                       ("STATE_EXTRA", gc.STATE_EXTRA),
                       ("ACC_BITS", gc.ACC_BITS),
                       ("NBAND", gc.NBAND),
                       ("WIN_BITS", 8)):
        check(lp.get(name) == want,
              "RTL %s is %d, the value the generator selected" % (name, want),
              "RTL has %s" % lp.get(name))

    frac = lp.get("COEFF_FRAC", width - 2)
    rnd = re.findall(r"acc\s*\+\s*prod\s*\+\s*44'sd(\d+)", text)
    check(len(rnd) == 1 and int(rnd[0]) == 1 << (frac - 1),
          "the write-back rounds by exactly 1 << (COEFF_FRAC - 1) = %d"
          % (1 << (frac - 1)), "%s" % rnd)
    check(len(re.findall(r">>>\s*COEFF_FRAC", text)) == 1,
          "the write-back shifts right by COEFF_FRAC exactly once")

    freqs = gc.band_frequencies()
    want = {}
    for k, f0 in enumerate(freqs):
        q = gc.quantize(gc.rbj_bandpass(f0), width)
        for key in ("b0", "na1", "na2"):
            want[("band_" + key, k)] = q[key]

    got = {}
    for idx, name, sign, mag in COEFF_RE.findall(text):
        v = int(mag)
        got[(name, int(idx))] = -v if sign else v

    check(len(got) == gc.NBAND * 3,
          "RTL declares all %d coefficients" % (gc.NBAND * 3),
          "found %d" % len(got))

    missing = sorted(set(want) - set(got))
    extra = sorted(set(got) - set(want))
    check(not missing, "no band/term is missing from the RTL", "%s" % missing[:6])
    check(not extra, "no band/term appears in the RTL that the generator lacks",
          "%s" % extra[:6])

    bad = [(k, want[k], got.get(k)) for k in sorted(want) if want[k] != got.get(k)]
    check(not bad, "every coefficient matches the regenerated value",
          "%s" % bad[:6])

    # The header this was pasted from is an intermediate. If it has drifted from
    # the generator, the RTL could match the header and still be wrong, so the
    # header is compared too rather than trusted.
    vh = os.path.join(TOOLS, "biquad_coeffs.vh")
    if os.path.exists(vh):
        with open(vh, "r", encoding="utf-8") as fh:
            vgot = {}
            for idx, name, sign, mag in COEFF_RE.findall(fh.read()):
                v = int(mag)
                vgot[(name, int(idx))] = -v if sign else v
        vbad = [(k, want[k], vgot.get(k)) for k in sorted(want)
                if want[k] != vgot.get(k)]
        check(not vbad, "tools/biquad_coeffs.vh still matches the generator",
              "%s" % vbad[:6])


# --------------------------------------------------------------------------
# 2. Geometry.
# --------------------------------------------------------------------------
# RTL localparam name -> the preview constant it must equal. Names differ where
# the preview computes a last-inclusive bound inline.
GEOMETRY_MAP = [
    ("PANEL_X", rp.PANEL_X), ("PANEL_Y", rp.PANEL_Y),
    ("PANEL_W", rp.PANEL_W), ("PANEL_H", rp.PANEL_H),
    ("PANEL_X_LAST", rp.PANEL_X_LAST), ("PANEL_Y_LAST", rp.PANEL_Y_LAST),
    ("BAR_X", rp.BAR_X), ("BAR_Y", rp.BAR_Y),
    ("BAR_W", rp.BAR_W), ("BAR_H", rp.BAR_H),
    ("BAR_X_LAST", rp.BAR_X_LAST), ("BAR_Y_LAST", rp.BAR_Y_LAST),
    ("BAR_CELL", rp.BAR_CELL), ("BAR_PX", rp.BAR_PX),
    ("BAR_OFF_LO", rp.BAR_OFF_LO), ("BAR_OFF_HI", rp.BAR_OFF_HI),
    ("NBAND", rp.NBAND),
    ("LEFT_X", rp.LEFT_X), ("LEFT_W", rp.LEFT_W),
    ("LEFT_X_LAST", rp.LEFT_X + rp.LEFT_W - 1),
    ("RIGHT_X", rp.RIGHT_X), ("RIGHT_W", rp.RIGHT_W),
    ("RIGHT_X_LAST", rp.RIGHT_X + rp.RIGHT_W - 1),
    ("METER_X", rp.METER_X), ("METER_W", rp.METER_W),
    ("METER_X_LAST", rp.METER_X + rp.METER_W - 1),
    ("METER_L_Y", rp.METER_L_Y), ("METER_H", rp.METER_H),
    ("METER_L_Y_LAST", rp.METER_L_Y + rp.METER_H - 1),
    ("METER_R_Y", rp.METER_R_Y),
    ("METER_R_Y_LAST", rp.METER_R_Y + rp.METER_H - 1),
]


def check_geometry(text):
    print("\n[2] panel geometry vs render_spectrum_preview.py")

    lp, skipped = eval_localparams(text)
    check(bool(lp), "localparams parsed and evaluated", "%d found" % len(lp))

    bad = [(n, want, lp.get(n)) for n, want in GEOMETRY_MAP if lp.get(n) != want]
    check(not bad, "every geometry localparam equals the preview's constant",
          "%s" % bad)

    # The dB ticks are derived in both places from the same formula, so
    # comparing them to each other is weak. Compare against db_row(), which the
    # preview's own check_render verifies against a bar it actually renders.
    for db in rp.DB_MARKS:
        y_tick, env_mark = rp.db_row(db)
        yname = "DB%d_Y" % db
        ename = "DB%d_ENV" % db
        check(lp.get(yname) == y_tick,
              "%s is the row db_row(%d) verified against a rendered bar" % (yname, db),
              "RTL %s, preview %s" % (lp.get(yname), y_tick))
        check(lp.get(ename) == env_mark,
              "%s is the envelope db_row(%d) derived" % (ename, db),
              "RTL %s, preview %s" % (lp.get(ename), env_mark))

    # Tick spans, which the preview writes inline in panel_pixel.
    check(lp.get("TICK_LEFT_HI_W") == rp.LEFT_X + 3,
          "left dB tick spans the preview's LEFT_X .. LEFT_X+3",
          "RTL %s, preview %s" % (lp.get("TICK_LEFT_HI_W"), rp.LEFT_X + 3))
    check(lp.get("TICK_RIGHT_LO_W") == rp.RIGHT_X + rp.RIGHT_W - 4,
          "right dB tick spans the preview's last four columns",
          "RTL %s, preview %s" % (lp.get("TICK_RIGHT_LO_W"),
                                  rp.RIGHT_X + rp.RIGHT_W - 4))


# --------------------------------------------------------------------------
# 3. Palette and priority order.
# --------------------------------------------------------------------------
# (predicate as it appears in stage_rgb, colour constant in the preview). The
# ORDER is part of the expectation: peak cap beats bar beats meter beats chip
# beats border beats tick beats grid beats dimmed panel.
STAGE_ORDER = [
    ("peak_pixel", rp.C_PEAK),
    ("bar_pixel", None),                      # bar_rgb, checked separately
    ("meter_l_pixel", rp.C_METER_L),
    ("meter_r_pixel", rp.C_METER_R),
    ("chip_l", rp.C_METER_L),
    ("chip_r", rp.C_METER_R),
    ("panel_border", rp.C_BORDER),
    ("db_tick", rp.C_TICK),
    ("grid_line", rp.C_GRID),
    ("in_panel", None),                       # panel_rgb
]


def check_palette(text):
    print("\n[3] palette and stage_rgb priority vs render_spectrum_preview.py")

    m = re.search(r"assign\s+stage_rgb\s*=(.*?);", text, re.DOTALL)
    if not check(m is not None, "stage_rgb assignment found"):
        return
    body = m.group(1)

    # Splitting on '?' puts each level's RESULT at the head of the next part,
    # after a ':'. So part 0 is the top condition and every later part yields
    # one result followed by one condition. Pairing parts[i] with parts[i]
    # instead compares each condition against the colour one level too high.
    parts = [re.sub(r"\s+", " ", p).strip() for p in re.split(r"\?", body)]
    conds = [parts[0]]
    results = []
    fallthrough = None
    for j, seg in enumerate(parts[1:]):
        res, _, nxt = seg.partition(":")
        results.append(res.strip())
        if j < len(parts) - 2:
            conds.append(nxt.strip())
        else:
            # the tail of the last part is the value drawn when every condition
            # above it is false, not another condition
            fallthrough = nxt.strip()

    check(len(conds) == len(STAGE_ORDER),
          "stage_rgb has exactly %d priority levels" % len(STAGE_ORDER),
          "found %d: %s" % (len(conds), conds))

    for i, (want_cond, want_colour) in enumerate(STAGE_ORDER):
        if i >= len(conds):
            break
        check(conds[i] == want_cond,
              "priority %d is %s" % (i, want_cond),
              "RTL has %r" % conds[i])
        if want_colour is None:
            continue
        got = results[i].replace("24'h", "").strip()
        check(got.upper() == "%06X" % want_colour,
              "%s draws 24'h%06X" % (want_cond, want_colour),
              "RTL has %s" % results[i])

    check(results[1].strip() == "bar_rgb", "bar_pixel draws the gradient",
          "RTL has %s" % results[1])
    check(fallthrough == "I_rgb",
          "the fallthrough outside the panel is the untouched input",
          "RTL has %s" % fallthrough)

    mb = re.search(r"assign\s+bar_rgb\s*=(.*?);", text, re.DOTALL)
    if check(mb is not None, "bar_rgb assignment found"):
        gbody = re.sub(r"\s+", " ", mb.group(1))
        for bit, colour in (("bar_rel_y[5]", rp.C_BAR_HI),
                            ("bar_rel_y[4]", rp.C_BAR_MID)):
            pat = re.escape(bit) + r"\s*\?\s*24'h([0-9A-Fa-f]{6})"
            mm = re.search(pat, gbody)
            check(mm is not None and int(mm.group(1), 16) == colour,
                  "%s selects 24'h%06X" % (bit, colour),
                  "RTL has %s" % (mm.group(1) if mm else "nothing"))
        mm = re.search(r":\s*24'h([0-9A-Fa-f]{6})\s*$", gbody)
        check(mm is not None and int(mm.group(1), 16) == rp.C_BAR_LO,
              "the bar base colour is 24'h%06X" % rp.C_BAR_LO,
              "RTL has %s" % (mm.group(1) if mm else "nothing"))

    # The preview's palette gate already proves these nine are distinct; the
    # point of re-checking here is that the RTL did not introduce a tenth.
    lits = sorted(set(int(h, 16) for h in re.findall(r"24'h([0-9A-Fa-f]{6})", text)))
    allowed = sorted({rp.C_BORDER, rp.C_GRID, rp.C_BAR_HI, rp.C_BAR_MID,
                      rp.C_BAR_LO, rp.C_PEAK, rp.C_METER_L, rp.C_METER_R,
                      rp.C_TICK})
    check(lits == allowed,
          "the RTL uses exactly the preview's nine colours and no others",
          "extra %s, missing %s" % ([hex(c) for c in lits if c not in allowed],
                                    [hex(c) for c in allowed if c not in lits]))


# --------------------------------------------------------------------------
# 4. Structure.
# --------------------------------------------------------------------------
GONE = ["wave_mem", "write_idx", "sample_div", "last_sample", "zero_distance",
        "period_est", "energy_hold", "peak_hold", "WAVE_", "sample_to_y",
        "wave_pixel", "center_line", "sparkle", "bin_level", "get_bin_level",
        "get_peak_level", "update_bins", "decay_bins"]

PORTS = ["I_clk", "I_rst", "I_de", "I_frame_start", "I_rgb", "I_audio_valid",
         "I_audio_left", "I_audio_right", "O_rgb"]


def check_structure(text):
    print("\n[4] structure")

    for name in GONE:
        check(name not in text, "%s is gone" % name.rstrip("_"))

    for p in PORTS:
        check(re.search(r"\b%s\b" % p, text) is not None,
              "port %s is still declared" % p)

    with open(TOP, "r", encoding="utf-8") as fh:
        top = fh.read()
    mi = re.search(r"\)\s*u_audio_visualizer\s*\((.*?)\);", top, re.DOTALL)
    if check(mi is not None, "instantiation found in top_tf_hdmi_audio.v"):
        connected = re.findall(r"\.(\w+)\s*\(", mi.group(1))
        check(sorted(connected) == sorted(PORTS),
              "the instantiation connects exactly the module's ports",
              "top has %s" % sorted(connected))

    # A reg driven from two always blocks is a multiple-driver error that only
    # synthesis reports. Cheap to rule out here.
    blocks = re.findall(r"always\s*@\([^)]*\)\s*begin(.*?)\n\s*end\n", text,
                        re.DOTALL)
    check(len(blocks) >= 2, "both always blocks were located",
          "%d found" % len(blocks))
    driven = {}
    for bi, b in enumerate(blocks):
        for lhs in re.findall(r"^\s*(\w+)\s*(?:\[[^\]]*\])?\s*<=", b, re.MULTILINE):
            driven.setdefault(lhs, set()).add(bi)
        for lhs in re.findall(r"^\s*(\w+)\s*\[(?:[^\]]*)\]\s*<=", b, re.MULTILINE):
            driven.setdefault(lhs, set()).add(bi)
    multi = {k: v for k, v in driven.items() if len(v) > 1}
    check(not multi, "no register is driven from two always blocks", "%s" % multi)

    # Every non-blocking assignment inside the FSM case must be reachable from
    # exactly one state arm; a stray assignment outside the case would run every
    # cycle and silently override the sequenced ones.
    mm = re.search(r"case\s*\(mac_state\)(.*?)endcase", text, re.DOTALL)
    if check(mm is not None, "the mac_state case block was located"):
        case_body = mm.group(1)
        rest = text.replace(case_body, "")
        for sig in ("acc", "prod", "band_cnt", "mac_state"):
            inside = len(re.findall(r"^\s*%s\s*<=" % sig, case_body, re.MULTILINE))
            outside = len(re.findall(r"^\s*%s\s*<=" % sig, rest, re.MULTILINE))
            # Two, not one: the reset branch and the launch branch. A third site
            # would run every cycle and silently override the sequenced value.
            check(outside == 2,
                  "%s is assigned outside the FSM case only by reset and launch"
                  % sig, "%d outside" % outside)
            check(inside >= 1, "%s is sequenced inside the FSM case" % sig,
                  "%d inside" % inside)

        # A case arm that is missing does not error and does not warn: the state
        # silently falls to `default`, which here assigns mac_state <= ST_IDLE.
        # Because the case runs after the launch block in source order, that is
        # the later non-blocking assignment and it cancels the launch. A missing
        # ST_IDLE arm is therefore a dead analyser that every other check in
        # this file passes. Found once by hand-tracing; this is the mechanical
        # version.
        states = re.findall(r"^\s*localparam\s+(ST_\w+)\s*=", text, re.MULTILINE)
        check(len(states) == MAC_STATES, "the FSM declares %d states" % MAC_STATES,
              "%d declared: %s" % (len(states), states))
        arms = set(re.findall(r"^\s*(ST_\w+)\s*:", case_body, re.MULTILINE))
        for st in states:
            check(st in arms, "%s has an explicit arm in case (mac_state)" % st,
                  "falls to default, which overrides the launch")
        check(re.search(r"^\s*default\s*:", case_body, re.MULTILINE) is not None,
              "the FSM case has a default arm")


# --------------------------------------------------------------------------
# 5. Arithmetic scaling.
# --------------------------------------------------------------------------
#
# Every expected value here is read out of the model that owns it, so changing a
# model moves this gate with it instead of leaving the gate asserting a number
# nobody believes any more.
BANK_SRC = sb.__file__
PREVIEW_SRC = rp.__file__


def model_match(path, pattern, what):
    """Search a model's source and return the match, or None.

    A pattern that stops matching is a hard failure, not a skip. The quiet
    alternative is a gate that keeps printing "all checks pass" while checking
    one thing less than it did yesterday.
    """
    with open(path, "r", encoding="utf-8") as fh:
        m = re.search(pattern, fh.read())
    if not check(m is not None, "model expression located: %s" % what,
                 "%r no longer matches %s" % (pattern, os.path.basename(path))):
        return None
    return m


def model_expr(path, pattern, what):
    m = model_match(path, pattern, what)
    return m.group(1) if m else None


def check_arithmetic(text):
    print("\n[5] arithmetic scaling")

    bank = sb.Bank(sb.make_coeffs())

    # -- (a) the envelope target -------------------------------------------
    # The bug that shipped. y_new is the 24-bit Q8 STATE, so reaching the
    # envelope's 0..127 range takes two shifts: state_extra to get the 16-bit
    # filter output, then env_shift to get the envelope. The model applies both
    # (`y = s_new >> state_extra`, then `target = abs(y) >> env_shift`); the RTL
    # applied only the first. Both 8s were individually correct, which is why no
    # constant comparison could see it.
    model_match(BANK_SRC, r"y\s*=\s*sat\(s_new\s*>>\s*self\.state_extra",
                "the state -> output shift")
    model_match(BANK_SRC, r"target\s*=\s*abs\(y\)\s*>>\s*self\.env_shift",
                "the output -> envelope shift")
    lo = bank.state_extra + bank.env_shift
    hi = sb.STATE_BITS - 1

    m = re.search(
        r"wire\s+\[\s*(\d+)\s*:\s*0\s*\]\s+target\s*=\s*"
        r"\{\s*(\d+)'d0\s*,\s*y_abs\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*\}\s*;", text)
    if check(m is not None,
             "the envelope target is a zero-extended slice of y_abs"):
        width, pad, got_hi, got_lo = (int(m.group(1)), int(m.group(2)),
                                      int(m.group(3)), int(m.group(4)))
        if got_lo == lo:
            check(True, "the target slice is y_abs[%d:%d], both model shifts "
                        "applied (state_extra %d + env_shift %d)"
                  % (hi, lo, bank.state_extra, bank.env_shift))
        else:
            scale = 2 ** abs(lo - got_lo)
            check(False, "the target slice is y_abs[%d:%d], both model shifts "
                         "applied (state_extra %d + env_shift %d)" % (hi, lo,
                  bank.state_extra, bank.env_shift),
                  "RTL has y_abs[%d:%d], so the target is %dx too %s and every "
                  "band above -48 dBFS pins env at 127"
                  % (got_hi, got_lo, scale, "big" if got_lo < lo else "small"))
        check(got_hi == hi, "the slice reaches the state's sign-magnitude top",
              "y_abs[%d:..] but STATE_BITS is %d" % (got_hi, sb.STATE_BITS))
        check(pad + (got_hi - got_lo + 1) == width + 1,
              "the zero pad brings the slice up to the declared target width",
              "%d + %d bits into a [%d:0] wire" % (pad, got_hi - got_lo + 1, width))

    # -- (b) the dx8 pre-shift ---------------------------------------------
    # The model left-shifts dx by state_extra so the state carries fraction bits.
    # The RTL writes that as a concatenation, which is the same arithmetic in a
    # form no constant comparison recognises.
    model_match(BANK_SRC, r"dx8\s*=\s*dx\s*<<\s*self\.state_extra",
                "the dx pre-shift into state format")
    m = re.search(
        r"wire\s+signed\s+\[\s*(\d+)\s*:\s*0\s*\]\s+dx8_w\s*=\s*"
        r"\{\s*dx_w\s*,\s*(\d+)'b(0+)\s*\}\s*;", text)
    if check(m is not None, "dx8_w is dx_w concatenated with a zero pad"):
        # 8'b00 is a sized literal: the width prefix says how many bits it
        # contributes, the digits only say what value they hold. Counting the
        # digits would read this as a 2-bit pad.
        check(int(m.group(2)) == bank.state_extra,
              "the dx8 pad is %d zero bits, matching state_extra"
              % bank.state_extra, "%s'b%s contributes %s bits"
              % (m.group(2), m.group(3), m.group(2)))
        check(set(m.group(3)) == {"0"}, "the dx8 pad's value is zero",
              "%s'b%s" % (m.group(2), m.group(3)))
        check(int(m.group(1)) == sb.DATA_BITS + bank.state_extra,
              "dx8_w is wide enough for the shifted dx",
              "[%s:0] holds %d bits, dx is %d and the pad is %d"
              % (m.group(1), int(m.group(1)) + 1, sb.DATA_BITS + 1,
                 bank.state_extra))

    # -- (c) the L/R meter slice -------------------------------------------
    # These read the raw channels and never touch the filter bank. That is why
    # they kept working while the bars were pinned, and why "the meters look
    # fine" is not evidence the analyser is.
    am = re.search(r"input\s+wire\s+\[\s*(\d+)\s*:\s*0\s*\]\s+I_audio_left", text)
    if check(am is not None, "the audio port width was read from the RTL"):
        audio_bits = int(am.group(1)) + 1
        mshift = model_expr(BANK_SRC, r"abs\(left\)\s*>>\s*(\d+)",
                            "the raw channel -> meter shift")
        if mshift is not None:
            # `abs(left) >> 16` on a 24-bit value keeps bits 16..23, so the
            # shift IS the slice's low index. Reading it as `bits-1-shift`
            # looks equally plausible and is off by twice the difference.
            want = "[%d:%s]" % (audio_bits - 1, mshift)
            for name in ("abs_l", "abs_r"):
                slices = set(re.findall(r"%s\[\s*(\d+)\s*:\s*(\d+)\s*\]" % name,
                                        text))
                check(slices == {(str(audio_bits - 1), mshift)},
                      "every %s slice is %s, the meter shift alone" % (name, want),
                      "found %s" % sorted(slices))

    # -- (d) the mix16 pre-shift -------------------------------------------
    # A bit-select is UNSIGNED in Verilog, so `I_audio_left[23:8]` alone turns a
    # negative half-cycle into a large positive number. The sign bit has to be
    # re-attached, and forgetting it is silent in exactly the same way.
    mm = model_match(BANK_SRC,
                     r"\(left\s*>>\s*(\d+)\)\s*\+\s*\(right\s*>>\s*(\d+)\)",
                     "the channel -> 16-bit mix shift")
    if mm is not None:
        lm = mm.group(1)
        check(lm == mm.group(2),
              "the model shifts both channels by the same amount before mixing",
              "left >> %s but right >> %s" % (lm, mm.group(2)))
        if am is not None:
            audio_bits = int(am.group(1)) + 1
            want = (str(audio_bits - 1), lm)
            for ch in ("left", "right"):
                m = re.search(r"\{\s*I_audio_%s\[\s*(\d+)\s*\]\s*,\s*"
                              r"I_audio_%s\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*\}" % (ch, ch),
                              text)
                if check(m is not None,
                         "the %s mix term re-attaches its sign bit" % ch):
                    check(m.group(1) == str(audio_bits - 1),
                          "the re-attached bit is %s's sign bit" % ch,
                          "I_audio_%s[%s] but the port is %d bits"
                          % (ch, m.group(1), audio_bits))
                    check((m.group(2), m.group(3)) == want,
                          "the %s mix slice is [%s:%s], the model's mix shift"
                          % (ch, want[0], want[1]),
                          "found [%s:%s]" % (m.group(2), m.group(3)))

    # -- (e,f) bar height and peak row -------------------------------------
    # Both come from the preview, which verifies them by actually rendering a
    # frame rather than by arithmetic.
    for fn, arg, sig, rtl_sig, reg in (
            ("bar_height", "env", "bar_h", "bar_env", "env"),
            ("peak_row", "peak", "peak_row", "bar_peak", "peak")):
        shift = model_expr(PREVIEW_SRC,
                           r"def %s\(%s\):\s*\n\s*return %s >> (\d+)" % (fn, arg, arg),
                           "%s's shift" % fn)
        if shift is None:
            continue
        # The slice's top index is the register's own msb, parsed rather than
        # remembered: a literal 7 here would keep passing after the envelope
        # widened and would then be asserting nothing.
        em = re.search(r"reg\s+\[\s*(\d+)\s*:\s*0\s*\]\s+%s\s*\[" % reg, text)
        if not check(em is not None, "the %s register's width was read from the RTL"
                     % reg):
            continue
        m = re.search(r"assign\s+%s\s*=\s*%s\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*;"
                      % (sig, rtl_sig), text)
        if check(m is not None, "%s is a slice of %s" % (sig, rtl_sig)):
            check(int(m.group(2)) == int(shift),
                  "%s drops %s bits, matching %s()" % (sig, shift, fn),
                  "RTL slices [%s:%s], so %s is %d px per %s step instead of %d"
                  % (m.group(1), m.group(2), fn, 2 ** int(m.group(2)), reg,
                     2 ** int(shift)))
            check(int(m.group(1)) == int(em.group(1)),
                  "%s keeps the whole %s register" % (sig, reg),
                  "sliced from bit %s but %s is declared [%s:0]"
                  % (m.group(1), reg, em.group(1)))

    # -- (g) meter width ---------------------------------------------------
    shift = model_expr(PREVIEW_SRC,
                       r"def meter_width\(lvl\):\s*\n\s*return lvl << (\d+)",
                       "meter_width's shift")
    if shift is not None:
        for name in ("meter_l_w", "meter_r_w"):
            m = re.search(r"assign\s+%s\s*=\s*\{\s*(\d+)'d0\s*,\s*lvl_[lr]\s*\}"
                          r"\s*<<\s*(\d+)\s*;" % name, text)
            if check(m is not None, "%s is a zero-extended left shift" % name):
                check(int(m.group(2)) == int(shift),
                      "%s shifts by %s, matching meter_width()" % (name, shift),
                      "RTL shifts by %s" % m.group(2))


def main():
    print("=" * 72)
    print("audio_visualizer.v transcription gate")
    print("=" * 72)

    with open(RTL, "r", encoding="utf-8") as fh:
        text = fh.read()

    check_coefficients(text)
    check_geometry(text)
    check_palette(text)
    check_structure(text)
    check_arithmetic(text)

    print("\n" + "=" * 72)
    if FAILURES:
        print("%d/%d checks FAILED:" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  " + f)
        return 1
    print("all %d checks pass" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
