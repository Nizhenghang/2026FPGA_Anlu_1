#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate and validate the 16-band biquad bandpass coefficient table for
audio_visualizer.v.

Why this script gates the build
-------------------------------
The filter bank runs at an internal 24 kHz (the 48 kHz HDMI audio stream
decimated by two). At that rate the lowest band, 100 Hz, sits at
w0 = 0.0262 rad, where cos(w0) = 0.99966 and the a1 coefficient is within
0.0034 of the -2 boundary of a Q1.(W-2) format. Coarse quantization there does
three distinct kinds of damage, and each needs its own check:

  * the pole can be pushed onto or outside the unit circle -> the band
    oscillates forever instead of decaying (POLE),
  * the pole angle can move -> the band's center frequency drifts, so a tone
    lights the wrong bar (FREQ), and
  * the rounding bias injected at the state requantization point is amplified
    by the DC gain 1/(1 - NA1 - NA2) ~ (fs/(2*pi*f0))**2 -> a residual offset
    that stands bars up with no audio playing (DC).

The last one is the least obvious and the one that forced this design, because
it is independent of Q and depends only on how close the band sits to DC. It is
handled by carrying STATE_EXTRA fractional bits in the filter state and by
raising the lowest band from 60 Hz, where the amplification is 4053x, to
100 Hz, where it is 1458x. The measured residual is reported per band below.

Frequency drift is the one that is easy to miss after stability, because the
filter stays perfectly bounded while being wrong. A width that only just clears
MAX_CENTER_ERR is rejected even though it passes -- see SELECT_ERR_MARGIN.

Nothing is emitted unless every band passes every check. Run:

    python tools/gen_biquad_coeffs.py

The Verilog block is printed to stdout and also written to
tools/biquad_coeffs.vh for pasting into audio_visualizer.v.
"""

import cmath
import math
import os

# --------------------------------------------------------------------------
# Design targets. fs is the INTERNAL rate after decimation, not the 48 kHz
# audio rate: halving it doubles w0 for every band, which is what buys the
# coefficient precision the 60 Hz band needs.
# --------------------------------------------------------------------------
FS = 24000.0
NBAND = 16
F_LO = 100.0
F_HI = 7000.0
Q = 4.0

DATA_BITS = 16          # signed sample width presented to the envelope detector
STATE_BITS = 24         # filter state: the 16-bit output plus STATE_EXTRA fraction bits
STATE_EXTRA = 8         # see the note on quantization-driven DC offset below
ACC_BITS = 44           # products are at most 18+25 = 43 bits
IMPULSE_LEN = 2000

# Tolerated residual DC at the output, in 16-bit LSBs. The envelope detector
# maps |y|>>8, so anything under 256 cannot raise a bar at all; 8 is a tight
# bound chosen so the margin is obvious rather than marginal.
MAX_RESIDUAL_DC = 8

# Acceptance criteria, applied to the QUANTIZED coefficients.
MAX_POLE_RADIUS = 0.9995
MAX_CENTER_ERR = 0.02   # 2% of the nominal center frequency

# A width that only just clears MAX_CENTER_ERR is not a design margin. At 16
# bits the 132.7 Hz band lands at 1.79% against the 2% limit purely because of
# how its coefficients happened to round, so it would flip to failing if F_LO or
# Q moved. Require the selected width to clear half the limit as well. The extra
# bits are nearly free in hardware: the MAC engine is serial and shares one
# multiplier across all 16 bands, so this widens a single instance rather than
# sixteen.
SELECT_ERR_MARGIN = 0.5

CANDIDATE_WIDTHS = (16, 18, 20, 22, 24)


def band_frequencies():
    ratio = (F_HI / F_LO) ** (1.0 / (NBAND - 1))
    return [F_LO * ratio ** k for k in range(NBAND)]


def rbj_bandpass(f0, fs=None, q=None):
    """RBJ constant-0dB-peak bandpass, returned in the y[n] = B0*dx +
    NA1*y[n-1] + NA2*y[n-2] form the RTL uses.

    b1 is identically zero and b2 == -b0 for this topology, so the numerator
    collapses to B0*(x[n] - x[n-2]) and only three coefficients are stored.

    fs and q default to the module globals, read at call time rather than bound
    as defaults, so that magnitude() and peak_frequency() -- which use the
    globals directly -- always agree with this function.
    """
    if fs is None:
        fs = FS
    if q is None:
        q = Q
    w0 = 2.0 * math.pi * f0 / fs
    alpha = math.sin(w0) / (2.0 * q)
    a0 = 1.0 + alpha
    b0 = alpha / a0
    a1 = -2.0 * math.cos(w0) / a0
    a2 = (1.0 - alpha) / a0
    return {"b0": b0, "na1": -a1, "na2": -a2, "w0": w0, "alpha": alpha}


def quantize(c, width):
    """Q1.(width-2) signed: range [-2, +2), resolution 2**-(width-2)."""
    frac = width - 2
    scale = 1 << frac
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    out = {}
    clipped = False
    for k in ("b0", "na1", "na2"):
        v = int(round(c[k] * scale))
        if v < lo or v > hi:
            clipped = True
            v = max(lo, min(hi, v))
        out[k] = v
    out["frac"] = frac
    out["clipped"] = clipped
    return out


def poles(qc):
    """Roots of z^2 - NA1*z - NA2 for the quantized coefficients."""
    frac = qc["frac"]
    scale = float(1 << frac)
    na1 = qc["na1"] / scale
    na2 = qc["na2"] / scale
    disc = complex(na1 * na1 + 4.0 * na2, 0.0)
    root = cmath.sqrt(disc)
    return [(na1 + root) / 2.0, (na1 - root) / 2.0]


def magnitude(freq, qc):
    frac = qc["frac"]
    scale = float(1 << frac)
    b0 = qc["b0"] / scale
    na1 = qc["na1"] / scale
    na2 = qc["na2"] / scale
    w = 2.0 * math.pi * freq / FS
    z1 = cmath.exp(complex(0.0, -w))
    z2 = z1 * z1
    num = b0 * (1.0 - z2)
    den = 1.0 - na1 * z1 - na2 * z2
    if abs(den) < 1e-30:
        return float("inf")
    return abs(num / den)


def peak_frequency(qc, f_nominal):
    """Locate |H| maximum on a fine grid bracketing the nominal center.

    Bracketing rather than sweeping DC..Nyquist keeps this fast and avoids
    picking up the trivial response at the band edges.
    """
    lo = max(1.0, f_nominal * 0.5)
    hi = min(FS / 2.0 - 1.0, f_nominal * 1.6)
    n = 4000
    best_f, best_m = lo, -1.0
    for i in range(n + 1):
        f = lo + (hi - lo) * i / n
        m = magnitude(f, qc)
        if m > best_m:
            best_m, best_f = m, f
    return best_f, best_m


def sat(v, bits):
    hi = (1 << (bits - 1)) - 1
    lo = -(1 << (bits - 1))
    return hi if v > hi else (lo if v < lo else v)


def impulse_response(qc, round_half_up=True):
    """Fixed-point impulse response, mirroring the RTL exactly.

    The state carries STATE_EXTRA more fractional bits than the 16-bit output,
    which is the whole point of this function. Requantizing y to 16 bits every
    sample injects a half-LSB rounding bias directly into the feedback summing
    node. That injection point sees a DC gain of

        1 / (1 - NA1 - NA2) = a0 / (2*(1 - cos w0)) ~ (fs / (2*pi*f0))**2

    which is a property of how close the band sits to DC and is independent of
    Q. At 60 Hz and fs=24 kHz that is about 4053, so a half-LSB bias becomes a
    2027 LSB DC offset -- the lowest bars stand up on their own with no audio
    present. Keeping STATE_EXTRA fraction bits divides that by 2**STATE_EXTRA.

    Returns the 16-bit output trace.
    """
    frac = qc["frac"]
    b0, na1, na2 = qc["b0"], qc["na1"], qc["na2"]
    full = (1 << (DATA_BITS - 1)) - 1

    x0, x1, x2 = full, 0, 0   # x[n], x[n-1], x[n-2]
    s1, s2 = 0, 0             # filter state, Q STATE_EXTRA
    trace = []
    dx_seen = []
    for n in range(IMPULSE_LEN):
        # The numerator needs a TWO-sample delay. For a unit impulse this makes
        # dx read +full, 0, -full, then 0 forever; a single delay register would
        # give +full, -full, 0 and silently measure a first difference instead.
        dx = x0 - x2
        if n < 4:
            dx_seen.append(dx)
        acc = b0 * (dx << STATE_EXTRA) + na1 * s1 + na2 * s2
        if round_half_up:
            acc += 1 << (frac - 1)
        s_new = sat(acc >> frac, STATE_BITS)
        trace.append(sat(s_new >> STATE_EXTRA, DATA_BITS))
        x2, x1, x0 = x1, x0, 0   # only the first sample is an impulse
        s2, s1 = s1, s_new
    if dx_seen != [full, 0, -full, 0]:
        raise AssertionError("x delay line is not two samples: dx[:4] = %r"
                             % (dx_seen,))
    return trace


def reference_peaks(freqs):
    """Peak frequency of the unquantized design, computed once per band.

    This is the reference for separating two different errors: how far the
    topology itself sits from f_nominal, and how much further quantization
    pushes it.
    """
    refs = []
    for f0 in freqs:
        ideal = rbj_bandpass(f0)
        frac = 48
        scale = float(1 << frac)
        near = {k: int(round(ideal[k] * scale)) for k in ("b0", "na1", "na2")}
        near["frac"] = frac
        refs.append(peak_frequency(near, f0)[0])
    return refs


def check_width(width, freqs, refs):
    """Validate one candidate coefficient width across all bands."""
    rows = []
    ok = True
    for f0, f_float in zip(freqs, refs):
        ideal = rbj_bandpass(f0)
        qc = quantize(ideal, width)

        if qc["clipped"]:
            rows.append((f0, None, None, None, None, None, "CLIPPED"))
            ok = False
            continue

        pl = poles(qc)
        r = max(abs(p) for p in pl)

        f_q, gain = peak_frequency(qc, f0)
        err_total = abs(f_q - f0) / f0
        err_quant = abs(f_q - f_float) / f0

        trace = impulse_response(qc)
        # Decay must be monotonic once past the initial ring-up. Windows are a
        # full period of the lowest band so the comparison is envelope to
        # envelope, not sample to sample.
        w = int(FS / freqs[0])
        peaks = [max(abs(v) for v in trace[i:i + w])
                 for i in range(0, IMPULSE_LEN - w, w)]
        grows = any(b > a for a, b in zip(peaks[2:], peaks[3:]))
        # Residual DC with the input silent. This is the quantization-driven
        # offset described in impulse_response(), and it is what makes a bar
        # stand up with no audio playing.
        residual = max(abs(v) for v in trace[-w:])

        bad = []
        if r >= MAX_POLE_RADIUS:
            bad.append("POLE")
        if err_total > MAX_CENTER_ERR:
            bad.append("FREQ")
        if grows:
            bad.append("GROWS")
        if residual > MAX_RESIDUAL_DC:
            bad.append("DC=%d" % residual)
        if bad:
            ok = False
        rows.append((f0, r, f_q, err_total, err_quant, residual,
                     ",".join(bad) or "ok"))

    return ok, rows


def emit_verilog(width, freqs):
    frac = width - 2
    lines = []
    lines.append("// ---------------------------------------------------------------")
    lines.append("// Generated by tools/gen_biquad_coeffs.py -- do not edit by hand.")
    lines.append("//")
    lines.append("// %d-band log-spaced RBJ constant-0dB-peak bandpass bank," % NBAND)
    lines.append("// %.0f Hz .. %.0f Hz, Q = %.1f, internal sample rate %.0f Hz."
                 % (F_LO, F_HI, Q, FS))
    lines.append("// Coefficients are signed Q1.%d in %d bits." % (frac, width))
    lines.append("//")
    lines.append("// Difference equation, with b1 == 0 and b2 == -b0 exactly:")
    lines.append("//   y[n] = B0*(x[n] - x[n-2]) + NA1*y[n-1] + NA2*y[n-2]")
    lines.append("// so (x[n] - x[n-2]) is computed once and shared by all bands.")
    lines.append("// ---------------------------------------------------------------")
    lines.append("")
    lines.append("localparam COEFF_W = %d;" % width)
    lines.append("localparam COEFF_FRAC = %d;" % frac)
    lines.append("")

    for name, key, doc in (
        ("band_b0", "b0", "numerator, applied to x[n] - x[n-2]"),
        ("band_na1", "na1", "-a1, applied to y[n-1]"),
        ("band_na2", "na2", "-a2, applied to y[n-2]"),
    ):
        lines.append("// %s: %s" % (name, doc))
        lines.append("function signed [COEFF_W-1:0] %s;" % name)
        lines.append("    input [3:0] idx;")
        lines.append("    begin")
        lines.append("        case (idx)")
        for k, f0 in enumerate(freqs):
            v = quantize(rbj_bandpass(f0), width)[key]
            if v < 0:
                lit = "-%d'sd%d" % (width, -v)
            else:
                lit = "%d'sd%d" % (width, v)
            lines.append("            4'd%-2d: %s = %-16s // %7.1f Hz"
                         % (k, name, lit + ";", f0))
        lines.append("            default: %s = %d'sd0;" % (name, width))
        lines.append("        endcase")
        lines.append("    end")
        lines.append("endfunction")
        lines.append("")
    return "\n".join(lines)


def main():
    freqs = band_frequencies()

    print("16-band bank: %.0f..%.0f Hz, Q=%.1f, fs=%.0f Hz (48 kHz decimated by 2)"
          % (F_LO, F_HI, Q, FS))
    print("spacing ratio %.4f = %.3f octaves; Q=%.1f -3dB bandwidth ~%.3f octaves"
          % ((F_HI / F_LO) ** (1.0 / (NBAND - 1)),
             math.log((F_HI / F_LO) ** (1.0 / (NBAND - 1)), 2),
             Q,
             math.log2((1 + 1.0 / (2 * Q)) / (1 - 1.0 / (2 * Q)))))
    print()

    refs = reference_peaks(freqs)

    chosen = None
    chosen_rows = None
    for width in CANDIDATE_WIDTHS:
        ok, rows = check_width(width, freqs, refs)
        worst_r = max(r[1] for r in rows if r[1] is not None)
        worst_e = max(r[3] for r in rows if r[3] is not None)
        worst_d = max(r[5] for r in rows if r[5] is not None)
        margin_ok = worst_e <= MAX_CENTER_ERR * SELECT_ERR_MARGIN
        if not ok:
            verdict = "FAIL"
        elif not margin_ok:
            verdict = "PASS (rejected: no center-err margin)"
        else:
            verdict = "PASS"
        print("Q1.%-2d (%2d-bit): %s   worst pole r=%.6f   worst center err=%.2f%%"
              "   worst residual DC=%d LSB"
              % (width - 2, width, verdict, worst_r, worst_e * 100, worst_d))
        if not ok:
            for f0, r, fq, et, eq, residual, note in rows:
                if note != "ok":
                    print("        %7.1f Hz -> %s" % (f0, note))
        if ok and margin_ok and chosen is None:
            chosen = width
            chosen_rows = rows

    if chosen is None:
        print("\nNo candidate width is selectable. Not emitting anything.")
        print("Either a hard check failed, or none cleared the %.2f%% center-err"
              " selection threshold (MAX_CENTER_ERR * SELECT_ERR_MARGIN)."
              % (MAX_CENTER_ERR * SELECT_ERR_MARGIN * 100))
        return 1

    print("\nSelected Q1.%d (%d-bit signed coefficients)." % (chosen - 2, chosen))
    print()
    print("  band   f_nominal   pole_r    f_actual   err_total  err_quant   resid_dc")
    for k, (f0, r, fq, et, eq, residual, note) in enumerate(chosen_rows):
        print("  %2d    %9.1f   %.6f  %9.2f   %6.2f%%   %6.2f%%   %5d"
              % (k, f0, r, fq, et * 100, eq * 100, residual))

    text = emit_verilog(chosen, freqs)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "biquad_coeffs.vh")
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text + "\n")
    print("\nWrote %s" % out)
    print()
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
