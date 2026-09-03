#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle-accurate model of the audio_visualizer.v bandpass filter bank.

Why this exists
---------------
There is no iverilog or verilator on this machine, so the filter bank is
verified here before it is synthesized. The model mirrors the RTL's arithmetic
exactly -- shift widths, rounding, saturation order -- and every test carries a
negative control, because a test that cannot fail does not tell you anything.

Coefficients are imported from gen_biquad_coeffs.py rather than restated, so
the model cannot drift away from the ROM that goes into the RTL.

Structure of the model
----------------------
Two levels, cross-checked against each other:

  step_sample()   one internal (24 kHz) sample: all 16 bands, exact arithmetic.
                  This is what the signal-level tests run, because Python cannot
                  afford 1049 cycles per sample over tens of thousands of samples.

  cycle_replay()  a literal cycle-by-cycle walk of the MAC finite state machine
                  for a single sample. A test asserts it lands on exactly the
                  same state as step_sample(), which is what licenses the fast
                  path, and it counts cycles so the 1049-cycle budget is
                  measured rather than assumed.

Non-blocking semantics: every function reads the pre-edge snapshot into locals,
computes all next-values, and only then assigns. A model that let a later stage
see an earlier stage's new value in the same cycle would be able to do things
the hardware cannot.

What the envelope section below is defending against
----------------------------------------------------
The first version of this model smoothed |y| once per internal sample with an
8-bit envelope and a `env - (env >> 6)` release. That is broken in a way that
looks like a filter problem and is not:

  * `env >> 6` is 0 for every env below 64, so the minimum-step-of-1 rule that
    stops the release stalling at 1 turns it into a LINEAR 1-per-sample ramp
    across the entire useful range, and
  * how far that ramp drags the reading depends on how long the rectified
    waveform spends descending, which is set by the band's own frequency.

Measured: a full-scale 100 Hz tone drove band 0's |y| to 15658 of an expected
15625 -- the filter was perfect -- while its envelope read 13 of 61. Band 15,
whose rectified period is 1.7 samples instead of 120, read correctly. The
display would have shown a monotonic low-to-high tilt for any input whatsoever,
which is exactly what a spectrum analyzer must not do.

The fix is the two-stage detector below: a peak-hold over a 256-sample window
(one counter, one comparator per band), smoothed once per window rather than
once per sample. Window rate is 93.75 Hz, so the smoothing shifts operate on
windows and an 8-bit envelope is enough. 256 samples is more than one full
rectified period of the lowest band, so every band's true peak lands inside
some window by construction rather than by luck.

Run:

    python tools/sim_biquad_bank.py
"""

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gen_biquad_coeffs as gc  # noqa: E402

# --------------------------------------------------------------------------
# System constants. These mirror the hardware, not the design intent.
# --------------------------------------------------------------------------
VIDEO_CLK = 25_175_000      # audio_visualizer runs on video_clk
AUDIO_RATE = 48_000         # I_audio_valid pulses per second
INTERNAL_RATE = 24_000      # after decimation by two
FRAME_RATE = 60

CYCLES_PER_AUDIO = VIDEO_CLK / AUDIO_RATE          # 524.5
CYCLES_PER_INTERNAL = VIDEO_CLK / INTERNAL_RATE    # 1049.0 -- the MAC budget
CYCLES_PER_FRAME = VIDEO_CLK / FRAME_RATE

NBAND = gc.NBAND
COEFF_W = 18
FRAC = COEFF_W - 2          # Q1.16
STATE_EXTRA = gc.STATE_EXTRA
STATE_BITS = gc.STATE_BITS
DATA_BITS = gc.DATA_BITS
ACC_BITS = gc.ACC_BITS

TONE_AMP = 2_000_000        # the -12.5 dBFS sustain the analyser actually sees;
                            # hdmi_audio_tone_i2s_64fs.v's AMP is 4x this, since
                            # its envelope sits ENV_SUSTAIN=2 steps below peak

# Envelope detector. WINDOW must exceed one rectified period of the lowest
# band: 24000/100/2 = 120 samples, so 256 gives better than 2x.
WINDOW = 256
WINDOW_RATE = INTERNAL_RATE / WINDOW               # 93.75 Hz
ATTACK_SHIFT = 1            # tau = 2 windows  = 21 ms
RELEASE_SHIFT = 4           # tau = 16 windows = 171 ms

MAC_STATES_PER_BAND = 5


def sat(v, bits):
    hi = (1 << (bits - 1)) - 1
    lo = -(1 << (bits - 1))
    return hi if v > hi else (lo if v < lo else v)


def band_frequencies():
    return gc.band_frequencies()


def make_coeffs(width=COEFF_W, f_lo=None, q=None):
    """Quantized coefficient table, straight from the generator.

    f_lo and q are overrides for the negative controls; None means the shipped
    design.
    """
    saved_lo, saved_q = gc.F_LO, gc.Q
    try:
        if f_lo is not None:
            gc.F_LO = f_lo
        if q is not None:
            gc.Q = q
        return [gc.quantize(gc.rbj_bandpass(f0), width)
                for f0 in gc.band_frequencies()]
    finally:
        gc.F_LO, gc.Q = saved_lo, saved_q


def mix16(left, right):
    """sat16(signext(L[23:8]) + signext(R[23:8])).

    L[23:8] is the top 16 bits of the 24-bit sample. Summing two channels needs
    one extra bit, so the result saturates rather than wraps -- full-scale audio
    clips gracefully at 2x instead of flipping sign.
    """
    return sat((left >> 8) + (right >> 8), DATA_BITS)


def smooth(env, target, atk=ATTACK_SHIFT, rel=RELEASE_SHIFT):
    """Two-sided tracker, called once per WINDOW, not once per sample.

    Both branches need the minimum step of 1. Without it the attack stalls when
    target is 1..3 above env, and the release stalls at any env below 2**rel --
    with rel=4 that pins every band at 15 forever once the audio stops, leaving
    a 7 px bar standing in silence. Because this runs at 93.75 Hz rather than
    per sample, the linear region the minimum step creates is a small tail of
    the display timescale instead of the whole of it.
    """
    if target > env:
        step = (target - env) >> atk
        return sat(env + (step or 1), 8)
    if target < env:
        step = (env >> rel) or 1
        return env - step if env > step else 0
    return env


class Bank(object):
    """The filter bank, envelope detectors, peak caps and level meters."""

    def __init__(self, coeffs, state_extra=STATE_EXTRA, env_shift=8,
                 window=WINDOW):
        self.c = coeffs
        self.state_extra = state_extra
        self.env_shift = env_shift
        self.window = window

        # Global x history, shared by all 16 bands: the numerator is
        # B0*(x[n] - x[n-2]) and every band sees the same x.
        self.x_n1 = 0
        self.x_n2 = 0

        self.s1 = [0] * NBAND       # per-band state, Q state_extra
        self.s2 = [0] * NBAND

        self.pk = [0] * NBAND       # windowed peak-hold
        self.env = [0] * NBAND      # smoothed, drives the bars
        self.peak = [0] * NBAND     # falling cap
        self.peak_vel = [1] * NBAND
        self.win_cnt = 0

        self.pk_l = 0               # L/R meter peak-holds
        self.pk_r = 0
        self.lvl_l = 0
        self.lvl_r = 0

        self.frame_cnt = 0
        self.mix_prev = 0
        self.decim = 0

    # -- one 48 kHz stereo pair --------------------------------------------
    def audio_valid(self, left, right):
        """Called once per I_audio_valid. Every second call runs the bank."""
        m = mix16(left, right)

        # Meters read the raw 24-bit channels: 8388607 >> 16 == 127, the same
        # 0..127 range the band envelopes use. They peak-hold here and smooth
        # on the same window tick as the bands, for the same reason.
        self.pk_l = max(self.pk_l, sat(abs(left) >> 16, 8))
        self.pk_r = max(self.pk_r, sat(abs(right) >> 16, 8))

        ran = False
        if self.decim:
            # 2-tap average of the two 48 kHz samples -> one 24 kHz sample.
            # Averaging cannot overflow, so this needs no saturator of its own.
            xin = (m + self.mix_prev) >> 1
            self.decim = 0
            ran = self.step_sample(xin)
        else:
            self.mix_prev = m
            self.decim = 1
        return ran

    # -- one internal sample, all 16 bands ---------------------------------
    def step_sample(self, xin):
        """Returns True on the cycle the envelope window closes."""
        xin = sat(xin, DATA_BITS)
        dx = sat(xin - self.x_n2, DATA_BITS + 1)
        dx8 = dx << self.state_extra
        win_tick = (self.win_cnt == self.window - 1)

        for k in range(NBAND):
            y, _ = self._one_band(k, dx8)
            target = abs(y) >> self.env_shift
            p = self.pk[k] if self.pk[k] > target else target
            if win_tick:
                # Envelope is updated inside the band loop, not in a separate
                # parallel pass. Serial costs one comparator instead of
                # sixteen, and the 80 cycles it spreads across is 3.2 us at
                # video_clk -- invisible at 60 Hz.
                self.env[k] = smooth(self.env[k], p)
                self.pk[k] = 0
            else:
                self.pk[k] = p

        if win_tick:
            self.lvl_l = smooth(self.lvl_l, self.pk_l)
            self.lvl_r = smooth(self.lvl_r, self.pk_r)
            self.pk_l = 0
            self.pk_r = 0

        self.x_n2 = self.x_n1
        self.x_n1 = xin
        self.win_cnt = (self.win_cnt + 1) % self.window
        return win_tick

    def _one_band(self, k, dx8):
        """y[n] = B0*dx + NA1*y[n-1] + NA2*y[n-2].

        The partial sums saturate one at a time, in the same order the FSM
        accumulates them, so this and cycle_replay agree by construction rather
        than by luck of the 44-bit headroom. Rounding is applied once, after
        the last term: rounding each term separately would inject three biases
        per sample instead of one and bring back the DC offset this design
        works to avoid.
        """
        c = self.c[k]
        acc = sat(c["b0"] * dx8, ACC_BITS)
        acc = sat(acc + sat(c["na1"] * self.s1[k], ACC_BITS), ACC_BITS)
        acc = sat(acc + sat(c["na2"] * self.s2[k], ACC_BITS), ACC_BITS)
        acc = sat(acc + (1 << (FRAC - 1)), ACC_BITS)
        s_new = sat(acc >> FRAC, STATE_BITS)
        y = sat(s_new >> self.state_extra, DATA_BITS)

        self.s2[k] = self.s1[k]
        self.s1[k] = s_new
        return y, s_new

    # -- one video frame ----------------------------------------------------
    def frame_start(self):
        """Peak caps fall with a velocity that ramps up, so a full-scale peak
        drops in about half a second instead of the 4.25 s a 1-per-frame step
        takes."""
        self.frame_cnt = (self.frame_cnt + 1) & 0xFF
        for k in range(NBAND):
            env, peak, vel = self.env[k], self.peak[k], self.peak_vel[k]
            if peak > env:
                # Guarded: peak 2 with velocity 4 must land on 0, not 254.
                self.peak[k] = peak - vel if peak > vel else 0
                if (self.frame_cnt & 0x3) == 0 and vel < 4:
                    self.peak_vel[k] = vel + 1
            else:
                self.peak[k] = env
                self.peak_vel[k] = 1

    # -- derived display values --------------------------------------------
    def bar_heights(self):
        """The bar area is exactly 64 px and env is 0..127, so >> 1 fills it
        with no multiplier anywhere in the path."""
        return [e >> 1 for e in self.env]

    def peak_rows(self):
        return [p >> 1 for p in self.peak]


# --------------------------------------------------------------------------
# Cycle-level replay of the MAC finite state machine.
# --------------------------------------------------------------------------
def cycle_replay(coeffs, xin, s1_in, s2_in, x_n2, state_extra=STATE_EXTRA):
    """Literal cycle-by-cycle walk of the MAC finite state machine.

    Five cycles per band, so 80 for the whole bank against a budget of 1049.
    The multiplier is a single registered instance shared by all 16 bands:
    operands are launched in one cycle and the product captured in the next,
    which is why the three products are sequenced rather than computed at once.
    """
    ST_LAUNCH0, ST_LAUNCH1, ST_LAUNCH2, ST_ROUND, ST_WRITE = range(5)

    s1 = list(s1_in)
    s2 = list(s2_in)
    dx8 = sat(xin - x_n2, DATA_BITS + 1) << state_extra

    k = 0
    state = ST_LAUNCH0
    acc = 0
    prod = 0
    ma, mb = 0, 0
    cycles = 0

    while True:
        cycles += 1
        if cycles > 4 * CYCLES_PER_INTERNAL:
            raise RuntimeError("MAC FSM did not terminate")

        # Snapshot. Every right-hand side below is a value this cycle started
        # with; nothing observes another stage's update in the same cycle.
        nstate, nk, nacc, nprod = state, k, acc, prod
        nma, nmb = ma, mb
        ns1, ns2 = list(s1), list(s2)

        if state == ST_LAUNCH0:
            nma, nmb = coeffs[k]["b0"], dx8
            nstate = ST_LAUNCH1
        elif state == ST_LAUNCH1:
            nprod = sat(nma * nmb, ACC_BITS)       # B0*dx
            nma, nmb = coeffs[k]["na1"], s1[k]
            nstate = ST_LAUNCH2
        elif state == ST_LAUNCH2:
            nacc = sat(nacc + nprod, ACC_BITS)     # folds in B0*dx
            nprod = sat(nma * nmb, ACC_BITS)       # NA1*y[n-1]
            nma, nmb = coeffs[k]["na2"], s2[k]
            nstate = ST_ROUND
        elif state == ST_ROUND:
            nacc = sat(nacc + nprod, ACC_BITS)     # folds in NA1*y[n-1]
            nprod = sat(nma * nmb, ACC_BITS)       # NA2*y[n-2]
            nstate = ST_WRITE
        else:  # ST_WRITE
            nacc = sat(nacc + nprod, ACC_BITS)     # folds in NA2*y[n-2]
            nacc = sat(nacc + (1 << (FRAC - 1)), ACC_BITS)
            s_new = sat(nacc >> FRAC, STATE_BITS)
            ns2[k] = s1[k]
            ns1[k] = s_new
            nacc, nprod = 0, 0
            nk = k + 1
            nstate = ST_LAUNCH0

        acc, prod, ma, mb = nacc, nprod, nma, nmb
        s1, s2, k, state = ns1, ns2, nk, nstate

        if k >= NBAND:
            break

    return s1, s2, cycles


# --------------------------------------------------------------------------
# Test signals, 24-bit signed.
# --------------------------------------------------------------------------
def sq(f, amp=TONE_AMP):
    n = 0

    def gen():
        nonlocal n
        v = amp if (math.sin(2 * math.pi * f * n / AUDIO_RATE) >= 0) else -amp
        n += 1
        return v, v
    return gen


def sine(f, amp=TONE_AMP):
    n = 0

    def gen():
        nonlocal n
        v = int(round(amp * math.sin(2 * math.pi * f * n / AUDIO_RATE)))
        n += 1
        return v, v
    return gen


def silence():
    return lambda: (0, 0)


def noise(amp=TONE_AMP, seed=12345):
    rng = random.Random(seed)

    def gen():
        return rng.randint(-amp, amp), rng.randint(-amp, amp)
    return gen


def dc(level):
    return lambda: (level, level)


def run(bank, gen, n48, frames=True):
    """Drive n48 stereo pairs, applying frame_start at its true 60 Hz rate."""
    cyc = 0.0
    next_frame = CYCLES_PER_FRAME
    internal = 0
    for _ in range(n48):
        left, right = gen()
        if bank.audio_valid(left, right):
            internal += 1
        cyc += CYCLES_PER_AUDIO
        while cyc >= next_frame:
            if frames:
                bank.frame_start()
            next_frame += CYCLES_PER_FRAME
    return internal


def drive(bank, f, seconds=0.5, kind=sine):
    run(bank, kind(f), int(seconds * AUDIO_RATE))


# --------------------------------------------------------------------------
# Results plumbing
# --------------------------------------------------------------------------
FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    CHECKS[0] += 1
    if not cond:
        FAILURES.append("%s %s" % (label, detail))
        print("    FAIL  %s %s" % (label, detail))
    return cond


def expect_fail(cond, label, detail=""):
    """A negative control: the perturbed design MUST fail this assertion.

    If it passes, the test is not measuring what it claims to measure, which is
    worse than no test at all.
    """
    CHECKS[0] += 1
    if cond:
        FAILURES.append("negative control did not bite: '%s' held when it must "
                        "not %s" % (label, detail))
        print("    FAIL  negative control did not bite: '%s' held when it must "
              "not %s" % (label, detail))
        return False
    print("    ok    control bites: '%s' is false, as required" % label)
    return True


def argmax_env(bank):
    return max(range(NBAND), key=lambda i: bank.env[i])


# ==========================================================================
# Test 0 -- the two model levels agree, and the FSM fits its budget.
# ==========================================================================
def test_model_consistency():
    print("\n[0] cycle-level FSM vs sample-level fast path")
    coeffs = make_coeffs()
    rng = random.Random(999)

    fast = Bank(coeffs)
    s1 = [0] * NBAND
    s2 = [0] * NBAND
    x_n1 = x_n2 = 0
    worst = 0

    for i in range(300):
        xin = rng.randint(-32768, 32767) if i % 7 else 0
        fast.step_sample(xin)
        s1, s2, cyc = cycle_replay(coeffs, xin, s1, s2, x_n2)
        worst = max(worst, cyc)
        x_n2, x_n1 = x_n1, xin

    check(s1 == fast.s1, "state s1 identical after 300 samples")
    check(s2 == fast.s2, "state s2 identical after 300 samples")
    print("    FSM cycles per internal sample: %d (budget %.0f, %.1f%% used)"
          % (worst, CYCLES_PER_INTERNAL, 100.0 * worst / CYCLES_PER_INTERNAL))
    check(worst == MAC_STATES_PER_BAND * NBAND,
          "FSM cycle count is exactly %d" % (MAC_STATES_PER_BAND * NBAND),
          "got %d" % worst)
    check(worst < CYCLES_PER_INTERNAL, "FSM fits the 24 kHz budget")

    # Cross-check against the generator's own impulse model. Two independent
    # implementations of the same fixed-point recursion agreeing is much
    # stronger evidence than either passing on its own. This is what caught the
    # generator's single-register x delay line.
    print("    cross-check vs gen_biquad_coeffs.impulse_response")
    for k in (0, 5, 11, 15):
        ref = gc.impulse_response(coeffs[k])
        b = Bank(coeffs)
        got = []
        for i, xv in enumerate([32767] + [0] * (len(ref) - 1)):
            b.step_sample(xv)
            got.append(sat(b.s1[k] >> STATE_EXTRA, DATA_BITS))
        check(got == ref, "band %d impulse trace matches generator" % k,
              "first divergence at %d" % next(
                  (i for i, (a, c) in enumerate(zip(got, ref)) if a != c), -1))


# ==========================================================================
# Test A -- selectivity.
# ==========================================================================
def test_selectivity():
    freqs = band_frequencies()
    print("\n[A] selectivity")
    coeffs = make_coeffs()

    # A tone exactly on a band center, so the assertion is sharp. 440 Hz from
    # the original plan sits between band 5 (412.1) and band 6 (547.1) and
    # would light both, making "which band won" a question about geometry
    # rather than about the filter.
    k = 5
    b = Bank(coeffs)
    drive(b, freqs[k], 1.0)
    peak = argmax_env(b)
    print("    sine %.1f Hz -> band %d (env %d), nominal band %d"
          % (freqs[k], peak, b.env[peak], k))
    check(peak == k, "sine at band %d center peaks in band %d" % (k, k),
          "got band %d" % peak)

    # Neighbour rejection. The threshold is set from what Q=4 actually measures
    # with margin, not from a dB figure chosen in advance: the measured value is
    # -6.9 dB, and Q=2 measures -2.6 dB, so -5 dB separates them cleanly.
    nb = max(b.env[k - 1], b.env[k + 1])
    reject = 20.0 * math.log10(max(nb, 1) / max(b.env[k], 1))
    THRESHOLD = -5.0
    print("    immediate-neighbour rejection at Q=4: %.1f dB (env %d vs %d)"
          % (reject, b.env[k], nb))
    check(reject < THRESHOLD, "neighbour at least %.0f dB down" % abs(THRESHOLD),
          "measured %.1f dB" % reject)

    # Far bands must be dark -- but only for a sine. A square wave legitimately
    # lights every band its harmonics land in, and asserting otherwise would be
    # testing the test's own ignorance of the input spectrum.
    #
    # The threshold is relative, not absolute: a second-order bandpass rolls off
    # at 12 dB per octave, so two octaves out the theoretical rejection is
    # 24 dB. Measured here as -23.5 dB, which is the filter being correct. An
    # absolute count threshold would just be a guess about the input level.
    far = [i for i in range(NBAND)
           if abs(math.log(freqs[i] / freqs[k])) > math.log(4)]
    far_db = [(i, b.env[i],
               round(20.0 * math.log10(max(b.env[i], 1) / max(b.env[k], 1)), 1))
              for i in far]
    print("    bands >2 octaves away, sine (band, env, dB rel peak): %s"
          % (far_db,))
    check(all(db <= -20.0 for _, _, db in far_db),
          "far bands at least 20 dB below the peak, per 12 dB/octave roll-off",
          "got %s" % [(i, db) for i, _, db in far_db if db > -20.0])

    # Harmonic structure of a square wave: amplitude falls as 1/n for odd n.
    print("    square wave %.1f Hz harmonic profile" % freqs[k])
    bs = Bank(coeffs)
    drive(bs, freqs[k], 1.0, kind=sq)
    print("      " + " ".join("%d:%d" % (i, bs.env[i]) for i in range(NBAND)))
    fund = bs.env[k]
    check(fund == max(bs.env), "square wave fundamental band is the maximum")

    def nearest(f):
        return min(range(NBAND), key=lambda i: abs(math.log(freqs[i] / f)))
    h3, h5 = nearest(freqs[k] * 3), nearest(freqs[k] * 5)
    print("      3f -> band %d env %d (expect ~%d); 5f -> band %d env %d "
          "(expect ~%d)" % (h3, bs.env[h3], fund // 3, h5, bs.env[h5], fund // 5))
    check(bs.env[h3] < fund, "3rd harmonic below fundamental")
    check(bs.env[h5] < bs.env[h3], "5th harmonic below 3rd")
    check(0.5 * fund / 3 <= bs.env[h3] <= 2.0 * fund / 3 + 2,
          "3rd harmonic within a factor of 2 of 1/3 fundamental",
          "got %d vs %d" % (bs.env[h3], fund // 3))

    # Negative control: Q=2 widens the -3 dB bandwidth to 0.74 octaves against
    # 0.409 octaves of spacing, so neighbours must fail the same threshold.
    print("    negative control: Q=2")
    b2 = Bank(make_coeffs(q=2.0))
    drive(b2, freqs[k], 1.0)
    nb2 = max(b2.env[k - 1], b2.env[k + 1])
    reject2 = 20.0 * math.log10(max(nb2, 1) / max(b2.env[k], 1))
    print("      Q=2 neighbour rejection: %.1f dB" % reject2)
    expect_fail(reject2 < THRESHOLD, "Q=2 passes the rejection threshold",
                "Q=2 measured %.1f dB, threshold %.1f dB" % (reject2, THRESHOLD))


# ==========================================================================
# Test B -- stability and return to floor.
#
# Driven by a tone, not by DC: a bandpass has zero DC gain, so feeding DC
# leaves the states at zero and the decay assertion passes vacuously. That was
# the first version of this test, and it reported "env under DC: 0" as though
# that were a stability result.
# ==========================================================================
def test_stability():
    freqs = band_frequencies()
    print("\n[B] energy must decay to the floor after the audio stops")
    coeffs = make_coeffs()

    b = Bank(coeffs)
    drive(b, freqs[5], 1.0)
    driven = max(b.env)
    run(b, silence(), int(1.0 * AUDIO_RATE))
    floor = max(b.env)
    print("    env while driven: %d, after 1.0 s of silence: %d"
          % (driven, floor))
    check(driven > 20, "the tone actually drives the bank (test not vacuous)",
          "driven env %d" % driven)
    check(floor == 0, "all envelopes reach 0 after silence", "worst %d" % floor)

    # A DC step injects a broadband transient through dx = x[n] - x[n-2]. This
    # is the honest way to use DC in a stability test: a bandpass has zero DC
    # gain, so only the edge excites it. The 48->24 kHz decimator is phase
    # aligned here (9600 samples is an even number of internal samples), so the
    # step reaches the bank as xin = [32767, 32767, ...] and dx = [32767,
    # 32767, 0, 0, ...] -- measured, not assumed.
    #
    # The kick has to be snapshotted at the moment it happens, because tau runs
    # from 306 samples at band 0 down to 8 at band 15: measuring after a long
    # run sees zero everywhere and the test passes without measuring anything.
    held = dc(8_000_000)
    b2 = Bank(coeffs)
    run(b2, silence(), 9600)
    run(b2, held, 4800)                 # step lands at this run's first sample
    kicked = max(b2.env)
    kick_levels = list(b2.env)          # snapshot: the run below erases this
    kick_bands = [(i, v) for i, v in enumerate(kick_levels) if v]
    run(b2, held, 48000)                # then hold DC for a full second
    settled = max(b2.env)
    print("    DC step kick: max env %d, bands affected %s"
          % (kicked, kick_bands))
    print("    held at DC for 1.0 s: max env %d" % settled)
    check(kicked > 0, "DC step injects a transient", "env %d" % kicked)
    check(settled == 0, "steady DC leaves every band at 0 (zero DC gain)",
          "env %d" % settled)
    # The kick is broadband, and its envelope falls monotonically with band
    # index. Not because B0 falls -- B0 rises 214 -> 7165 -- but because
    # B0/sin(w0) = 1/(2Q(1+alpha)) is the same 0.12 for every band, so the
    # resonator's ring-up exactly cancels B0's rise and peak|y| is flat
    # (measured 6822 at band 0 against 3647 at band 15). What actually orders
    # the envelopes is the pole radius: tau runs 306 samples at 100 Hz down to
    # 8 at 7 kHz, so the low bands are still ringing across several envelope
    # windows while the high bands are dead inside the first one.
    check(sum(1 for v in kick_levels if v) >= NBAND // 2,
          "the step transient reaches most of the bank, not one band",
          "%s" % kick_bands)
    check(all(hi <= lo for lo, hi in zip(kick_levels, kick_levels[1:])),
          "kick envelope is non-increasing with band index, per tau falling "
          "with f", "%s" % kick_bands)

    # Negative control: negate one band's NA2. That flips the sign of a2',
    # turning a complex pole pair into a real root outside the unit circle
    # (|z| ~ 2.39 for band 3), so the band must run away and saturate.
    print("    negative control: NA2 of band 3 negated")
    bad = [dict(c) for c in coeffs]
    bad[3]["na2"] = -bad[3]["na2"]
    bb = Bank(bad)
    drive(bb, freqs[5], 1.0)
    run(bb, silence(), int(1.0 * AUDIO_RATE))
    print("      band 3 |state| after silence: %d (24-bit limit %d); max env %d"
          % (max(abs(v) for v in bb.s1), (1 << (STATE_BITS - 1)) - 1,
             max(bb.env)))
    expect_fail(max(bb.env) == 0, "unstable bank still decays to 0",
                "max env %d" % max(bb.env))


# ==========================================================================
# Test B' -- silence floor. This is the quantization DC offset.
# ==========================================================================
def test_silence_floor():
    freqs = band_frequencies()
    print("\n[B'] bars must be at zero when nothing is playing")
    coeffs = make_coeffs()

    # Cold-start silence is trivially zero: dx is 0 from the first sample, so
    # the states never leave zero and no rounding bias is ever injected. The
    # case that matters -- and the one the original bug would have shown -- is
    # audio that plays and then stops, leaving its rounding residue behind.
    cold = Bank(coeffs)
    run(cold, silence(), int(1.0 * AUDIO_RATE))

    b = Bank(coeffs)
    drive(b, freqs[5], 1.0)
    run(b, silence(), int(3.0 * AUDIO_RATE))
    print("    cold start, 1 s silence  : worst env %d, worst bar %d px"
          % (max(cold.env), max(cold.bar_heights())))
    print("    1 s tone then 3 s silence: worst env %d, worst bar %d px"
          % (max(b.env), max(b.bar_heights())))
    check(max(b.env) == 0, "every envelope is exactly 0 after audio stops",
          "worst %d" % max(b.env))
    check(max(b.bar_heights()) == 0, "no bar is left standing after audio stops",
          "worst %d" % max(b.bar_heights()))

    # Negative control: drop the extra fractional bits from the state. The
    # rounding bias then lands directly in the 16-bit feedback node, where the
    # DC gain (fs/2*pi*f0)^2 amplifies it by 1458x at 100 Hz.
    print("    negative control: STATE_EXTRA=0")
    bb = Bank(coeffs, state_extra=0)
    drive(bb, freqs[5], 1.0)
    run(bb, silence(), int(3.0 * AUDIO_RATE))
    raised = [(i, bb.env[i], bb.bar_heights()[i])
              for i in range(NBAND) if bb.bar_heights()[i] > 0]
    print("      bars left standing with no audio: %s" % (raised[:8],))
    expect_fail(max(bb.env) == 0, "STATE_EXTRA=0 still returns to the floor",
                "worst env %d, %d bars raised" % (max(bb.env), len(raised)))


# ==========================================================================
# Test C -- the band index really is the center frequency.
# ==========================================================================
def test_center_frequency():
    freqs = band_frequencies()
    print("\n[C] each band responds most at its own nominal center")
    coeffs = make_coeffs()

    wrong = []
    envs = []
    for k in range(NBAND):
        b = Bank(coeffs)
        drive(b, freqs[k], 1.0)
        envs.append(b.env[k])
        peak = argmax_env(b)
        if peak != k:
            wrong.append((k, round(freqs[k], 1), peak, b.env[k], b.env[peak]))
    print("    self-band env: " + " ".join("%3d" % e for e in envs))
    if wrong:
        print("    mismatched (band, f, peaked_in, env_self, env_peak): %s"
              % (wrong,))
    check(not wrong, "all 16 bands peak at their nominal center")

    # Frequency-flatness of the envelope: every band must read about the same
    # for the same input level. This is the assertion that catches the
    # per-sample-smoothing defect described in the module docstring, which
    # showed up as band 0 reading 13 while band 15 read correctly.
    lo, hi = min(envs), max(envs)
    tilt = 20.0 * math.log10(max(hi, 1) / max(lo, 1))
    print("    envelope spread across 16 bands: %d..%d (%.1f dB)"
          % (lo, hi, tilt))
    check(tilt < 3.0, "envelope is flat to within 3 dB across the bank",
          "%.1f dB tilt, %d..%d" % (tilt, lo, hi))

    # Negative control: per-sample smoothing instead of windowed peak-hold.
    # Same filter, same input, broken detector.
    print("    negative control: per-sample smoothing, release env>>6")

    class PerSample(Bank):
        """The broken detector: smooth |y| every internal sample."""

        def step_sample(self, xin):
            xin = sat(xin, DATA_BITS)
            dx = sat(xin - self.x_n2, DATA_BITS + 1)
            dx8 = dx << self.state_extra
            for k in range(NBAND):
                y, _ = self._one_band(k, dx8)
                self.env[k] = smooth(self.env[k], abs(y) >> self.env_shift,
                                     atk=2, rel=6)
            self.x_n2 = self.x_n1
            self.x_n1 = xin
            return False

    envs_ps = []
    for k in range(NBAND):
        p = PerSample(coeffs)
        drive(p, freqs[k], 1.0)
        envs_ps.append(p.env[k])
    tilt_ps = 20.0 * math.log10(max(max(envs_ps), 1) / max(min(envs_ps), 1))
    print("      per-sample env: " + " ".join("%3d" % e for e in envs_ps))
    print("      tilt %.1f dB" % tilt_ps)
    expect_fail(tilt_ps < 3.0, "per-sample smoothing is also flat",
                "%.1f dB tilt, %d..%d" % (tilt_ps, min(envs_ps), max(envs_ps)))

    # Negative control: give band 0 band 2's coefficients. A 100 Hz tone must
    # then light band 2, proving the test measures the filter's actual center
    # rather than the loop index it happens to be iterating.
    print("    negative control: band 0 given band 2's coefficients")
    swapped = [dict(c) for c in coeffs]
    swapped[0] = dict(coeffs[2])
    bs = Bank(swapped)
    drive(bs, freqs[0], 1.0)
    peak = argmax_env(bs)
    print("      %.1f Hz now peaks in band %d (env %d); band 0 reads %d"
          % (freqs[0], peak, bs.env[peak], bs.env[0]))
    expect_fail(peak == 0, "swapped coefficients still peak in band 0",
                "peaked in band %d" % peak)


# ==========================================================================
# Test D -- no dead bands.
# ==========================================================================
def test_no_dead_bands():
    print("\n[D] white noise reaches every band")
    coeffs = make_coeffs()
    b = Bank(coeffs)
    run(b, noise(), int(2.0 * AUDIO_RATE))
    print("    env: " + " ".join("%3d" % e for e in b.env))
    dead = [i for i in range(NBAND) if b.env[i] < 2]
    check(not dead, "no band reads below 2 under white noise", "dead %s" % dead)
    lo, hi = min(b.env), max(b.env)
    print("    spread %d..%d. White noise is flat per Hz but the bands are "
          "log-spaced, so high bands see more power; a rising tilt is correct "
          "here, unlike in test C." % (lo, hi))

    print("    negative control: band 7 coefficients zeroed")
    bad = [dict(c) for c in coeffs]
    for key in ("b0", "na1", "na2"):
        bad[7][key] = 0
    bb = Bank(bad)
    run(bb, noise(), int(2.0 * AUDIO_RATE))
    print("      band 7 env %d" % bb.env[7])
    expect_fail(bb.env[7] >= 2, "zeroed band still responds",
                "env %d" % bb.env[7])


# ==========================================================================
# Test E -- convergence, and the real test tone's bar heights.
# ==========================================================================
def test_envelope_and_scaling():
    freqs = band_frequencies()
    print("\n[E] envelope converges exactly, and the real tone fills the bars")
    coeffs = make_coeffs()

    e = 0
    for _ in range(200):
        e = smooth(e, 100)
    check(e == 100, "attack converges exactly to target", "stalled at %d" % e)
    for _ in range(4000):
        e = smooth(e, 0)
    check(e == 0, "release reaches exactly 0, never sticks at 1",
          "stalled at %d" % e)

    print("    negative control: no minimum-step-of-1 rule")

    def smooth_nomin(env, target):
        if target > env:
            return sat(env + ((target - env) >> ATTACK_SHIFT), 8)
        if target < env:
            return env - (env >> RELEASE_SHIFT)
        return env
    e = 0
    for _ in range(4000):
        e = smooth_nomin(e, 100)
    expect_fail(e == 100, "un-guarded attack still converges",
                "stalled at %d" % e)
    e = 5
    for _ in range(20000):
        e = smooth_nomin(e, 0)
    expect_fail(e == 0, "un-guarded release still reaches 0",
                "stalled at %d" % e)

    # A steady square at band 5's centre, at the level the shipped tone's
    # sustain plateau sits at -- not the shipped tone itself, which is a ten
    # segment loop (eight ADSR notes, a detuned second voice, two geometric
    # sweeps) whose attack peaks 12 dB above this. Steady is the right stand-in
    # for the plateau and the wrong one for the loop, so the real segment table
    # is measured end to end in sim_tone_gen.py's [G] section. This is the
    # number that decides whether the panel looks alive or broken, so it is
    # measured rather than estimated.
    print("    bar heights at the tone's sustain level (412.1 Hz square)")
    b = Bank(coeffs)
    drive(b, freqs[5], 1.0, kind=sq)
    heights = b.bar_heights()
    print("      env    : " + " ".join("%3d" % e for e in b.env))
    print("      bar px : " + " ".join("%3d" % h for h in heights))
    print("      L/R meter: %d / %d" % (b.lvl_l, b.lvl_r))
    check(heights[5] >= 20, "driven band reaches at least 20 of 63 px",
          "got %d px" % heights[5])
    check(heights[5] <= 63, "driven band does not overflow the bar area",
          "got %d px" % heights[5])

    # The driven level is -12.5 dBFS, so TONE_AMP >> 16 == 30 of 127 is the
    # right reading. Deriving the bound from the amplitude keeps the assertion
    # about the meter rather than about a number picked to make it pass.
    expect_lvl = TONE_AMP >> 16
    print("      expected meter reading for a %.1f dBFS tone: %d of 127"
          % (20.0 * math.log10(TONE_AMP / 8388607.0), expect_lvl))
    check(abs(b.lvl_l - expect_lvl) <= max(2, expect_lvl // 10),
          "L meter reads the tone's actual level",
          "got %d, expected ~%d" % (b.lvl_l, expect_lvl))
    check(b.lvl_l == b.lvl_r, "identical channels give identical meters",
          "%d vs %d" % (b.lvl_l, b.lvl_r))

    # Peak caps. While a tone holds steady, peak == env by definition and the
    # cap cannot fall, so the measurement has to be made after the audio stops:
    # the cap should lag the envelope on the way down and reach zero in about
    # half a second, not the 4.25 s a fixed 1-per-frame step would take.
    print("    peak cap fall time after the tone stops")
    for _ in range(10):
        b.frame_start()
    cap_top = max(b.peak)
    frames_to_zero = None
    for f in range(1, 181):
        run(b, silence(), int(AUDIO_RATE / FRAME_RATE), frames=False)
        b.frame_start()
        if max(b.peak) == 0 and frames_to_zero is None:
            frames_to_zero = f
    print("      cap at stop: %d, reached 0 after %s frames (%.2f s)"
          % (cap_top, frames_to_zero,
             (frames_to_zero or 999) / float(FRAME_RATE)))
    check(frames_to_zero is not None, "peak caps do reach 0 after audio stops")
    check(frames_to_zero is not None and frames_to_zero <= 90,
          "caps fall within 1.5 s, not the 4.25 s of a 1-per-frame step",
          "took %s frames" % frames_to_zero)
    check(max(b.env) == 0, "envelopes are also at 0 by then",
          "worst env %d" % max(b.env))

    # Window length must exceed the lowest band's rectified period, or band 0's
    # peak can straddle a window boundary and read low.
    lowest_period = INTERNAL_RATE / freqs[0] / 2.0
    print("    window %d samples vs lowest band rectified period %.0f samples"
          % (WINDOW, lowest_period))
    check(WINDOW > lowest_period, "window is longer than one rectified period",
          "%d vs %.0f" % (WINDOW, lowest_period))


def main():
    print("=" * 72)
    print("audio_visualizer filter-bank model")
    print("=" * 72)
    print("coefficients : Q1.%d, %d-bit signed, imported from "
          "gen_biquad_coeffs.py" % (FRAC, COEFF_W))
    print("state        : %d-bit signed, Q%d (%d extra fractional bits)"
          % (STATE_BITS, STATE_EXTRA, STATE_EXTRA))
    print("accumulator  : %d-bit" % ACC_BITS)
    print("rates        : %d Hz audio in, %d Hz internal, %.1f video_clk per "
          "internal sample" % (AUDIO_RATE, INTERNAL_RATE, CYCLES_PER_INTERNAL))
    print("envelope     : %d-sample peak-hold window (%.2f Hz), attack tau "
          "%.0f ms, release tau %.0f ms"
          % (WINDOW, WINDOW_RATE,
             1000.0 * (1 << ATTACK_SHIFT) / WINDOW_RATE,
             1000.0 * (1 << RELEASE_SHIFT) / WINDOW_RATE))
    print("bands        : " + ", ".join("%.0f" % f for f in band_frequencies()))

    test_model_consistency()
    test_selectivity()
    test_stability()
    test_silence_floor()
    test_center_frequency()
    test_no_dead_bands()
    test_envelope_and_scaling()

    print("\n" + "=" * 72)
    if FAILURES:
        print("%d of %d checks FAILED" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - " + f)
        return 1
    print("all %d checks passed" % CHECKS[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
