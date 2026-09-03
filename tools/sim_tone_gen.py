"""Cycle-accurate model of hdmi_audio_tone_i2s_64fs.v's sample generation.

No Verilog simulator on this machine, so this mirrors the RTL and asserts the
properties the analyser depends on. The I2S bit timing is deliberately NOT
modelled: it is unchanged from the version that already works on hardware, and
`git diff` on the .v file is a stronger proof than a re-derivation would be.
What is modelled is the sample value, which is where all the new arithmetic
lives -- sweep endpoints, ADSR boundaries, the detuned second voice, the mix.

Constants are parsed out of the .v file rather than restated here. A model that
re-declares SWEEP_SHIFT_UP = 14 and then verifies that 14 is right proves
nothing; it would agree with the RTL even if both were wrong, and would keep
agreeing after one of them was edited. Parsing means the two cannot drift, and
the checks below then compare the parsed values against arithmetic derived from
first principles (inc(f) = f * 2**32 / 48000, segment lengths in seconds, the
analyser's band spacing).

The model also exposes a generator in the (left, right) form sim_biquad_bank.run
expects, which is what makes the end-to-end test at the bottom possible: the
real tone driving the real filter bank, asserting the bars actually move.

Run from anywhere:  python tools/sim_tone_gen.py
"""
import math
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

import sim_biquad_bank as sb            # noqa: E402

RTL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source",
                   "hdmi_audio_tone_i2s_64fs.v")

M32 = (1 << 32) - 1
FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    CHECKS[0] += 1
    if cond:
        print("    ok    %s" % label)
        return True
    FAILURES.append("%s%s" % (label, (" -- " + detail) if detail else ""))
    print("    FAIL  %s%s" % (label, (" -- " + detail) if detail else ""))
    return False


def expect_fail(cond, label, detail=""):
    """A negative control: passes when cond is False."""
    CHECKS[0] += 1
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s" % label)
    print("    FAIL  control did NOT bite: %s%s"
          % (label, (" -- " + detail) if detail else ""))
    return False


# --------------------------------------------------------------------------
# Parse the RTL.
# --------------------------------------------------------------------------
PARAM_RE = re.compile(
    r"^[ \t]*(?:parameter|localparam)[ \t]+(.*?)\b(\w+)[ \t]*=[ \t]*"
    r"([^,\n]+?)[ \t]*[,;]?[ \t]*$",
    re.MULTILINE)
LUT_RE = re.compile(
    r"(?:(\d)'d(\d)\s*|default\s*):\s*note_inc_lut\s*=\s*32'd(\d+)\s*;"
    r"\s*//\s*(\S+)\s+([A-G]\d)\s+([\d.]+)Hz")
DETUNE_RE = re.compile(r"W_inc2\s*=\s*W_inc\s*\+\s*\(\s*W_inc\s*>>\s*(\d+)\s*\)")

# 24'sd2000000 -> 2000000. The size is discarded, not kept: substituting it
# back in would turn 24'sd2000000 into 242000000.
SIZED_RE = re.compile(r"\b\d+'\s*[sS]?\s*[dD](\d+)")


def parse_rtl(path=RTL):
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    # Comments have to go before the parameter match, or a trailing "// 20480"
    # becomes part of the expression. The note LUT is parsed from the original
    # text, because its pitches live in the comments.
    code = re.sub(r"//[^\n]*", "", text)

    cfg = {}
    for _junk, name, expr in PARAM_RE.findall(code):
        py = SIZED_RE.sub(r"\1", expr.strip())
        try:
            cfg[name] = int(eval(py, {"__builtins__": {}}, dict(cfg)))
        except Exception:
            continue

    notes = []
    for _sz, idx, inc, _solfege, _name, hz in LUT_RE.findall(text):
        # The eighth entry is the `default:` arm, which a 3-bit input makes
        # index 7.
        notes.append((int(idx) if idx else 7, int(inc), float(hz)))
    notes.sort()
    if [n[0] for n in notes] != list(range(8)):
        raise SystemExit("sim_tone_gen: note LUT indices are %s, expected 0..7"
                         % [n[0] for n in notes])

    dm = DETUNE_RE.search(code)
    cfg["DETUNE_SHIFT"] = int(dm.group(1)) if dm else None

    for req in ("AMP", "NOTE_HOLD_FRAMES", "NOTE_LEN", "SEG_LAST",
                "SEG_SWEEP_UP", "SEG_LAST_NOTE", "SWEEP_INC_LO",
                "SWEEP_SHIFT_UP", "SWEEP_SHIFT_DOWN", "SWEEP_UP_LEN",
                "SWEEP_DOWN_LEN", "ENV_ATTACK_END", "ENV_DECAY_END",
                "ENV_RELEASE_BEG", "ENV_TAIL_BEG", "ENV_SUSTAIN", "ENV_FLOOR",
                "ENV_FADE_LEN", "DETUNE_SHIFT"):
        if cfg.get(req) is None:
            raise SystemExit("sim_tone_gen: could not parse %r out of %s\n"
                             "the model is stale; fix the parser, do not "
                             "hardcode the value" % (req, os.path.basename(path)))
    return cfg, [n[1] for n in notes], notes


def inc_of_freq(f):
    return f * (1 << 32) / 48000.0


def freq_of_inc(i):
    return i * 48000.0 / (1 << 32)


def sweep_up(inc0, shift, n, mask=True):
    """The RTL recurrence inc += inc >> shift, iterated in integers.

    n is the segment length in samples, so n-1 increments: the RTL skips the
    update on the frame that crosses the segment boundary.

    mask=True reproduces the 32-bit register. mask=False is for the range
    check, where a wrapped value would look legal and pass.
    """
    i = inc0
    for _ in range(n - 1):
        i = i + (i >> shift)
        if mask:
            i &= M32
    return i


def sweep_down(inc0, shift, n):
    i = inc0
    for _ in range(n - 1):
        i = (i - (i >> shift)) & M32
    return i


# --------------------------------------------------------------------------
# The model. One step() == one 48 kHz stereo frame.
# --------------------------------------------------------------------------
class Tone(object):
    def __init__(self, cfg, note_inc):
        self.c = cfg
        self.note_inc = note_inc
        self.phase = 0
        self.phase2 = 0
        self.sweep_inc = cfg["SWEEP_INC_LO"]
        # The RTL carries a two-deep pipeline: shift_reg is loaded from
        # S_sample_next read BEFORE this frame's update, and S_sample_word --
        # which reloads shift_reg mid-frame for the other channel -- is assigned
        # from that same pre-update value. Both halves of a frame therefore
        # carry the sample computed one frame earlier, which is what makes the
        # "right channel reuses the same sample" comment true.
        self.sample_next = 0
        self.sample_word = 0
        self.seg = 0
        self.cnt = 0
        self.left = 0
        self.right = 0

    # -- combinational, from the state as it stands this cycle --------------
    def is_sweep_up(self):
        return self.seg == self.c["SEG_SWEEP_UP"]

    def is_sweep_down(self):
        return self.seg == self.c["SEG_LAST"]

    def is_sweep(self):
        return self.is_sweep_up() or self.is_sweep_down()

    def seg_len(self):
        """W_seg_len. Three-way, matching the RTL: the two-way form this
        replaces handed the note segments the down-sweep's length."""
        if self.is_sweep_up():
            return self.c["SWEEP_UP_LEN"]
        if self.is_sweep_down():
            return self.c["SWEEP_DOWN_LEN"]
        return self.c["NOTE_LEN"]

    def cnt_rem(self):
        """W_cnt_rem: samples left after this one."""
        return self.seg_len() - 1 - self.cnt

    def inc(self):
        if self.is_sweep():
            return self.sweep_inc
        return self.note_inc[self.seg & 7]

    def env_shift(self):
        c = self.cnt
        # Only the outer ends of the sweep pair fade. Across the turnaround the
        # RTL leaves S_sweep_inc alone and both sides sit at ENV_SUSTAIN, so
        # neither the pitch nor the level steps there.
        if self.is_sweep_up():
            if c < self.c["ENV_FADE_LEN"]:
                return self.c["ENV_FLOOR"] - ((c >> 8) & 7)
            return self.c["ENV_SUSTAIN"]
        if self.is_sweep_down():
            if self.cnt_rem() < self.c["ENV_FADE_LEN"]:
                return self.c["ENV_FLOOR"] - ((self.cnt_rem() >> 8) & 7)
            return self.c["ENV_SUSTAIN"]
        if c < self.c["ENV_ATTACK_END"]:
            return self.c["ENV_FLOOR"] - ((c >> 8) & 7)
        if c < self.c["ENV_DECAY_END"]:
            return (c >> 10) & 1
        if c < self.c["ENV_RELEASE_BEG"]:
            return self.c["ENV_SUSTAIN"]
        if c < self.c["ENV_TAIL_BEG"]:
            return self.c["ENV_SUSTAIN"] + \
                ((c - self.c["ENV_RELEASE_BEG"]) >> 9)
        return self.c["ENV_FLOOR"]

    def raw_voices(self):
        amp = self.c["AMP"]
        sq1 = amp if (self.phase >> 31) & 1 else -amp
        sq2 = amp if (self.phase2 >> 31) & 1 else -amp
        return sq1, sq2

    def sample(self):
        sq1, sq2 = self.raw_voices()
        mix = (sq1 >> 1) + (sq2 >> 1)
        return mix >> self.env_shift(), mix

    # -- sequential --------------------------------------------------------
    def step(self):
        """Advance one 48 kHz frame. Returns (left, right) as sent on I2S."""
        c = self.c
        inc = self.inc()
        inc2 = (inc + (inc >> c["DETUNE_SHIFT"])) & M32

        sample, mix = self.sample()

        # What actually goes out this frame: the pre-update pipeline value.
        out = self.sample_next
        self.left = out
        self.right = self.sample_word

        self.sample_word = self.sample_next
        self.sample_next = sample
        self.phase = (self.phase + inc) & M32
        self.phase2 = (self.phase2 + inc2) & M32

        if self.cnt == self.seg_len() - 1:
            self.cnt = 0
            leaving = self.seg
            self.seg = 0 if self.seg == c["SEG_LAST"] else self.seg + 1
            if leaving == c["SEG_LAST_NOTE"]:
                self.sweep_inc = c["SWEEP_INC_LO"]
        else:
            self.cnt += 1
            if self.is_sweep_up():
                self.sweep_inc = (self.sweep_inc +
                                  (self.sweep_inc >> c["SWEEP_SHIFT_UP"])) & M32
            elif self.is_sweep_down():
                self.sweep_inc = (self.sweep_inc -
                                  (self.sweep_inc >> c["SWEEP_SHIFT_DOWN"])) & M32

        return out, out, mix, sample


def run_frames(cfg, note_inc, n):
    """n frames; returns a list of per-frame records."""
    t = Tone(cfg, note_inc)
    recs = []
    for _ in range(n):
        seg, cnt = t.seg, t.cnt
        inc, eshift = t.inc(), t.env_shift()
        sweep = t.sweep_inc
        l, r, mix, sample = t.step()
        recs.append(dict(seg=seg, cnt=cnt, inc=inc, sweep=sweep, shift=eshift,
                         left=l, right=r, mix=mix, sample=sample,
                         phase=t.phase, phase2=t.phase2))
    return t, recs


def loop_len(cfg):
    return cfg["NOTE_LEN"] * 8 + cfg["SWEEP_UP_LEN"] + cfg["SWEEP_DOWN_LEN"]


# --------------------------------------------------------------------------
# Tests.
# --------------------------------------------------------------------------
def test_constants(cfg, notes):
    """The parsed RTL constants against arithmetic from first principles."""
    print("\n[A] parsed constants against first principles")

    # AMP is the pre-envelope amplitude, so it is NOT the level the analyser
    # sees. Plan section 4 calibrated the input scaling for a steady 2000000
    # sample (xin = +/-15625, env 61, a 30 px bar); the envelope's sustain sits
    # ENV_SUSTAIN steps below that, and the sustain is what the bars read for
    # the 341 ms of every 500 ms note.
    sus = cfg["AMP"] >> cfg["ENV_SUSTAIN"]
    check(sus == 2000000,
          "AMP at the envelope's sustain level is the -12.5 dBFS the analyser "
          "was calibrated for",
          "AMP %d >>> %d = %d" % (cfg["AMP"], cfg["ENV_SUSTAIN"], sus))
    check(cfg["AMP"] <= 8388607,
          "AMP itself fits the signed 24-bit range -- the attack peak plays at "
          "shift 0", "%d" % cfg["AMP"])
    check(cfg["NOTE_HOLD_FRAMES"] == 24000,
          "a note segment is 0.5 s at 48 kHz",
          "RTL has %d" % cfg["NOTE_HOLD_FRAMES"])
    check(cfg["NOTE_LEN"] == cfg["NOTE_HOLD_FRAMES"],
          "NOTE_LEN is NOTE_HOLD_FRAMES, not that value truncated by its 17-bit "
          "declaration", "%d vs %d" % (cfg["NOTE_LEN"], cfg["NOTE_HOLD_FRAMES"]))

    # The note LUT is pre-existing, but it has never been checked against its
    # own comments either.
    worst = 0.0
    for _idx, inc, hz in notes:
        err = abs(freq_of_inc(inc) - hz) / hz
        worst = max(worst, err)
    check(len(notes) == 8, "all eight note LUT entries parsed", "%d" % len(notes))
    check(worst < 1e-4, "every note LUT increment matches its commented pitch",
          "worst %.2e" % worst)

    bands = sb.band_frequencies()
    lo = freq_of_inc(cfg["SWEEP_INC_LO"])
    check(abs(lo - bands[0]) / bands[0] < 1e-3,
          "the sweep starts at the analyser's bottom band centre",
          "%.4f Hz vs %.4f Hz" % (lo, bands[0]))

    end_up = sweep_up(cfg["SWEEP_INC_LO"], cfg["SWEEP_SHIFT_UP"],
                      cfg["SWEEP_UP_LEN"])
    top = freq_of_inc(end_up)
    check(abs(top - bands[-1]) / bands[-1] < 2e-3,
          "the up-sweep lands on the analyser's top band centre, so band %d "
          "gets swept too" % (sb.NBAND - 1),
          "%.2f Hz vs %.2f Hz" % (top, bands[-1]))

    # The RTL does not touch S_sweep_inc on the frame that crosses the segment
    # boundary, so the down-sweep starts on exactly the up-sweep's last value.
    end_dn = sweep_down(end_up, cfg["SWEEP_SHIFT_DOWN"], cfg["SWEEP_DOWN_LEN"])
    check(abs(freq_of_inc(end_dn) - bands[0]) / bands[0] < 5e-3,
          "the down-sweep lands back on the bottom band centre",
          "%.2f Hz" % freq_of_inc(end_dn))

    # Unmasked, because the RTL's 32-bit register would wrap silently and the
    # wrapped value looks perfectly legal.
    wide = sweep_up(cfg["SWEEP_INC_LO"], cfg["SWEEP_SHIFT_UP"],
                    cfg["SWEEP_UP_LEN"], mask=False)
    check(wide + (wide >> cfg["DETUNE_SHIFT"]) <= M32,
          "the sweep increment stays inside 32 bits, detuned copy included",
          "%d" % wide)

    total = loop_len(cfg)
    check(5.0 < total / 48000.0 < 8.0,
          "one loop is a few seconds long", "%d frames = %.4f s"
          % (total, total / 48000.0))
    check(1.5 < cfg["SWEEP_UP_LEN"] / float(cfg["SWEEP_DOWN_LEN"]) < 2.5,
          "the up-sweep is about twice the down-sweep: slow enough to read on "
          "the way up, quick on the way back",
          "%.2fx" % (cfg["SWEEP_UP_LEN"] / float(cfg["SWEEP_DOWN_LEN"])))

    # ADSR must fit inside a note segment, in order, with room for the tail.
    a, d, rb, tb = (cfg["ENV_ATTACK_END"], cfg["ENV_DECAY_END"],
                    cfg["ENV_RELEASE_BEG"], cfg["ENV_TAIL_BEG"])
    check(0 < a < d < rb < tb < cfg["NOTE_HOLD_FRAMES"],
          "the ADSR boundaries are ordered and inside the segment",
          "%d %d %d %d / %d" % (a, d, rb, tb, cfg["NOTE_HOLD_FRAMES"]))
    check(cfg["ENV_FLOOR"] > cfg["ENV_SUSTAIN"] > 0,
          "the envelope floor is quieter than the sustain level")
    check(a % 256 == 0 and (d - a) % 1024 == 0 and (tb - rb) % 512 == 0,
          "every ADSR step length is a power of two, so the shift is a "
          "bit-select and not a division",
          "attack %d, decay %d, release %d" % (a, d - a, tb - rb))
    check((tb - rb) // 512 + cfg["ENV_SUSTAIN"] == cfg["ENV_FLOOR"],
          "the release ramp lands exactly on the floor, not short of it",
          "%d steps from %d" % ((tb - rb) // 512, cfg["ENV_SUSTAIN"]))

    fade = cfg["ENV_FADE_LEN"]
    check(fade % 256 == 0,
          "the sweep fade is a whole number of 256-sample 6 dB steps, so it is "
          "the same bit-select ramp the attack uses", "%d" % fade)
    check(fade // 256 == cfg["ENV_FLOOR"] - cfg["ENV_SUSTAIN"] + 1,
          "the fade runs ENV_FLOOR down to ENV_SUSTAIN inclusive, so it arrives "
          "on the plateau level instead of one 6 dB step above it",
          "%d steps for a %d -> %d ramp" % (fade // 256, cfg["ENV_FLOOR"],
                                            cfg["ENV_SUSTAIN"]))
    check(2 * fade < cfg["SWEEP_DOWN_LEN"],
          "both fades fit inside the shorter sweep without meeting",
          "2*%d vs %d" % (fade, cfg["SWEEP_DOWN_LEN"]))

    # Detune: +1/2**shift, expressed in cents and as a beat frequency.
    sh = cfg["DETUNE_SHIFT"]
    cents = 1200.0 * math.log2(1.0 + 2.0 ** -sh)
    check(12.0 < cents < 15.0, "the second voice is detuned ~13 cents",
          "%.2f cents at >>%d" % (cents, sh))
    beats = [f * (2.0 ** -sh) for f in (freq_of_inc(notes[0][1]),
                                        freq_of_inc(notes[-1][1]))]
    check(1.9 < min(beats) and max(beats) < 4.2,
          "the beat stays in 2..4 Hz across the note range",
          "%.2f..%.2f Hz" % (min(beats), max(beats)))


def test_schedule(cfg, note_inc):
    print("\n[B] segment schedule over one loop")
    n = loop_len(cfg)
    t, recs = run_frames(cfg, note_inc, n + 1)

    runs = []
    for r in recs[:n]:
        if runs and runs[-1][0] == r["seg"]:
            runs[-1][1] += 1
        else:
            runs.append([r["seg"], 1])
    check([s for s, _ in runs] == list(range(10)),
          "the loop walks segments 0..9 in order", "%s" % [s for s, _ in runs])
    lens = [ln for _, ln in runs]
    check(lens == [cfg["NOTE_HOLD_FRAMES"]] * 8 +
          [cfg["SWEEP_UP_LEN"], cfg["SWEEP_DOWN_LEN"]],
          "each segment lasts exactly its declared length", "%s" % lens)
    check(recs[n]["seg"] == 0 and recs[n]["cnt"] == 0,
          "the loop wraps back to segment 0")

    up = [r for r in recs[:n] if r["seg"] == cfg["SEG_SWEEP_UP"]]
    freqs = [freq_of_inc(r["sweep"]) for r in up]
    check(all(b > a for a, b in zip(freqs, freqs[1:])),
          "the up-sweep frequency is strictly increasing")
    dn = [r for r in recs[:n] if r["seg"] == cfg["SEG_LAST"]]
    dfreqs = [freq_of_inc(r["sweep"]) for r in dn]
    check(all(b < a for a, b in zip(dfreqs, dfreqs[1:])),
          "the down-sweep frequency is strictly decreasing")
    check(abs(freqs[0] - 100.0) < 0.1, "the up-sweep starts at 100 Hz",
          "%.3f" % freqs[0])
    # Exact, not a tolerance: the RTL leaves S_sweep_inc alone on the frame that
    # crosses the segment boundary, so the turnaround cannot be discontinuous by
    # construction. Asserting the equality keeps that construction honest.
    check(dn[0]["sweep"] == up[-1]["sweep"],
          "the turnaround is continuous: the down-sweep's first increment is "
          "the up-sweep's last",
          "%d -> %d (%.2f -> %.2f Hz)" % (up[-1]["sweep"], dn[0]["sweep"],
                                          freqs[-1], dfreqs[0]))

    # A note segment must hold one frequency for its whole length.
    seg3 = [r for r in recs[:n] if r["seg"] == 3]
    check(len(set(r["inc"] for r in seg3)) == 1,
          "a note segment holds a constant increment")
    check(abs(freq_of_inc(seg3[0]["inc"]) - 349.23) < 0.01,
          "segment 3 is F4", "%.2f Hz" % freq_of_inc(seg3[0]["inc"]))

    return recs


def test_envelope(cfg, note_inc, recs):
    print("\n[C] ADSR envelope")
    n = loop_len(cfg)
    seg0 = [r for r in recs[:n] if r["seg"] == 0]
    shifts = [r["shift"] for r in seg0]

    check(shifts[0] == cfg["ENV_FLOOR"], "a note starts at the -48 dB floor",
          "%d" % shifts[0])
    check(shifts[-1] == cfg["ENV_FLOOR"], "a note ends at the -48 dB floor",
          "%d" % shifts[-1])
    check(min(shifts) == 0, "the attack reaches full amplitude",
          "min %d" % min(shifts))
    check(sorted(set(shifts)) == list(range(9)),
          "all nine 6 dB steps are used", "%s" % sorted(set(shifts)))

    a = cfg["ENV_ATTACK_END"]
    check(all(y <= x for x, y in zip(shifts[:a], shifts[1:a])),
          "the attack ramp is monotone non-increasing")
    rb, tb = cfg["ENV_RELEASE_BEG"], cfg["ENV_TAIL_BEG"]
    check(all(y >= x for x, y in zip(shifts[rb:], shifts[rb + 1:])),
          "the release ramp is monotone non-decreasing")
    check(len(set(shifts[cfg["ENV_DECAY_END"]:rb])) == 1 and
          shifts[cfg["ENV_DECAY_END"]] == cfg["ENV_SUSTAIN"],
          "the sustain plateau is flat at -12 dB",
          "%s" % sorted(set(shifts[cfg["ENV_DECAY_END"]:rb])))

    fl = cfg["ENV_FADE_LEN"]
    up = [r["shift"] for r in recs[:n] if r["seg"] == cfg["SEG_SWEEP_UP"]]
    dn = [r["shift"] for r in recs[:n] if r["seg"] == cfg["SEG_LAST"]]

    check(up[0] == cfg["ENV_FLOOR"],
          "the up-sweep starts on the -48 dB floor", "%d" % up[0])
    check(all(y <= x for x, y in zip(up[:fl], up[1:fl])),
          "the up-sweep's fade-in is monotone")
    check(up[fl] == cfg["ENV_SUSTAIN"],
          "the up-sweep's fade-in arrives on the plateau level at the sample "
          "the plateau starts", "%d" % up[fl])
    check(len(set(up[fl:])) == 1,
          "the up-sweep holds ENV_SUSTAIN from there on, with no fade-out -- "
          "the turnaround is continuous in pitch so it needs none",
          "%s" % sorted(set(up[fl:])))

    check(dn[0] == up[-1] == cfg["ENV_SUSTAIN"],
          "the down-sweep starts at exactly the level the up-sweep ended on",
          "%d -> %d" % (up[-1], dn[0]))
    check(len(set(dn[:-fl])) == 1,
          "the down-sweep holds ENV_SUSTAIN until its fade-out",
          "%s" % sorted(set(dn[:-fl])))
    check(all(y >= x for x, y in zip(dn[-fl:], dn[-fl + 1:])),
          "the down-sweep's fade-out is monotone")
    check(dn[-1] == cfg["ENV_FLOOR"],
          "the down-sweep ends on the -48 dB floor", "%d" % dn[-1])


def test_amplitude(cfg, note_inc, recs):
    print("\n[D] amplitude bounds")
    n = loop_len(cfg)
    amp = cfg["AMP"]
    mixes = [r["mix"] for r in recs[:n]]
    check(max(abs(m) for m in mixes) == amp,
          "the two-voice mix peaks at exactly AMP, not 2*AMP",
          "max |mix| = %d, AMP = %d" % (max(abs(m) for m in mixes), amp))
    check(all(abs(m) <= amp for m in mixes), "no mix sample exceeds AMP")
    check(all(abs(r["sample"]) <= amp for r in recs[:n]),
          "no enveloped sample exceeds AMP")
    check(all(-8388608 <= r["sample"] <= 8388607 for r in recs[:n]),
          "every sample fits the signed 24-bit W_sample wire")
    check(all(r["left"] == r["right"] for r in recs[:n]),
          "both I2S halves carry the same sample")

    # The loudest sample in the loop must come from a note at shift 0, and the
    # envelope must actually have reduced something.
    loud = max(abs(r["sample"]) for r in recs[:n])
    quiet = min(abs(r["sample"]) for r in recs[:n] if r["shift"] == cfg["ENV_FLOOR"])
    check(loud == amp, "the peak sample reaches full AMP", "%d" % loud)
    check(quiet <= amp >> 7, "the floor is at least 42 dB below the peak",
          "%d" % quiet)


def test_no_click(cfg, note_inc, recs):
    """A discontinuity at a segment boundary is a click, and a click is
    broadband: it lights all 16 analyser bands at once.

    Two things can be discontinuous. Phase is covered by the accumulator never
    being cleared. Amplitude is covered by the envelope shift being *equal* on
    both sides of the boundary -- a stronger statement than "both sides are
    quiet", and the one that lets the up-sweep hand over to the down-sweep at
    ENV_SUSTAIN instead of forcing the top band through a fade.
    """
    print("\n[E] segment boundaries are click-free")
    n = loop_len(cfg)

    bounds = [i for i in range(1, n)
              if recs[i]["cnt"] == 0 and recs[i - 1]["cnt"] != 0]
    check(len(bounds) == 9, "nine segment boundaries in the loop",
          "%d" % len(bounds))

    shifts = [(recs[i - 1]["shift"], recs[i]["shift"]) for i in bounds]
    check(all(a == b for a, b in shifts),
          "the envelope shift is the same on both sides of every boundary, so "
          "the amplitude cannot step", "%s" % shifts)

    # Eight of the nine boundaries also change pitch. A pitch change with
    # continuous phase is not a click, but only if the amplitude at that instant
    # is at the floor; otherwise the new frequency's first half-cycle is a step
    # of 2*level. Only the turnaround may be loud, and only because its pitch is
    # continuous too.
    loud = [(i, a) for i, (a, _b) in zip(bounds, shifts)
            if a != cfg["ENV_FLOOR"]]
    check(len(loud) == 1, "exactly one boundary leaves the -48 dB floor",
          "%s" % loud)
    if len(loud) == 1:
        i = loud[0][0]
        check(recs[i - 1]["seg"] == cfg["SEG_SWEEP_UP"] and
              recs[i]["seg"] == cfg["SEG_LAST"],
              "the loud boundary is the sweep turnaround, not a note edge",
              "seg %d -> %d" % (recs[i - 1]["seg"], recs[i]["seg"]))
        check(recs[i - 1]["inc"] == recs[i]["inc"],
              "and the increment is identical across it, so the pitch does not "
              "step either", "%d vs %d" % (recs[i - 1]["inc"], recs[i]["inc"]))

    # And the phase accumulator must run straight through: a reset would put a
    # discontinuity in the waveform even at constant amplitude. recs[i] holds
    # the increment frame i used and the accumulator after it, so the
    # accumulator is exactly the running sum of the increments from zero.
    ph = [r["phase"] for r in recs[:n]]
    rebuilt = 0
    bad = 0
    for i in range(n):
        rebuilt = (rebuilt + recs[i]["inc"]) & M32
        if ph[i] != rebuilt:
            bad += 1
    check(bad == 0, "the phase accumulator is never cleared or reloaded",
          "%d frames disagree" % bad)


def sweep_dwell(recs, seg):
    """Samples of segment `seg` spent inside each analyser band.

    The band edges come from the analyser's own centre frequencies: a
    log-spaced set with ratio r has band k covering [f_k/sqrt(r), f_k*sqrt(r)),
    which tiles the axis without gaps or overlap.
    """
    bands = sb.band_frequencies()
    half = math.sqrt(bands[1] / bands[0])
    edges = [(b / half, b * half) for b in bands]
    dwell = [0] * sb.NBAND
    for r in recs:
        if r["seg"] != seg:
            continue
        f = freq_of_inc(r["sweep"])
        for k, (lo, hi) in enumerate(edges):
            if lo <= f < hi:
                dwell[k] += 1
                break
    return dwell


def test_sweep_dwell(cfg, note_inc, recs):
    """Why the sweep is geometric. Nothing else in this file measures it."""
    print("\n[F] sweep dwell per analyser band")
    # One envelope window, counted in 48 kHz frames: the analyser decimates by
    # two, so its WINDOW internal samples are 2*WINDOW frames of this tone.
    win = sb.WINDOW * 2
    print("      one envelope window is %d frames of the tone" % win)

    for seg, name in ((cfg["SEG_SWEEP_UP"], "up"), (cfg["SEG_LAST"], "down")):
        d = sweep_dwell(recs, seg)
        print("      %-5s per-band dwell in frames: %s" % (name, d))
        check(min(d) > 0,
              "the %s-sweep visits all %d bands" % (name, sb.NBAND),
              "zero in %s" % [k for k, v in enumerate(d) if v == 0])
        check(min(d) >= 2 * win,
              "the %s-sweep's shortest dwell spans two envelope windows, so "
              "even the bands it enters part-way have time to rise" % name,
              "min %d frames vs %d" % (min(d), 2 * win))
        # The first and last bands are entered part-way through, so they get
        # about half the dwell of the interior ones by geometry, not by defect.
        inner = d[1:-1]
        check(max(inner) <= min(inner) * 11 // 10,
              "the %s-sweep dwells uniformly across the interior bands -- the "
              "property a linear ramp cannot have" % name,
              "%d..%d, spread %.2fx" % (min(inner), max(inner),
                                        max(inner) / float(min(inner))))


def sweep_argmax(cfg, note_inc):
    """The whole loop through the real filter bank, sampled at the display's own
    60 Hz. Returns (frame, env_vector) pairs.

    The [G] trajectory checks and their [H] negative control both call this. The
    head trim in test_end_to_end is itself part of what is asserted there, so a
    control that re-implemented the trace would only be testing its own copy.
    """
    n = loop_len(cfg)
    t = Tone(cfg, note_inc)
    bank = sb.Bank(sb.make_coeffs())

    # The whole envelope vector is kept, not just the brightest band: recording
    # only the argmax would make "band k never lit" and "band k was never the
    # brightest" the same number, which is exactly the confusion [G] has to
    # avoid.
    per_frame = int(48000 / 60)
    trace = []
    max_lvl = 0
    for i in range(n):
        out, _out2, _mix, _sample = t.step()
        bank.audio_valid(out, out)
        max_lvl = max(max_lvl, bank.lvl_l, bank.lvl_r)
        if i % per_frame == 0:
            trace.append((i, list(bank.env)))
    return trace, max_lvl


def test_end_to_end(cfg, note_inc):
    """The real tone through the real filter bank. This is the check the whole
    change exists for: the spectrum has to move."""
    print("\n[G] end to end: the tone through the 16-band analyser")

    n = loop_len(cfg)
    trace, max_lvl = sweep_argmax(cfg, note_inc)

    up_start = cfg["NOTE_LEN"] * 8
    up_end = up_start + cfg["SWEEP_UP_LEN"]

    # Measured after the fade-in, not from up_start. For ENV_FADE_LEN samples
    # the sweep is deliberately quieter than the note it follows, so the
    # brightest band during that time is still C5's residue decaying at the
    # envelope's release rate. That is the analyser's tail, not the sweep's
    # trajectory, and including it put the first frames at band 4.8.
    body = up_start + cfg["ENV_FADE_LEN"]
    ups = [(i, max(range(sb.NBAND), key=lambda k: e[k]))
           for i, e in trace if body <= i < up_end]
    check(len(ups) > 50, "the up-sweep spans many video frames", "%d" % len(ups))

    first_band = sum(b for _, b in ups[:5]) / 5.0
    last_band = sum(b for _, b in ups[-5:]) / 5.0
    check(first_band <= 3, "the up-sweep starts in the bottom bands",
          "%.1f" % first_band)
    check(last_band >= sb.NBAND - 4,
          "the up-sweep ends in the top bands", "%.1f" % last_band)

    # Monotonicity belongs to the sweep's own trajectory, which does not start
    # at up_start. The sweep climbs from 100 Hz while C5's residue is still
    # decaying in band 6 -- release tau is 16 windows -- so argmax crosses
    # 6 -> 0 at the handover. That crossover is what the display shows and it is
    # not a regression; no note-to-sweep boundary can avoid it, because the
    # sweep restarts at the bottom. Trim the leading frames until the sweep owns
    # the display (argmax in the bottom four bands, all below any note's) and
    # require the prefix to be short, so "the sweep never takes over" still
    # fails rather than being trimmed away.
    head = next((j for j, (_i, b) in enumerate(ups) if b <= 3), len(ups))
    # A rail, not a controlled check, and it is worth saying so: four
    # single-constant perturbations were tried to push head past its threshold
    # (fade lengthened to 8192, release truncated so the note is loud at the
    # edge, fade removed, ramp slowed) and none got head above 2, chiefly
    # because `body` moves with ENV_FADE_LEN and so excludes the very fade that
    # would delay the takeover. What it guards against is the trim silently
    # swallowing the assertion below -- if the sweep never rises, head becomes
    # len(ups) and traj is empty. The assertion it protects is the one with a
    # control in [H] (SWEEP_SHIFT_UP=11, which regresses six times).
    check(head <= 4, "the sweep takes over the display within four video frames",
          "%d frames of note residue" % head)
    traj = ups[head:]
    backwards = sum(1 for (_, a), (_, b) in zip(traj, traj[1:]) if b < a - 1)
    check(backwards == 0,
          "the brightest band never jumps downwards during the up-sweep",
          "%d regressions" % backwards)

    band_max = [max(e[k] for _i, e in trace) for k in range(sb.NBAND)]
    lit = sum(1 for v in band_max if v > 8)
    check(lit == sb.NBAND,
          "every one of the 16 bars moves during the loop",
          "%d/16, per-band max env %s" % (lit, band_max))

    best = [0] * sb.NBAND
    for _i, e in trace:
        b = max(range(sb.NBAND), key=lambda k: e[k])
        best[b] = max(best[b], e[b])
    covered = sum(1 for v in best if v > 0)
    check(covered == sb.NBAND,
          "every bar is the brightest at some point in the loop",
          "%d/16, argmax peaks %s" % (covered, best))

    # Operating point. Section 4 of the plan calibrated the analyser's input
    # scaling against a constant -12.5 dBFS tone: xin = +/-15625, env 61, a 30 px
    # bar. Section 5 then gave that tone an ADSR whose sustain sits below the
    # peak, which silently moved the steady-state level. Measure it rather than
    # assume the old number still holds.
    def report(name, lo, hi):
        vals = sorted(max(e) for i, e in trace if lo <= i < hi)
        if not vals:
            return None
        mid = vals[len(vals) // 2]
        print("      %-14s env  min %3d  median %3d  max %3d   bar %2d px"
              % (name, vals[0], mid, vals[-1], mid >> 1))
        return mid

    # Segment 3, not segment 0. The bank starts cold, so frames 0..ATTACK_END
    # measure the startup transient: reporting that as "attack peak" once
    # printed a median of 0, which reads as a silent attack but was only an
    # empty filter bank.
    seg3 = cfg["NOTE_LEN"] * 3
    print("    operating point (bar area is 63 px):")
    sus = report("note sustain", seg3 + cfg["ENV_DECAY_END"],
                 seg3 + cfg["ENV_RELEASE_BEG"])
    report("note attack", seg3, seg3 + cfg["ENV_ATTACK_END"])
    swp = report("sweeps", up_start, up_end)
    report("whole loop", 0, n)
    print("      L/R meter      max lvl %3d of 127 -> %3d px of 508"
          % (max_lvl, max_lvl << 2))
    return sus, swp, max_lvl


# --------------------------------------------------------------------------
# Negative controls. Each perturbs one parsed constant and asserts that the
# check which is supposed to guard it actually fails.
# --------------------------------------------------------------------------
def controls(cfg, note_inc):
    print("\n[H] negative controls")
    base = dict(cfg)

    def fails_with(mut, probe):
        c = dict(base)
        c.update(mut)
        saved = list(FAILURES)
        del FAILURES[:]
        n0 = CHECKS[0]
        try:
            probe(c)
        except Exception:
            del FAILURES[:]
            FAILURES.extend(saved)
            CHECKS[0] = n0
            return True                      # a crash is a bite too
        hit = bool(FAILURES)
        del FAILURES[:]
        FAILURES.extend(saved)
        CHECKS[0] = n0
        return hit

    top_band = sb.band_frequencies()[-1]

    # Probes re-run only the assertion that the control is aimed at.
    def p_const(c):
        end = sweep_up(c["SWEEP_INC_LO"], c["SWEEP_SHIFT_UP"], c["SWEEP_UP_LEN"])
        if abs(freq_of_inc(end) - top_band) / top_band >= 2e-3:
            FAILURES.append("up-sweep endpoint")

    def p_sched(c):
        if not (1.5 < c["SWEEP_UP_LEN"] / float(c["SWEEP_DOWN_LEN"]) < 2.5):
            FAILURES.append("sweep length ratio")

    def p_dwell(c):
        # Only the up-sweep matters here; rebuild its increments directly rather
        # than running a whole 6 s loop for one number.
        recs = []
        i = c["SWEEP_INC_LO"]
        for _ in range(c["SWEEP_UP_LEN"]):
            recs.append(dict(seg=c["SEG_SWEEP_UP"], sweep=i))
            i = (i + (i >> c["SWEEP_SHIFT_UP"])) & M32
        if min(sweep_dwell(recs, c["SEG_SWEEP_UP"])) < 2 * sb.WINDOW * 2:
            FAILURES.append("dwell shorter than two envelope windows")

    def p_fade(c):
        fade = c["ENV_FADE_LEN"]
        if fade % 256 or fade // 256 != c["ENV_FLOOR"] - c["ENV_SUSTAIN"] + 1:
            FAILURES.append("fade does not land on the plateau")

    def p_env(c):
        a, d, rb, tb = (c["ENV_ATTACK_END"], c["ENV_DECAY_END"],
                        c["ENV_RELEASE_BEG"], c["ENV_TAIL_BEG"])
        if not (0 < a < d < rb < tb < c["NOTE_HOLD_FRAMES"]):
            FAILURES.append("ADSR order")
        if (tb - rb) // 512 + c["ENV_SUSTAIN"] != c["ENV_FLOOR"]:
            FAILURES.append("release does not land on the floor")

    def p_pow2(c):
        a, d, rb, tb = (c["ENV_ATTACK_END"], c["ENV_DECAY_END"],
                        c["ENV_RELEASE_BEG"], c["ENV_TAIL_BEG"])
        if not (a % 256 == 0 and (d - a) % 1024 == 0 and (tb - rb) % 512 == 0):
            FAILURES.append("step length is not a power of two")

    def p_detune(c):
        sh = c["DETUNE_SHIFT"]
        cents = 1200.0 * math.log2(1.0 + 2.0 ** -sh)
        if not (12.0 < cents < 15.0):
            FAILURES.append("detune")

    def p_amp(c):
        # Two voices summed without halving would peak at 2*AMP.
        if abs((c["AMP"] >> 1) + (c["AMP"] >> 1)) != c["AMP"]:
            FAILURES.append("mix")
        if c["AMP"] > 8388607:
            FAILURES.append("range")
        if c["AMP"] >> c["ENV_SUSTAIN"] != 2000000:
            FAILURES.append("sustain level")

    def p_click(c):
        n = loop_len(c)
        _t, recs = run_frames(c, note_inc, n)
        bounds = [i for i in range(1, n)
                  if recs[i]["cnt"] == 0 and recs[i - 1]["cnt"] != 0]
        shifts = [(recs[i - 1]["shift"], recs[i]["shift"]) for i in bounds]
        if any(a != b for a, b in shifts):
            FAILURES.append("envelope steps at a boundary")
        if len([a for a, _b in shifts if a != c["ENV_FLOOR"]]) != 1:
            FAILURES.append("wrong number of boundaries off the floor")

    def p_seglen(c):
        # Compare against NOTE_HOLD_FRAMES, not NOTE_LEN: loop_len derives from
        # NOTE_LEN, so perturbing NOTE_LEN would scale the expectation along
        # with the measurement and this would never bite.
        n = c["NOTE_LEN"] * 8 + c["SWEEP_UP_LEN"] + c["SWEEP_DOWN_LEN"]
        _t, recs = run_frames(c, note_inc, n)
        runs = []
        for r in recs:
            if runs and runs[-1][0] == r["seg"]:
                runs[-1][1] += 1
            else:
                runs.append([r["seg"], 1])
        want = ([c["NOTE_HOLD_FRAMES"]] * 8 +
                [c["SWEEP_UP_LEN"], c["SWEEP_DOWN_LEN"]])
        if [ln for _, ln in runs] != want:
            FAILURES.append("segment lengths")

    def p_overflow(c):
        # Unmasked: the RTL's register would wrap, and a wrapped increment looks
        # perfectly legal to every other check.
        top = sweep_up(c["SWEEP_INC_LO"], c["SWEEP_SHIFT_UP"],
                       c["SWEEP_UP_LEN"], mask=False)
        if top + (top >> c["DETUNE_SHIFT"]) > M32:
            FAILURES.append("increment overflow")

    def p_traj(c):
        # The only probe that runs the whole loop through the filter bank, so it
        # is also the only one that can show the [G] trajectory check has teeth.
        # It calls sweep_argmax rather than rebuilding the trace: the head trim
        # is part of what [G] asserts, and a control that re-implemented it would
        # be testing its own copy.
        trace, _lvl = sweep_argmax(c, note_inc)
        up_start = c["NOTE_LEN"] * 8
        up_end = up_start + c["SWEEP_UP_LEN"]
        ups = [(i, max(range(sb.NBAND), key=lambda k: e[k]))
               for i, e in trace
               if up_start + c["ENV_FADE_LEN"] <= i < up_end]
        head = next((j for j, (_i, b) in enumerate(ups) if b <= 3), len(ups))
        traj = ups[head:]
        if len(traj) <= 50:
            FAILURES.append("the trim swallowed the sweep")
        if any(b < a - 1 for (_i, a), (_j, b) in zip(traj, traj[1:])):
            FAILURES.append("trajectory regresses")

    CASES = [
        # The exact defect this model caught in the RTL: W_seg_len was a
        # two-way `is_sweep_up ? UP : DOWN`, so all eight notes inherited the
        # down-sweep's length. Invisible while the two were equal.
        ("W_seg_len collapsed back to a two-way select, so every note runs as "
         "long as the down-sweep",
         {"NOTE_LEN": cfg["SWEEP_DOWN_LEN"]}, p_seglen),
        # Same 100 -> 7000 Hz range, just faster, so the endpoint check still
        # passes and only the dwell check can see it. This is the linear ramp's
        # failure mode in miniature.
        ("up-sweep sped up to >>11 with its length cut to match, so the bars "
         "have under two envelope windows each",
         {"SWEEP_SHIFT_UP": 11, "SWEEP_UP_LEN": 8704}, p_dwell),
        ("up-sweep shift dropped to >>12 with the length left alone, so it "
         "overshoots the top band",
         {"SWEEP_SHIFT_UP": 12}, p_const),
        ("down-sweep stretched to match the up-sweep, losing the quick return",
         {"SWEEP_DOWN_LEN": cfg["SWEEP_UP_LEN"]}, p_sched),
        ("release moved after the tail, so the ramp never happens",
         {"ENV_RELEASE_BEG": cfg["ENV_TAIL_BEG"] + 100}, p_env),
        ("release truncated so the note is still loud at the boundary",
         {"ENV_RELEASE_BEG": cfg["NOTE_HOLD_FRAMES"] - 512,
          "ENV_TAIL_BEG": cfg["NOTE_HOLD_FRAMES"]}, p_click),
        ("the sweep fades removed, so both sweep boundaries click",
         {"ENV_FADE_LEN": 0}, p_click),
        ("the sweep fade cut one step short, so it arrives 6 dB above the "
         "plateau and jumps",
         {"ENV_FADE_LEN": 1536}, p_fade),
        ("ADSR step lengths made non-power-of-two (the plan's 40/60/80 ms)",
         {"ENV_ATTACK_END": 1920, "ENV_DECAY_END": 4800}, p_pow2),
        ("detune shifted to 1/2048, inaudible as a beat",
         {"DETUNE_SHIFT": 11}, p_detune),
        ("AMP put back to 2000000, in range but 12 dB under the analyser's "
         "calibration",
         {"AMP": 2000000}, p_amp),
        ("AMP raised past the 24-bit range",
         {"AMP": 9000000}, p_amp),
        ("up-sweep lengthened so the geometric increment wraps 32 bits",
         {"SWEEP_UP_LEN": 200000}, p_overflow),
        # The same overshoot the >>12 case catches at the endpoint, seen instead
        # at the display: the sweep runs off the top band and the brightest bar
        # walks back down. This is the control for the [G] trajectory check, and
        # the only one that pays for a whole loop through the filter bank.
        ("up-sweep sped up to >>11, so the brightest bar overshoots the top "
         "band and climbs back down",
         {"SWEEP_SHIFT_UP": 11}, p_traj),
    ]

    for label, mut, probe in CASES:
        expect_fail(fails_with(mut, probe) is False, label)


def main():
    print("=" * 72)
    print("hdmi_audio_tone_i2s_64fs.v sample-generation model")
    print("=" * 72)

    cfg, note_inc, notes = parse_rtl()
    print("parsed %d constants and %d note LUT entries from %s"
          % (len(cfg), len(notes), os.path.basename(RTL)))

    test_constants(cfg, notes)
    recs = test_schedule(cfg, note_inc)
    test_envelope(cfg, note_inc, recs)
    test_amplitude(cfg, note_inc, recs)
    test_no_click(cfg, note_inc, recs)
    test_sweep_dwell(cfg, note_inc, recs)
    test_end_to_end(cfg, note_inc)
    controls(cfg, note_inc)

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
