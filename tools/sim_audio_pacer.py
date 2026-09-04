#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model of audio_pcm_player.v (the 48 kHz video_clk pacer).

No Verilog simulator on this machine, so this mirrors the RTL register for
register and asserts the two properties the HDMI audio path lives or dies on:

  1. The fractional accumulator turns a 25 MHz video_clk into exactly 48000
     sample ticks per second (the same scheme hdmi_audio_tone_pcm_scale.v uses).
  2. O_audio_valid pulses on EVERY tick -- including when the FIFO is empty --
     because audio_arc_calculate derives one CTS every 48 valids and the HDMI
     core's audio clock regeneration needs an unbroken 48 kHz reference. An
     underrun must produce silence (zero samples), never a gap in valid.

It also checks the show-ahead FIFO read alignment: a tick with data pops exactly
one word and emits {dout[15:0],8'b0} / {dout[31:16],8'b0}, so frames come out in
order with the correct 16->24-bit left justification.

CLK_FREQ_HZ and SAMPLE_RATE_HZ are parsed out of the .v file, not restated.

Run from anywhere:  python tools/sim_audio_pacer.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source",
                   "audio_pcm_player.v")

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
    """Negative control: passes when cond is False."""
    CHECKS[0] += 1
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s" % label)
    print("    FAIL  control did NOT bite: %s%s"
          % (label, (" -- " + detail) if detail else ""))
    return False


def _parse_int(expr):
    expr = expr.strip().rstrip(",").strip().replace("_", "")
    m = re.match(r"(\d+)\s*'\s*[dD]\s*(\d+)$", expr)
    if m:
        return int(m.group(2))
    return int(expr, 0)


def parse_rtl(path=RTL):
    with open(path, "r", encoding="utf-8") as fh:
        code = re.sub(r"//[^\n]*", "", fh.read())
    cfg = {}
    for name, expr in re.findall(
            r"parameter\s+(?:integer\s+)?(?:\[\d+:\d+\]\s*)?(\w+)\s*=\s*"
            r"([^,\)\n]+)", code):
        try:
            cfg[name] = _parse_int(expr)
        except ValueError:
            continue
    for req in ("CLK_FREQ_HZ", "SAMPLE_RATE_HZ"):
        if req not in cfg:
            raise SystemExit("sim_audio_pacer: could not parse %r out of %s; "
                             "the model is stale, fix the parser" % (req, path))
    return cfg


# --------------------------------------------------------------------------
# The pacer. One step() == one video_clk cycle, mirroring the RTL always block
# with non-blocking semantics.
# --------------------------------------------------------------------------
class Pacer(object):
    def __init__(self, cfg, gate_valid_on_data=False):
        self.CLK = cfg["CLK_FREQ_HZ"]
        self.SR = cfg["SAMPLE_RATE_HZ"]
        # gate_valid_on_data models the BUG this design avoids: dropping valid
        # whenever the FIFO is empty, which would gap the ACR reference.
        self.gate_valid_on_data = gate_valid_on_data
        self.reset()

    def reset(self):
        self.acc = 0
        self.fifo_re = 0
        self.O_audio_valid = 0
        self.O_left = 0
        self.O_right = 0

    def step(self, rdusedw, dout):
        acc_next = self.acc + self.SR
        tick = acc_next >= self.CLK

        re_n = 0
        valid_n = 0
        left_n = self.O_left      # data regs hold between ticks (RTL never
        right_n = self.O_right    # reassigns them off-tick)

        if tick:
            acc_n = acc_next - self.CLK
            have = (rdusedw != 0)
            if have:
                re_n = 1
                left_n = ((dout & 0xFFFF) << 8) & 0xFFFFFF
                right_n = (((dout >> 16) & 0xFFFF) << 8) & 0xFFFFFF
            else:
                left_n = 0
                right_n = 0
            # The whole point: valid every tick. The buggy variant gates it.
            valid_n = 1 if (have or not self.gate_valid_on_data) else 0
        else:
            acc_n = acc_next

        self.acc = acc_n & 0xFFFFFFFF
        self.fifo_re = re_n
        self.O_audio_valid = valid_n
        self.O_left = left_n
        self.O_right = right_n


def run(pacer, queue, cycles, feed_at=None, feed_words=None):
    """Couple the pacer to a show-ahead FIFO modelled as a list.

    Cycle accounting mirrors the RTL pipeline: step() commits the registered
    outputs for the NEXT cycle, so O_audio_valid is recorded AFTER step (the
    value the consumer sees on that clock). A fifo_re asserted for cycle c pops
    the head at the end of cycle c, so the pop is applied at the start of the
    iteration that reads cycle c's head -- one iteration before the next tick
    samples it, exactly as the show-ahead FIFO prefetches.

    Returns (valid_count, samples, pops).
    """
    valid_count = 0
    samples = []
    pops = 0
    for c in range(cycles):
        # cycle-c FIFO head / occupancy, before this cycle's pop
        dout = queue[0] if queue else 0
        rdusedw = len(queue)
        # fifo_re committed for cycle c pops the head, effective for cycle c+1
        if pacer.fifo_re and queue:
            queue.pop(0)
            pops += 1
        # optional deferred fill (models the streamer catching up)
        if feed_at is not None and c == feed_at and feed_words:
            queue.extend(feed_words)
        pacer.step(rdusedw, dout)
        # record the outputs just committed for cycle c+1
        if pacer.O_audio_valid:
            valid_count += 1
            samples.append((pacer.O_left, pacer.O_right))
    return valid_count, samples, pops


def words_from_frames(frames):
    """FIFO word == {R[15:0], L[15:0]}, matching sd_audio_stream's fifo_di."""
    return [(((r & 0xFFFF) << 16) | (l & 0xFFFF)) for (l, r) in frames]


def test_tick_rate(cfg):
    print("\n[1] fractional accumulator produces exactly 48 kHz from 25 MHz")
    clk, sr = cfg["CLK_FREQ_HZ"], cfg["SAMPLE_RATE_HZ"]
    # A window that divides evenly: clk/gcd cycles -> sr/gcd ticks. Use 1/100 s.
    cycles = clk // 100                 # 250000
    expect_ticks = sr // 100            # 480
    pac = Pacer(cfg)
    valid_count, samples, _ = run(pac, [], cycles)   # empty FIFO -> all silence
    check(valid_count == expect_ticks,
          "%d clocks yield exactly %d valid pulses (48 kHz)"
          % (cycles, expect_ticks),
          "got %d" % valid_count)
    check(all(l == 0 and r == 0 for (l, r) in samples),
          "underrun outputs silence (left=right=0) on every tick")
    check(pac.acc == 0,
          "accumulator returns to 0 after an exactly-divisible window "
          "(no long-term drift)")


def test_data_path(cfg):
    print("\n[2] tick-with-data pops one word and left-justifies 16->24 bit")
    frames = [((0x1000 + i) & 0x7FFF, (0x2000 + i) & 0x7FFF) for i in range(50)]
    queue = words_from_frames(frames)
    # left  = {dout[15:0],8'b0}  = (w & 0xFFFF) << 8
    # right = {dout[31:16],8'b0} = ((w>>16)&0xFFFF) << 8
    expected = [(((w & 0xFFFF) << 8), (((w >> 16) & 0xFFFF) << 8)) for w in queue]
    pac = Pacer(cfg)
    # Enough cycles for ~50 ticks at 520 clocks each, with margin. The window
    # has one or two extra ticks after the queue drains, which emit silence and
    # pop nothing -- that is correct, so pops is checked against the word count.
    valid_count, samples, pops = run(pac, queue, 50 * 521 + 1000)
    n = len(frames)
    check(len(samples) >= n,
          "at least %d sample ticks in the window (got %d)" % (n, len(samples)))
    check(samples[:n] == expected[:n],
          "first %d emitted samples equal {L<<8, R<<8} of the FIFO words, in "
          "order (channels not swapped, alignment correct)" % n)
    check(pops == n,
          "each of the %d FIFO words is popped exactly once" % n,
          "got %d pops" % pops)


def test_underrun_continuity(cfg):
    print("\n[3] valid never gaps across an underrun, then resumes on data")
    cycles = cfg["CLK_FREQ_HZ"] // 100          # 250000 -> 480 ticks
    frames = [((0x100 + i) & 0x7FFF, (0x200 + i) & 0x7FFF) for i in range(200)]
    queue = []
    pac = Pacer(cfg)
    # Feed mid-window: first half starved, second half has data.
    valid_count, samples, pops = run(pac, queue, cycles,
                                     feed_at=cycles // 2,
                                     feed_words=words_from_frames(frames))
    expect_ticks = cfg["SAMPLE_RATE_HZ"] // 100
    check(valid_count == expect_ticks,
          "valid count is the full %d whether or not data is present -- the ACR "
          "reference is continuous through the underrun" % expect_ticks,
          "got %d" % valid_count)
    starved = samples[:len(samples) // 2]
    check(all(l == 0 and r == 0 for (l, r) in starved),
          "the starved first half is silent")
    fed = [s for s in samples if s != (0, 0)]
    check(len(fed) > 0,
          "after the feed the pacer emits real (non-zero) samples again")


def test_control_gated_valid(cfg):
    print("\n[4] NEGATIVE CONTROL: gating valid on data would gap the ACR ref")
    cycles = cfg["CLK_FREQ_HZ"] // 100
    expect_ticks = cfg["SAMPLE_RATE_HZ"] // 100
    buggy = Pacer(cfg, gate_valid_on_data=True)
    valid_count, _, _ = run(buggy, [], cycles)   # empty FIFO the whole time
    check(valid_count == 0,
          "the buggy variant emits ZERO valid pulses when starved",
          "got %d" % valid_count)
    expect_fail(valid_count == expect_ticks,
                "gated valid does NOT match the %d-pulse continuous reference "
                "(this is exactly the bug the real pacer avoids)" % expect_ticks)


def main():
    print("=" * 72)
    print("audio_pcm_player.v cycle-accurate model")
    print("=" * 72)
    cfg = parse_rtl()
    print("parsed from RTL: CLK_FREQ_HZ=%d  SAMPLE_RATE_HZ=%d"
          % (cfg["CLK_FREQ_HZ"], cfg["SAMPLE_RATE_HZ"]))

    test_tick_rate(cfg)
    test_data_path(cfg)
    test_underrun_continuity(cfg)
    test_control_gated_valid(cfg)

    print("\n" + "=" * 72)
    if FAILURES:
        print("FAILED %d of %d checks:" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("ALL %d CHECKS PASSED" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
