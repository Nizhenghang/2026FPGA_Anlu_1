#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model of sd_audio_stream.v (the TF-card WAV streamer).

No Verilog simulator on this machine, so this mirrors the RTL register for
register and asserts the properties the audio path depends on. It is a functional
model of the streamer's own logic -- header skip, 4-byte stereo-frame assembly,
sector-boundary backpressure, and the single-track loop -- driven by a minimal
model of the SD sector reader as sd_card_sec_read_write.v presents it:

    sd_sec_read_data_valid = (reader state == S_READ) && block_read_valid
    sd_sec_read_end        = (reader state == S_READ_END)

so `end` is a separate one-cycle state AFTER the 512 data bytes and never
overlaps the last data_valid. The real reader delivers a byte every few clocks
(25 MHz SPI against a 100 MHz sd_card_clk); this model delivers one byte per
clock, which is fine because the streamer only acts on data_valid cycles, so the
frame stream is identical at any byte rate.

HDR_LEN and PAUSE_THRESH are parsed out of the .v file rather than restated here,
so the model cannot silently drift from the RTL.

Run from anywhere:  python tools/sim_audio_stream.py
"""
import os
import re
import struct
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source",
                   "SD", "sd_audio_stream.v")

SECTOR = 512
FIFO_DEPTH = 512          # wfifo_32_32_512
FRAMES_PER_SECTOR = SECTOR // 4

S_IDLE, S_READ, S_WAIT, S_FAULT = 0, 1, 2, 3

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


# --------------------------------------------------------------------------
# Parse the RTL parameters.
# --------------------------------------------------------------------------
def _parse_int(expr):
    expr = expr.strip().rstrip(",").strip()
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
    for req in ("HDR_LEN", "PAUSE_THRESH"):
        if req not in cfg:
            raise SystemExit("sim_audio_stream: could not parse %r out of %s; "
                             "the model is stale, fix the parser" % (req, path))
    return cfg


# --------------------------------------------------------------------------
# The streamer. One step() == one sd_card_clk cycle. Mirrors the single always
# block in sd_audio_stream.v with non-blocking semantics: every next value is
# computed from the CURRENT registers, then committed together.
# --------------------------------------------------------------------------
class AudioStream(object):
    def __init__(self, cfg, wav_start_sector, wav_size):
        self.HDR_LEN = cfg["HDR_LEN"]
        self.PAUSE_THRESH = cfg["PAUSE_THRESH"]
        self.wav_start_sector = wav_start_sector & 0xFFFFFFFF
        self.wav_size = wav_size & 0xFFFFFFFF
        self.reset()

    def reset(self):
        self.state = S_IDLE
        self.pcm_total = 0
        self.pcm_cnt = 0
        self.hdr_skip = 0
        self.hdr_cnt = 0
        self.byte_phase = 0
        self.magic_done = 0
        self.riff = [0, 0, 0, 0]
        self.wave = [0, 0, 0, 0]
        self.l_lo = self.l_hi = self.r_lo = 0
        # registered outputs
        self.sd_sec_read = 0
        self.sd_sec_read_addr = 0
        self.fifo_we = 0
        self.fifo_di = 0

    # combinational wires
    def magic_ok(self):
        return (self.riff == [0x52, 0x49, 0x46, 0x46] and    # R I F F
                self.wave == [0x57, 0x41, 0x56, 0x45])       # W A V E

    def wav_usable(self):
        return self.wav_size > (self.HDR_LEN + 4)

    def pause_now(self, wrusedw):
        return wrusedw >= self.PAUSE_THRESH

    def step(self, start, data, data_valid, end, wrusedw):
        data &= 0xFF
        # next-value locals, seeded from current state (non-blocking defaults)
        state_n = self.state
        read_n = self.sd_sec_read
        addr_n = self.sd_sec_read_addr
        pcm_total_n = self.pcm_total
        pcm_cnt_n = self.pcm_cnt
        hdr_skip_n = self.hdr_skip
        hdr_cnt_n = self.hdr_cnt
        phase_n = self.byte_phase
        magic_done_n = self.magic_done
        riff_n = list(self.riff)
        wave_n = list(self.wave)
        l_lo_n, l_hi_n, r_lo_n = self.l_lo, self.l_hi, self.r_lo
        we_n = 0                      # fifo_we <= 1'b0 default every cycle
        di_n = self.fifo_di

        if self.state == S_IDLE:
            read_n = 0
            if start:
                if not self.wav_usable():
                    state_n = S_FAULT
                else:
                    pcm_total_n = (self.wav_size - self.HDR_LEN) & 0xFFFFFFFF
                    pcm_cnt_n = 0
                    hdr_skip_n = self.HDR_LEN
                    hdr_cnt_n = 0
                    phase_n = 0
                    addr_n = self.wav_start_sector
                    state_n = S_READ

        elif self.state == S_READ:
            read_n = 1
            if data_valid:
                if self.hdr_skip != 0:
                    if self.hdr_cnt == 0:
                        riff_n[0] = data
                    elif self.hdr_cnt == 1:
                        riff_n[1] = data
                    elif self.hdr_cnt == 2:
                        riff_n[2] = data
                    elif self.hdr_cnt == 3:
                        riff_n[3] = data
                    elif self.hdr_cnt == 8:
                        wave_n[0] = data
                    elif self.hdr_cnt == 9:
                        wave_n[1] = data
                    elif self.hdr_cnt == 10:
                        wave_n[2] = data
                    elif self.hdr_cnt == 11:
                        wave_n[3] = data
                    hdr_cnt_n = self.hdr_cnt + 1
                    hdr_skip_n = self.hdr_skip - 1
                    # magic is checked on the LAST header byte, against the
                    # already-committed riff/wave registers.
                    if self.hdr_skip == 1 and not self.magic_done:
                        magic_done_n = 1
                        if not self.magic_ok():
                            read_n = 0
                            state_n = S_FAULT
                elif self.pcm_cnt < self.pcm_total:
                    if self.byte_phase == 0:
                        l_lo_n = data
                        phase_n = 1
                    elif self.byte_phase == 1:
                        l_hi_n = data
                        phase_n = 2
                    elif self.byte_phase == 2:
                        r_lo_n = data
                        phase_n = 3
                    else:
                        di_n = ((data << 24) | (self.r_lo << 16) |
                                (self.l_hi << 8) | self.l_lo) & 0xFFFFFFFF
                        we_n = 1
                        phase_n = 0
                        pcm_cnt_n = (self.pcm_cnt + 4) & 0xFFFFFFFF
            if end:
                read_n = 0
                if self.pcm_cnt >= self.pcm_total:
                    addr_n = self.wav_start_sector      # single-track rewind
                    pcm_cnt_n = 0
                    hdr_skip_n = self.HDR_LEN
                    hdr_cnt_n = 0
                    phase_n = 0
                    state_n = S_READ
                elif self.pause_now(wrusedw):
                    addr_n = (self.sd_sec_read_addr + 1) & 0xFFFFFFFF
                    state_n = S_WAIT
                else:
                    addr_n = (self.sd_sec_read_addr + 1) & 0xFFFFFFFF
                    state_n = S_READ

        elif self.state == S_WAIT:
            read_n = 0
            if not self.pause_now(wrusedw):
                state_n = S_READ

        elif self.state == S_FAULT:
            read_n = 0

        else:
            read_n = 0
            state_n = S_IDLE

        # commit
        self.state = state_n
        self.sd_sec_read = read_n
        self.sd_sec_read_addr = addr_n & 0xFFFFFFFF
        self.pcm_total = pcm_total_n
        self.pcm_cnt = pcm_cnt_n
        self.hdr_skip = hdr_skip_n
        self.hdr_cnt = hdr_cnt_n
        self.byte_phase = phase_n
        self.magic_done = magic_done_n
        self.riff = riff_n
        self.wave = wave_n
        self.l_lo, self.l_hi, self.r_lo = l_lo_n, l_hi_n, r_lo_n
        self.fifo_we = we_n
        self.fifo_di = di_n


# --------------------------------------------------------------------------
# The SD sector reader, as the streamer observes it.
# --------------------------------------------------------------------------
class SdReader(object):
    WAIT, DELIVER, END = 0, 1, 2

    def __init__(self, card):
        self.card = card            # sector(int) -> 512 bytes
        self.state = self.WAIT
        self.addr = 0
        self.idx = 0
        self.sectors_read = 0

    def outputs(self):
        """Combinational (data, data_valid, end) from the current state."""
        if self.state == self.DELIVER:
            sec = self.card.get(self.addr, b"\x00" * SECTOR)
            return (sec[self.idx], 1, 0)
        if self.state == self.END:
            return (0, 0, 1)
        return (0, 0, 0)

    def step(self, sd_sec_read, sd_sec_read_addr):
        if self.state == self.WAIT:
            if sd_sec_read:
                self.addr = sd_sec_read_addr & 0xFFFFFFFF
                self.idx = 0
                self.state = self.DELIVER
                self.sectors_read += 1
        elif self.state == self.DELIVER:
            if self.idx == SECTOR - 1:
                self.state = self.END
            else:
                self.idx += 1
        elif self.state == self.END:
            self.state = self.WAIT


# --------------------------------------------------------------------------
# Test fixtures.
# --------------------------------------------------------------------------
def canonical_header(pcm_len, magic_riff=b"RIFF", magic_wave=b"WAVE"):
    return struct.pack("<4sI4s4sIHHIIHH4sI",
                       magic_riff, (36 + pcm_len) & 0xFFFFFFFF, magic_wave,
                       b"fmt ", 16, 1, 2, 48000, 192000, 4, 16,
                       b"data", pcm_len & 0xFFFFFFFF)


def make_frames(n):
    """Distinct L/R per frame so a channel swap or byte slip is visible."""
    return [((0x1000 + i) & 0x7FFF, (0x2000 + i) & 0x7FFF) for i in range(n)]


def frames_to_pcm(frames):
    return b"".join(struct.pack("<hh", l, r) for (l, r) in frames)


def expected_words(frames):
    """fifo_di == {R[15:0], L[15:0]}."""
    return [(((r & 0xFFFF) << 16) | (l & 0xFFFF)) for (l, r) in frames]


def build_wav(frames, riff=b"RIFF", wave=b"WAVE", inject=None):
    pcm = frames_to_pcm(frames)
    if inject is not None:
        # offset, byte -> splice one byte into the PCM region to misalign frames
        off, val = inject
        pcm = pcm[:off] + bytes([val]) + pcm[off:]
    hdr = canonical_header(len(frames) * 4, riff, wave)
    return hdr + pcm


def build_card(wav_bytes, start_sector):
    card = {}
    n_sec = (len(wav_bytes) + SECTOR - 1) // SECTOR
    for s in range(n_sec):
        chunk = wav_bytes[s * SECTOR:(s + 1) * SECTOR]
        if len(chunk) < SECTOR:
            chunk += b"\x00" * (SECTOR - len(chunk))
        card[start_sector + s] = chunk
    return card


def run(stream, reader, cycles, start=1, wrusedw_fn=lambda c: 0, collect=True):
    """Couple the two for `cycles` clocks. Returns list of fifo_di on fifo_we."""
    words = []
    for c in range(cycles):
        data, valid, end = reader.outputs()
        if collect and stream.fifo_we:
            words.append(stream.fifo_di)
        reader.step(stream.sd_sec_read, stream.sd_sec_read_addr)
        stream.step(start, data, valid, end, wrusedw_fn(c))
    return words


# --------------------------------------------------------------------------
# Tests.
# --------------------------------------------------------------------------
def test_framing_and_loop(cfg):
    print("\n[1] header skip, 4-byte frame assembly, and single-track loop")
    start_sector = 0x1000
    frames = make_frames(300)               # 1200 PCM bytes -> 3 sectors
    wav = build_wav(frames)
    card = build_card(wav, start_sector)
    st = AudioStream(cfg, start_sector, len(wav))
    rd = SdReader(card)

    exp = expected_words(frames)
    # ~3 sectors/pass, ~514 clocks/sector; 6000 clocks covers two full passes.
    words = run(st, rd, 6000)

    check(len(words) >= 2 * len(frames),
          "streamer emitted at least two loop passes (%d words)" % len(words),
          "got %d, wanted >= %d" % (len(words), 2 * len(frames)))
    check(words[:len(frames)] == exp,
          "pass 1 frames match {R,L} assembly exactly (header skipped, "
          "little-endian, channels in order)")
    check(words[len(frames):2 * len(frames)] == exp,
          "pass 2 (after rewind) replays the identical frame stream -> loop works")
    check(st.pcm_total == len(frames) * 4,
          "pcm_total == wav_size - HDR_LEN (%d)" % st.pcm_total)


def test_backpressure(cfg):
    print("\n[2] sector-boundary backpressure parks in S_WAIT and resumes")
    start_sector = 0x2000
    frames = make_frames(400)               # 1600 PCM bytes -> 4 sectors
    wav = build_wav(frames)
    card = build_card(wav, start_sector)
    st = AudioStream(cfg, start_sector, len(wav))
    rd = SdReader(card)

    # Run flat-out until the streamer is mid-stream (some frames written).
    run(st, rd, 700)
    check(st.pcm_cnt > 0 and st.state in (S_READ, S_WAIT),
          "streamer is actively streaming before backpressure (pcm_cnt=%d)"
          % st.pcm_cnt)

    # Now stall the read side: occupancy pinned at/over the threshold.
    sectors_before = rd.sectors_read
    stalled = run(st, rd, 2000, wrusedw_fn=lambda c: cfg["PAUSE_THRESH"])
    check(st.state == S_WAIT,
          "with wrusedw >= PAUSE_THRESH the streamer parks in S_WAIT",
          "state=%d" % st.state)
    check(rd.sectors_read == sectors_before,
          "no new sector is started while parked (sectors_read frozen at %d)"
          % rd.sectors_read)
    check(st.sd_sec_read == 0,
          "sd_sec_read is deasserted while parked")

    # Release: occupancy drops, streamer must resume and read the next sector.
    run(st, rd, 50, wrusedw_fn=lambda c: 0)
    check(st.state in (S_READ, S_WAIT) and rd.sectors_read > sectors_before,
          "after backpressure clears the streamer resumes reading",
          "state=%d sectors_read=%d (was %d)"
          % (st.state, rd.sectors_read, sectors_before))


def test_occupancy_bound(cfg):
    print("\n[3] static overflow bound: PAUSE_THRESH + one sector <= FIFO depth")
    bound = cfg["PAUSE_THRESH"] + FRAMES_PER_SECTOR
    check(bound <= FIFO_DEPTH,
          "PAUSE_THRESH(%d) + frames/sector(%d) = %d <= depth(%d), so a sector "
          "in flight can never overflow the FIFO"
          % (cfg["PAUSE_THRESH"], FRAMES_PER_SECTOR, bound, FIFO_DEPTH))


def test_control_misalign(cfg):
    print("\n[4] NEGATIVE CONTROL: one injected byte misaligns every frame")
    start_sector = 0x3000
    frames = make_frames(200)
    # Splice a stray byte at PCM offset 0: the streamer still skips exactly 44
    # header bytes, so real L_lo lands one byte late and every {R,L} word slips.
    wav_bad = build_wav(frames, inject=(0, 0xAA))
    card = build_card(wav_bad, start_sector)
    st = AudioStream(cfg, start_sector, len(wav_bad))
    rd = SdReader(card)
    words = run(st, rd, 4000)
    exp = expected_words(frames)
    expect_fail(words[:len(frames)] == exp,
                "misaligned stream does NOT reproduce the correct frames "
                "(proves the framing check in [1] has teeth)")


def test_control_bad_magic(cfg):
    print("\n[5] NEGATIVE CONTROL: wrong RIFF/WAVE magic -> silent S_FAULT")
    start_sector = 0x4000
    frames = make_frames(100)
    wav = build_wav(frames, riff=b"XIFF", wave=b"WAVE")
    card = build_card(wav, start_sector)
    st = AudioStream(cfg, start_sector, len(wav))
    rd = SdReader(card)
    words = run(st, rd, 3000)
    check(st.state == S_FAULT,
          "bad magic drives the streamer to the terminal silent S_FAULT",
          "state=%d" % st.state)
    expect_fail(len(words) > 0,
                "a faulted header emits zero FIFO words (no garbage audio)")


def test_control_unusable_size(cfg):
    print("\n[6] NEGATIVE CONTROL: wav_size <= HDR_LEN+4 -> S_FAULT, no spin")
    st = AudioStream(cfg, 0x5000, cfg["HDR_LEN"])   # size == 44, not usable
    rd = SdReader({})
    words = run(st, rd, 100)
    check(st.state == S_FAULT,
          "a too-small/absent file faults immediately instead of spinning the "
          "SD bus", "state=%d" % st.state)
    expect_fail(len(words) > 0, "faulted size emits zero FIFO words")


def main():
    print("=" * 72)
    print("sd_audio_stream.v cycle-accurate model")
    print("=" * 72)
    cfg = parse_rtl()
    print("parsed from RTL: HDR_LEN=%d  PAUSE_THRESH=%d"
          % (cfg["HDR_LEN"], cfg["PAUSE_THRESH"]))

    test_framing_and_loop(cfg)
    test_backpressure(cfg)
    test_occupancy_bound(cfg)
    test_control_misalign(cfg)
    test_control_bad_magic(cfg)
    test_control_unusable_size(cfg)

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
