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

Modelled properties: header skip, 4-byte stereo-frame assembly, sector-boundary
backpressure, the single-track loop, and the MUSC 0 withdraw (start falling while
a sector is in flight).

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
                if not start:
                    # Withdrawn by the audio source select (MUSC 0). Retiring
                    # here rather than the cycle start goes low hands the shared
                    # SD port back at the only point the arbiter in sd_card_bmp
                    # considers safe, and leaves no granted sector half
                    # ingested. Costs at most one sector of latency, which is
                    # inaudible because the top level has already muxed over to
                    # the test tone. The address and pcm_cnt are deliberately
                    # NOT rewound here: S_IDLE re-initialises both on the next
                    # start, so a re-arm restarts the track from the top.
                    state_n = S_IDLE
                elif self.pcm_cnt >= self.pcm_total:
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
            # Same withdraw as S_READ's sector boundary; S_WAIT is already
            # parked with the request low, so this one is immediate.
            if not start:
                state_n = S_IDLE
            elif not self.pause_now(wrusedw):
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


def step_once(st, rd, start, words):
    """One coupled sd_card_clk with per-cycle visibility.

    Returns (grant_live, end_pulse, request), all sampled at the START of the
    cycle, because that is what the arbiter in sd_card_bmp sees when it decides
    whether the port is still audio's. grant_live means the reader is mid
    transaction on a sector it has already been granted, i.e. arb_aud_own is
    still set and the port cannot change hands this cycle.

    wrusedw is pinned to 0: the withdraw tests exercise no backpressure.
    """
    grant_live = rd.state in (SdReader.DELIVER, SdReader.END)
    request = st.sd_sec_read
    data, valid, end = rd.outputs()
    if words is not None and st.fifo_we:
        words.append(st.fifo_di)
    rd.step(st.sd_sec_read, st.sd_sec_read_addr)
    st.step(start, data, valid, end, 0)
    return grant_live, end, request


def step_immediate_withdraw(st, rd, start, words):
    """MUTANT of step_once: retire the cycle start goes low instead of waiting
    for the sector boundary. Same signature and same sampled outputs, so a test
    can drive both through identical stimulus.
    """
    grant_live = rd.state in (SdReader.DELIVER, SdReader.END)
    request = st.sd_sec_read
    data, valid, end = rd.outputs()
    if start == 0 and st.state == S_READ:
        st.state = S_IDLE
        st.sd_sec_read = 0
        st.fifo_we = 0
        rd.step(0, st.sd_sec_read_addr)
    else:
        if words is not None and st.fifo_we:
            words.append(st.fifo_di)
        rd.step(st.sd_sec_read, st.sd_sec_read_addr)
        st.step(start, data, valid, end, 0)
    return grant_live, end, request


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


def test_withdraw_and_rearm(cfg):
    print("\n[7] MUSC 0 withdraw: retire at the sector boundary, replay from the top")
    start_sector = 0x6000
    frames = make_frames(400)               # 1600 PCM bytes -> 4 sectors
    wav = build_wav(frames)
    card = build_card(wav, start_sector)
    st = AudioStream(cfg, start_sector, len(wav))
    rd = SdReader(card)
    exp = expected_words(frames)

    # --- phase A: stream flat-out until genuinely mid-sector, past the header.
    run(st, rd, 700)
    check(st.state == S_READ and st.magic_done == 1 and st.pcm_cnt > 0,
          "streaming and past the header before the withdraw "
          "(state=%d pcm_cnt=%d magic_done=%d)"
          % (st.state, st.pcm_cnt, st.magic_done))
    check(rd.state == SdReader.DELIVER and 0 < rd.idx < SECTOR - 1,
          "the reader is part-way through a GRANTED sector when start falls",
          "reader state=%d idx=%d" % (rd.state, rd.idx))
    pcm_at_drop = st.pcm_cnt
    sectors_at_drop = rd.sectors_read

    # --- phase B: start low. The retire must wait for sd_sec_read_end.
    tail_words, request_held, abandoned = [], [], []
    c = 0
    while st.state != S_IDLE and c < 2000:
        grant_live, _, req = step_once(st, rd, 0, tail_words)
        if grant_live:
            request_held.append(req)
            if not req:
                abandoned.append(c)
        c += 1
    check(st.state == S_IDLE and st.sd_sec_read == 0,
          "the streamer retires to S_IDLE on the sector's end pulse (%d cycles "
          "after start fell), not the cycle start goes low" % c)
    check(len(request_held) > 0 and all(request_held),
          "the request stays asserted for every cycle of the granted sector "
          "(%d cycles), so the port is never granted-but-abandoned"
          % len(request_held),
          "abandoned on cycles %s" % abandoned)
    check(len(tail_words) <= FRAMES_PER_SECTOR,
          "fifo_we is NOT gated by start, so the remainder of the in-flight "
          "sector still lands in the FIFO: %d stale words (<= %d), which the "
          "player drains unheard in <= %.1f ms while the tone is muxed in"
          % (len(tail_words), FRAMES_PER_SECTOR,
             len(tail_words) / 48000.0 * 1000.0))

    # --- phase C: withdrawn. The streamer must never touch the SD bus again,
    #     which is the whole point of MUSC 0 -- the pictures get the port alone.
    we_while_idle = 0
    for _ in range(3000):
        _, _, req = step_once(st, rd, 0, None)
        we_while_idle += st.fifo_we
        if req:
            break
    check(rd.sectors_read == sectors_at_drop,
          "no new sector is requested while withdrawn (sectors_read frozen at "
          "%d over 3000 clocks)" % rd.sectors_read,
          "was %d at the drop" % sectors_at_drop)
    check(st.state == S_IDLE and st.sd_sec_read == 0 and we_while_idle == 0,
          "the withdrawn streamer sits silent in S_IDLE and does not spin the "
          "shared SD port")

    # --- phase D: re-arm. S_IDLE re-initialises pcm_cnt and the address, so the
    #     track restarts from the top rather than resuming mid-song.
    check(st.pcm_cnt == pcm_at_drop + 4 * len(tail_words),
          "the withdraw deliberately does NOT rewind pcm_cnt: at S_IDLE it is "
          "%d, exactly the %d it had when start fell plus the %d stale words "
          "the in-flight sector still produced"
          % (st.pcm_cnt, pcm_at_drop, len(tail_words)))
    step_once(st, rd, 1, None)
    check(st.pcm_cnt == 0 and st.sd_sec_read_addr == start_sector
          and st.state == S_READ,
          "one cycle after re-arm pcm_cnt is 0 and the address is back at "
          "wav_start_sector (%d)" % st.sd_sec_read_addr)
    words = []
    for _ in range(3200):
        step_once(st, rd, 1, words)
    check(words[:len(frames)] == exp,
          "the re-armed replay reproduces the whole track exactly and "
          "frame-aligned -> it restarted from byte 0, not from the drop point")

    # --- phase E: magic_done is sticky. Withdraw once more, trash the header on
    #     the card, and re-arm: RIFF/WAVE must NOT be re-verified.
    guard = 0
    while st.state != S_IDLE and guard < 4000:
        step_once(st, rd, 0, None)
        guard += 1
    trashed = bytearray(card[start_sector])
    trashed[0:12] = b"XXXXYYYYZZZZ"
    card[start_sector] = bytes(trashed)
    words = []
    for _ in range(3200):
        step_once(st, rd, 1, words)
    check(st.state != S_FAULT and st.magic_done == 1,
          "a corrupted header on replay is not re-checked: magic_done stays 1 "
          "and the streamer never faults", "state=%d" % st.state)
    check(words[:len(frames)] == exp,
          "so MUSC 0 -> MUSC 1 always replays the same clean track, however "
          "often it is toggled")


def test_control_midsector_retire(cfg):
    print("\n[8] NEGATIVE CONTROL: a mid-sector retire loses the granted sector")
    # A short de-arm pulse, driven identically through the real boundary retire
    # and through the mutant that gives up the cycle start goes low. The real one
    # absorbs it without losing a single audio byte. The mutant abandons a sector
    # the arbiter has already granted, then on re-arm mistakes that sector's end
    # pulse for its own, so it skips sector 0 entirely: the header is never read
    # and every frame from the pulse onward is misaligned -- silently, because
    # magic_done is already 1.
    start_sector = 0x7000
    frames = make_frames(400)
    wav = build_wav(frames)
    exp = expected_words(frames)
    PULSE = 3

    def drive(stepfn):
        card = build_card(wav, start_sector)
        st = AudioStream(cfg, start_sector, len(wav))
        rd = SdReader(card)
        words, abandoned = [], 0
        mid = False
        for i in range(700 + PULSE + 3200):
            start = 0 if 700 <= i < 700 + PULSE else 1
            if i == 700:
                mid = (rd.state == SdReader.DELIVER and 0 < rd.idx < SECTOR - 1)
            grant_live, _, req = stepfn(st, rd, start, words)
            if grant_live and not req:
                abandoned += 1
        return st, words, abandoned, mid

    st_r, words_r, abn_r, mid_r = drive(step_once)
    st_m, words_m, abn_m, mid_m = drive(step_immediate_withdraw)

    check(mid_r and mid_m,
          "both runs are part-way through a granted sector when the %d-cycle "
          "de-arm pulse lands" % PULSE)
    check(abn_r == 0,
          "real: the request is high on every cycle the arbiter still holds the "
          "port for audio (0 abandoned cycles)")
    expect_fail(abn_m == 0,
                "mutant: retiring mid-sector leaves %d cycles where the port is "
                "granted to audio but audio is no longer asking for it" % abn_m)
    check(words_r[:len(frames)] == exp,
          "real: a %d-cycle de-arm glitch costs nothing -- all %d frames still "
          "come out exactly right" % (PULSE, len(frames)))
    expect_fail(words_m[:len(frames)] == exp,
                "mutant: the same glitch corrupts the stream from frame %d "
                "onward" % next((i for i in range(len(frames))
                                 if words_m[i] != exp[i]), -1))
    expect_fail(st_m.state == S_FAULT,
                "mutant: the damage is SILENT garbage, not a fault -- magic_done "
                "is already 1 so the skipped header is never noticed")


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
    test_withdraw_and_rearm(cfg)
    test_control_midsector_retire(cfg)

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
