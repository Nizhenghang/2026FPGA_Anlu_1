#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model of the SD sector-read port arbiter in sd_card_bmp.v.

No Verilog simulator on this machine (see README "验证工具"), so this mirrors the
arbiter, the two consumers sharing the port, and the reader serving them,
register for register, and asserts the properties the audio/video sync change
depends on.

WHAT CHANGED IN THE RTL
The single SD sector-read port used to be switched by the audio_phase LEVEL,
which forced bmp_read (pictures) and sd_audio_stream (music) to be strictly
time-disjoint, so the track could only start once every picture had committed --
about five seconds of silent slideshow after the first picture appeared. It is
now a one-hot sector-granularity arbiter, and the track is armed by the same
event that raises display_valid.

THREE PROPERTIES ARE LOAD-BEARING, each paired with a negative control below:
  * sd_sec_read is driven by the OWNER FLAGS, never by the consumer's request.
    Both consumers drop their request for exactly one cycle at each sector
    boundary, and on that cycle the arbiter is idle and re-arbitrates; if the
    port followed the request instead, the reader would latch a sector for the
    loser on the very cycle the grant went to the winner, and the winner would
    swallow the loser's bytes (control N1 measures how often the reader starts
    an ungranted sector, control N1b forces the contention that turns it into
    misdelivery).
  * a grant covers exactly one sector and is released only on sd_sec_read_end,
    because sd_card_sec_read_write latches the address in S_WAIT_READ_WRITE on
    any cycle sd_sec_read is high and returns there only through the
    single-cycle S_READ_END. Re-arbitrating mid-sector retargets a sector that
    is already in flight (control N3 drops the !own guard).
  * each consumer sees only the response gated by its own ownership, otherwise
    bmp_read's bmp_len_cnt counts audio bytes into the file length and shifts
    every later pixel (control N2 broadcasts the response).

Owner-driven sd_sec_read also settles the withdrawal case for free: load_abort
puts bmp_read back in ST_IDLE, a RIFF/WAVE rejection puts sd_audio_stream in
S_FAULT, and MUSC 0 (music_req falling, i.e. the test tone re-selected) retires
sd_audio_stream to S_IDLE at the end of the sector in flight. All three drop
their request while the grant is still outstanding or shortly after it.
Because the port no longer depends on the request, the granted sector always
runs to its end pulse and the arbiter always releases. The wasted sector is
ignored by the withdrawer -- bmp_len_cnt only counts in ST_LOAD_DATA and rd_cnt
only advances under reading_sector, which excludes ST_IDLE. music_req also gates
the arm itself, so in the power-up tone mode the streamer never enters the
arbiter at all (pass F, pass G, control N4).

A withdrawal does NOT deadlock a request-driven mux, which is what an earlier
reading of this design assumed: the reader latches on the grant cycle, one edge
before the consumer's request can fall. So E and E2 assert that the shipped
arbiter keeps serving the other consumer, and N1 attacks the boundary race
instead.

TIMING COMPRESSION
One factor compresses everything: the reader delivers a byte every BYTE_CYCLES
clocks instead of the real 17*div+36, and the audio FIFO drains a frame every
round(CLK/SAMPLE_RATE * BYTE_CYCLES/real_byte_cost) clocks instead of every
2083. The RATIO between an audio sector period and a picture sector period is
exact, which is all the latency and bandwidth results depend on; absolute
milliseconds are recovered by real_ms(). Sector counts are likewise divided by
SCALE, which does not touch that ratio either.

Constants are parsed out of the .v files rather than restated here, so the model
cannot silently drift from the RTL.

Run from anywhere:  python tools/sim_sd_arbiter.py
"""
import glob
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
SD = os.path.join(ROOT, "src", "user_source", "hdl_source", "SD")
HDL = os.path.join(ROOT, "src", "user_source", "hdl_source")
IP = os.path.join(HDL, "IP")
STAGE = os.path.join(ROOT, "doc", "convert", "demo_labeled_stage")

SECTOR = 512
CYC_PER_SEC = 1_500_000

FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    CHECKS[0] += 1
    tail = (" -- " + detail) if detail else ""
    if cond:
        print("    ok    %s" % label)
        return True
    FAILURES.append("%s%s" % (label, tail))
    print("    FAIL  %s%s" % (label, tail))
    return False


def expect_fail(cond, label, detail=""):
    """Negative control: passes when cond is False."""
    CHECKS[0] += 1
    tail = (" -- " + detail) if detail else ""
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s%s" % (label, tail))
    print("    FAIL  control did NOT bite: %s%s" % (label, tail))
    return False


def note(text):
    print("    ..    %s" % text)


# --------------------------------------------------------------------------
# Parse the RTL constants.
# --------------------------------------------------------------------------
def _strip_comments(text):
    return re.sub(r"//[^\n]*", "", text)


def _read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


_RADIX = {"b": 2, "o": 8, "d": 10, "h": 16}


def _int(expr):
    """Verilog literal or plain integer: 3'd4, 9'd256, 1'b1, 32'd131071, 44."""
    expr = expr.strip().rstrip(",").strip().replace("_", "")
    m = re.match(r"^\d*'\s*([bBdDhHoO])\s*([0-9a-fA-F]+)$", expr)
    if m:
        return int(m.group(2), _RADIX[m.group(1).lower()])
    return int(expr, 0)


def _find(pattern, text, what, flags=0):
    m = re.search(pattern, text, flags)
    if not m:
        raise SystemExit("sim_sd_arbiter: cannot find %s; the model would "
                         "silently drift from the RTL." % what)
    return m


def parse_rtl():
    cfg = {}

    bmp = _strip_comments(_read(os.path.join(SD, "sd_card_bmp.v")))
    cfg["SCAN_TARGET_COUNT"] = _int(
        _find(r"SCAN_TARGET_COUNT\s*=\s*([^,)\n]+)", bmp,
              "SCAN_TARGET_COUNT").group(1))
    cfg["AUDIO_START_ON_FIRST_IMAGE"] = _int(
        _find(r"AUDIO_START_ON_FIRST_IMAGE\s*=\s*([^,)\n]+)", bmp,
              "AUDIO_START_ON_FIRST_IMAGE").group(1))

    # The audio source select. All three are required, so removing music_req
    # from the port list, dropping it out of audio_start_now, or deleting the
    # de-arm term fails here instead of silently leaving the model to test an
    # arm law the silicon does not implement.
    _find(r"input\s+music_req\b", bmp, "the music_req port")
    _find(r"wire\s+audio_start_now\s*=\s*music_req\s*&&", bmp,
          "music_req gating audio_start_now")
    _find(r"audio_phase\s*&&\s*!music_req", bmp, "the music_req de-arm term")

    inst = _find(r"sd_audio_stream\s*#\s*\((.*?)\)\s*sd_audio_stream_m0", bmp,
                 "the sd_audio_stream instance", re.S).group(1)
    cfg["HDR_LEN"] = _int(
        _find(r"\.HDR_LEN\s*\(([^)]*)\)", inst, "HDR_LEN").group(1))
    cfg["PAUSE_THRESH"] = _int(
        _find(r"\.PAUSE_THRESH\s*\(([^)]*)\)", inst, "PAUSE_THRESH").group(1))

    # sd_card_top's high-speed divider, i.e. the one in force once init
    # completes.
    top = _strip_comments(_read(os.path.join(SD, "sd_card_top.v")))
    cfg["SPI_HIGH_SPEED_DIV"] = _int(
        _find(r"SPI_HIGH_SPEED_DIV\s*=\s*([^,)\n]+)", top,
              "SPI_HIGH_SPEED_DIV").group(1))

    # "One SPI byte costs 17*div + 36 sys_clk" -- parsed from the comment that
    # states it, so a change to spi_master's framing shows up here.
    rw = _read(os.path.join(SD, "sd_card_sec_read_write.v"))
    m = _find(r"One SPI byte costs\s*(\d+)\s*\*\s*div\s*\+\s*(\d+)\s*sys_clk",
              rw, "the SPI byte-cost formula")
    cfg["BYTE_COST_MUL"], cfg["BYTE_COST_ADD"] = int(m.group(1)), int(m.group(2))
    cfg["REAL_BYTE_CYCLES"] = (cfg["BYTE_COST_MUL"] * cfg["SPI_HIGH_SPEED_DIV"]
                               + cfg["BYTE_COST_ADD"])
    cfg["RD_RETRY_MAX"] = _int(
        _find(r"RD_RETRY_MAX\s*=\s*([^;\n]+)", _strip_comments(rw),
              "RD_RETRY_MAX").group(1))

    # sd_card_cmd's read timeout: the bound on how long a granted sector can
    # take before an end pulse is guaranteed.
    cmd = _strip_comments(_read(os.path.join(SD, "sd_card_cmd.v")))
    cfg["READ_TIMEOUT_MAX"] = _int(
        _find(r"READ_TIMEOUT_MAX\s*=\s*([^;\n]+)", cmd,
              "READ_TIMEOUT_MAX").group(1))

    # Audio FIFO depth, the player's sample rate, and the SD clock.
    af = _strip_comments(_read(os.path.join(IP, "afifo_16_32_256.v")))
    cfg["FIFO_DEPTH"] = 1 << int(
        _find(r"module\s+wfifo_32_32_512\s*#\(.*?ADDR_WIDTH_W\s*=\s*(\d+)", af,
              "wfifo_32_32_512 ADDR_WIDTH_W", re.S).group(1))
    tl = _strip_comments(_read(os.path.join(HDL, "top_tf_hdmi_audio.v")))
    player = _find(r"audio_pcm_player\s*#\s*\((.*?)\)\s*u_audio_pcm_player", tl,
                   "the audio_pcm_player instance", re.S).group(1)
    cfg["SAMPLE_RATE_HZ"] = _int(
        _find(r"\.SAMPLE_RATE_HZ\s*\(([^)]*)\)", player,
              "SAMPLE_RATE_HZ").group(1))
    card = _find(r"sd_card_bmp\s*#\s*\((.*?)\)\s*sd_card_bmp_m0", tl,
                 "the sd_card_bmp instance", re.S).group(1)
    cfg["SD_CLK_HZ"] = _int(
        _find(r"\.CLK_FREQ_HZ\s*\(([^)]*)\)", card, "CLK_FREQ_HZ").group(1))
    cfg["SCK_HZ"] = cfg["SD_CLK_HZ"] // (2 * (cfg["SPI_HIGH_SPEED_DIV"] + 2))

    cfg["FIFO_SLACK_MS"] = cfg["FIFO_DEPTH"] / (cfg["SAMPLE_RATE_HZ"] / 1000.0)
    cfg["READ_TIMEOUT_MS"] = cfg["READ_TIMEOUT_MAX"] / (cfg["SD_CLK_HZ"] / 1000.0)
    return cfg


def real_images():
    """Sector counts of the actual judge-demo card, newest staging dir."""
    out = []
    for p in sorted(glob.glob(os.path.join(STAGE, "*.bmp"))):
        n = os.path.getsize(p)
        out.append((os.path.basename(p), n, (n + SECTOR - 1) // SECTOR))
    return out


def card_byte(addr, idx):
    """Deterministic pseudo-content, so a wrong-address delivery shows up as
    wrong bytes and not merely as a wrong tag."""
    return (addr * 7 + idx * 13 + (addr >> 5)) & 0xFF


def card_sector(addr):
    return [(addr, card_byte(addr, i)) for i in range(SECTOR)]


# --------------------------------------------------------------------------
# Reader: sd_card_sec_read_write.v read path.
# --------------------------------------------------------------------------
R_WAIT, R_CMD17, R_READ, R_END, R_GAP = range(5)


class SdReader:
    """S_WAIT_READ_WRITE / S_CMD17 / S_READ / S_READ_END / S_RETRY_GAP.

    S_WAIT_READ_WRITE latches sd_sec_read_addr on ANY cycle sd_sec_read is high
    and is re-entered only through the single-cycle S_READ_END, which is what
    makes a sector atomic. S_RETRY_GAP re-arms CMD17 for the SAME address after
    a missed 0xFE start token and produces NO end pulse, so an owner granted
    before a retry keeps the port for the whole retry budget.
    """

    def __init__(self, byte_cycles, cmd_cycles=20, gap_cycles=64, retry_max=2):
        self.byte_cycles = byte_cycles
        self.cmd_cycles = cmd_cycles
        self.gap_cycles = gap_cycles
        self.retry_max = retry_max
        self.miss_plan = 0        # retries still to inject
        self.state = R_WAIT
        self.addr = 0
        self.cyc = 0
        self.byte_idx = 0
        self.retry = 0
        self.sectors_served = 0
        self.gaps_entered = 0

    def out_valid(self):
        return (self.state == R_READ and self.byte_idx < SECTOR
                and (self.cyc % self.byte_cycles) == 0)

    def out_byte(self):
        return card_byte(self.addr, self.byte_idx)

    def out_end(self):
        return self.state == R_END

    @property
    def in_flight(self):
        return self.state in (R_CMD17, R_READ, R_END)

    def step(self, sec_read, sec_addr):
        st = self.state
        if st == R_WAIT:
            if sec_read:
                self.addr = sec_addr
                self.state = R_CMD17
                self.cyc = 0
                self.retry = self.retry_max
        elif st == R_CMD17:
            self.cyc += 1
            if self.cyc >= self.cmd_cycles:
                self.cyc = 0
                self.byte_idx = 0
                # A missed start token surfaces as cmd_req_error inside S_READ;
                # divert before any payload byte is delivered.
                if self.miss_plan > 0 and self.retry > 0:
                    self.miss_plan -= 1
                    self.retry -= 1
                    self.state = R_GAP
                    self.gaps_entered += 1
                else:
                    self.state = R_READ
        elif st == R_READ:
            if self.out_valid():
                self.byte_idx += 1
            self.cyc += 1
            if self.byte_idx >= SECTOR:
                self.state = R_END
        elif st == R_END:
            self.state = R_WAIT
            self.sectors_served += 1
        elif st == R_GAP:
            self.cyc += 1
            if self.cyc >= self.gap_cycles:
                self.cyc = 0
                self.state = R_CMD17


# --------------------------------------------------------------------------
# Consumers.
# --------------------------------------------------------------------------
B_IDLE, B_HDR, B_ACK, B_DATA = range(4)


class BmpConsumer:
    """bmp_read.v ST_IDLE / ST_LOAD_HDR / ST_LOAD_WAIT / ST_LOAD_DATA.

    ST_LOAD_DATA holds sd_sec_read high and drops it for exactly one cycle on
    the gated end, re-asserting with addr+1. ST_LOAD_HDR re-reads the SAME
    sector as the first data sector (sd_sec_read_addr <= load_sector_latched),
    so an image costs 1 + n reads.
    """

    def __init__(self, images, ack_cycles=8, arm_cycles=3):
        self.images = images          # [(start_sector, n_data_sectors), ...]
        self.ack_cycles = ack_cycles
        self.arm_cycles = arm_cycles
        self.state = B_IDLE
        self.req = 0
        self.addr = 0
        self.k = 0
        self.data_left = 0
        self.ack_left = 0
        self.arm_left = arm_cycles
        self.recv = []                # (addr, byte) actually delivered to us
        self.ends_seen = 0
        self.committed = 0            # images fully read
        self.aborted = False

    @property
    def busy(self):
        return not self.aborted and (self.k < len(self.images)
                                     or self.state != B_IDLE)

    def expected_addrs(self):
        """Address sequence this consumer should be served, in order."""
        out = []
        for s, n in self.images:
            out.append(s)                                  # header read
            out.extend(s + i for i in range(n))            # data reads
        return out

    def step(self, dv, en, abort=False):
        st, req, addr = self.state, self.req, self.addr
        nreq, naddr, nstate, nabort = req, addr, st, self.aborted

        if abort:
            # load_abort: back to ST_IDLE with sd_sec_read low. The address
            # register is NOT cleared, so a sector already granted still
            # completes against a valid LBA.
            nstate, nreq, nabort = B_IDLE, 0, True
        elif st == B_IDLE:
            nreq = 0
            if self.k < len(self.images):
                self.arm_left -= 1
                if self.arm_left <= 0:
                    self.arm_left = self.arm_cycles
                    naddr = self.images[self.k][0]
                    nstate = B_HDR
        elif st == B_HDR:
            nreq = 1
            if en:
                nreq = 0
                nstate = B_ACK
                self.ack_left = self.ack_cycles
                naddr = self.images[self.k][0]     # re-read as first data sector
                self.data_left = self.images[self.k][1]
        elif st == B_ACK:
            nreq = 0                               # ST_LOAD_WAIT: request low
            self.ack_left -= 1
            if self.ack_left <= 0:
                nstate = B_DATA
        elif st == B_DATA:
            nreq = 1
            if en:
                nreq = 0
                self.data_left -= 1
                self.ends_seen += 1
                if self.data_left <= 0:
                    self.committed += 1
                    self.k += 1
                    nstate = B_IDLE
                else:
                    naddr = addr + 1

        if dv is not None:
            self.recv.append(dv)

        self.req, self.addr, self.state, self.aborted = nreq, naddr, nstate, nabort


A_IDLE, A_READ, A_WAIT, A_FAULT = range(4)


class AudioConsumer:
    """sd_audio_stream.v S_IDLE / S_READ / S_WAIT / S_FAULT.

    Same one-cycle request gap at a sector boundary as bmp_read. S_WAIT holds
    the request low while wrusedw >= PAUSE_THRESH, and that pause is the
    multi-cycle window the arbiter uses to serve pictures. A falling `armed`
    (MUSC 0, i.e. audio_phase de-asserted) retires to S_IDLE -- at the sector's
    end pulse from S_READ, immediately from S_WAIT.
    """

    def __init__(self, wav_sector, pcm_frames, hdr_len, pause_thresh,
                 drain_cycles, fifo_depth):
        self.wav_sector = wav_sector
        self.pcm_frames = pcm_frames
        self.hdr_len = hdr_len
        self.pause_thresh = pause_thresh
        self.drain_cycles = drain_cycles
        self.fifo_depth = fifo_depth
        self.state = A_IDLE
        self.req = 0
        self.addr = 0
        self.wrusedw = 0
        self.hdr_left = 0
        self.byte_phase = 0
        self.frame_cnt = 0
        self.recv = []
        self.drained = 0
        self.underrun_cycles = 0
        self.first_byte_cycle = None
        self.fault = False
        self.fault_cycle = None
        self.withdraw_cycle = None

    @property
    def pause_now(self):
        return self.wrusedw >= self.pause_thresh

    def step(self, dv, en, armed, cycle, fault_inject=False):
        st, req, addr = self.state, self.req, self.addr
        nreq, naddr, nstate = req, addr, st

        if fault_inject and st == A_READ:
            # RIFF/WAVE rejected mid-header: the request drops while the granted
            # sector is still in flight and never comes back.
            nstate, nreq = A_FAULT, 0
            self.fault, self.fault_cycle = True, cycle
        elif st == A_IDLE:
            nreq = 0
            if armed:
                naddr = self.wav_sector
                self.hdr_left = self.hdr_len
                self.byte_phase = 0
                self.frame_cnt = 0
                nstate = A_READ
        elif st == A_READ:
            nreq = 1
            if en:
                nreq = 0
                if not armed:
                    # MUSC 0 withdraw. Retires at the sector boundary, never
                    # mid-sector, so the arbiter's release is always observed by
                    # an idle streamer. The address and frame_cnt are NOT
                    # rewound here; A_IDLE re-initialises both on the next arm.
                    nstate = A_IDLE
                    self.withdraw_cycle = cycle
                elif self.frame_cnt >= self.pcm_frames:
                    # single-track loop: rewind and re-skip the header
                    naddr = self.wav_sector
                    self.hdr_left = self.hdr_len
                    self.byte_phase = 0
                    self.frame_cnt = 0
                else:
                    naddr = addr + 1
                    nstate = A_WAIT if self.pause_now else A_READ
        elif st == A_WAIT:
            nreq = 0
            if not armed:
                nstate = A_IDLE          # already parked, so this is immediate
                self.withdraw_cycle = cycle
            elif not self.pause_now:
                nstate = A_READ
        elif st == A_FAULT:
            nreq = 0

        # FIFO drain: the player pops one frame per sample tick, and keeps
        # ticking on underrun so the ACR reference never gaps.
        self.drained += 1
        if self.drained >= self.drain_cycles:
            self.drained = 0
            if self.wrusedw > 0:
                self.wrusedw -= 1
            else:
                self.underrun_cycles += 1

        if dv is not None:
            self.recv.append(dv)
            if self.first_byte_cycle is None:
                self.first_byte_cycle = cycle
            if self.state == A_FAULT:
                pass                     # withdrawn: bytes are ignored
            elif self.hdr_left > 0:
                self.hdr_left -= 1
            else:
                self.byte_phase += 1
                if self.byte_phase >= 4:
                    self.byte_phase = 0
                    self.frame_cnt += 1
                    if self.wrusedw < self.fifo_depth:
                        self.wrusedw += 1

        self.req, self.addr, self.state = nreq, naddr, nstate


# --------------------------------------------------------------------------
# Arbiter: the RTL under test.
# --------------------------------------------------------------------------
class Arbiter:
    """One-hot ownership of the sector-read port.

    req_mode="owner"       -> sd_sec_read = arb_bmp_own | arb_aud_own   (shipped)
    req_mode="passthrough" -> sd_sec_read = the grantee's request       (N1)
    guard=False            -> re-arbitrate every cycle, no one-sector hold (N3)
    gate=False             -> broadcast data_valid/end to both consumers  (N2)
    """

    def __init__(self, req_mode="owner", guard=True, gate=True,
                 audio_priority=True):
        self.req_mode = req_mode
        self.guard = guard
        self.gate = gate
        self.audio_priority = audio_priority
        self.bmp_own = 0
        self.aud_own = 0
        self.grants = {"bmp": 0, "aud": 0}
        self.switches_midsector = 0

    def sec_read(self, bmp_req, aud_req):
        if self.req_mode == "owner":
            return 1 if (self.bmp_own or self.aud_own) else 0
        return aud_req if self.aud_own else bmp_req

    def sec_addr(self, bmp_addr, aud_addr):
        return aud_addr if self.aud_own else bmp_addr

    def step(self, end, bmp_req, aud_req):
        prev = (self.bmp_own, self.aud_own)
        if end:
            self.bmp_own = 0
            self.aud_own = 0
        elif not self.guard or not prev[0] and not prev[1]:
            if self.audio_priority:
                if aud_req:
                    self.aud_own, self.bmp_own = 1, 0
                elif bmp_req:
                    self.bmp_own, self.aud_own = 1, 0
            else:
                if bmp_req:
                    self.bmp_own, self.aud_own = 1, 0
                elif aud_req:
                    self.aud_own, self.bmp_own = 1, 0
        cur = (self.bmp_own, self.aud_own)
        if cur != (0, 0):
            if prev == (0, 0):
                self.grants["bmp" if cur[0] else "aud"] += 1
            elif cur != prev:
                self.switches_midsector += 1


# --------------------------------------------------------------------------
# The bench.
# --------------------------------------------------------------------------
class Bench:
    def __init__(self, cfg, images, byte_cycles=1, wav_sector=900000,
                 wav_frames=0, arb=None, gate_arm_on_music=True):
        self.cfg = cfg
        self.byte_cycles = byte_cycles
        # gate_arm_on_music=False reproduces the pre-change arm law, where
        # audio_phase ignored music_req. Only used by control N4.
        self.gate_arm_on_music = gate_arm_on_music
        # Audio drain keeps the audio-sector : picture-sector ratio exact under
        # compression; see the module docstring.
        real_drain = cfg["SD_CLK_HZ"] // cfg["SAMPLE_RATE_HZ"]       # 2083
        self.drain_cycles = max(
            1, round(real_drain * byte_cycles / cfg["REAL_BYTE_CYCLES"]))
        self.reader = SdReader(byte_cycles, retry_max=cfg["RD_RETRY_MAX"])
        self.bmp = BmpConsumer(images)
        # wav_frames=0 -> an effectively endless single track (loop never taken)
        self.aud = AudioConsumer(wav_sector, wav_frames or (1 << 30),
                                 cfg["HDR_LEN"], cfg["PAUSE_THRESH"],
                                 self.drain_cycles, cfg["FIFO_DEPTH"])
        self.arb = arb or Arbiter()
        self.gate = self.arb.gate
        self.cycle = 0
        self.audio_phase = 0
        self.music_req = 1          # sd_card_bmp's music_req input (MUSC 0/1)
        self.audio_arm_cycle = None
        self.audio_dearm_cycle = None
        self.sector_owner = {}      # latched addr -> 'bmp'/'aud' at latch time
        self.served = []            # (cycle, addr, owner) per latched sector
        self.bmp_asked = set()      # addrs driven while bmp owned the port
        self.aud_asked = set()
        self.bmp_served = []        # addrs served to bmp, in order
        self.aud_served = []
        self.bmp_latency = []
        self.aud_latency = []
        self.ungranted_latch = []   # cycles the reader latched with no owner
        self.ungranted_count = 0
        self.addr_mismatch = []     # cycles the in-flight addr != grantee's addr
        self.mismatch_count = 0
        self.bad_count = 0          # bytes delivered to a consumer that never
        self.bad_first = []         #   asked the port for that address
        self.arb_idle_with_req = 0  # cycles the arbiter wasted a ready request
        self.aborted = False
        self._bmp_pending = None
        self._aud_pending = None
        self.stopped_at = None

    # -- injection predicates, each takes the bench --------------------------
    def abort_on_grant(self):
        """load_abort landing on the very cycle the arbiter grants pictures."""
        return bool(self.arb.bmp_own and self.audio_phase and not self.aborted)

    def fault_on_grant(self):
        """RIFF/WAVE rejection landing while audio owns the port."""
        return bool(self.arb.aud_own and self.aud.state == A_READ
                    and not self.aud.fault)

    def run(self, max_cycles, music_req=1, abort_when=None, fault_when=None,
            miss_at=None, stop_when=None):
        # music_req is sd_card_bmp's input: a constant level, or a predicate on
        # the bench so a pass can drop MUSC 0 mid-load and raise it again.
        req_fn = music_req if callable(music_req) else (lambda b: music_req)
        for _ in range(max_cycles):
            c = self.cycle
            self.music_req = bool(req_fn(self))
            if miss_at is not None and c == miss_at and self.reader.miss_plan == 0:
                self.reader.miss_plan = self.cfg["RD_RETRY_MAX"]
            abort = bool(abort_when and abort_when(self))
            fault = bool(fault_when and fault_when(self))

            # --- 1. combinational outputs off the pre-cycle snapshot ---------
            end = self.reader.out_end()
            valid = self.reader.out_valid()
            raw = (self.reader.addr, self.reader.out_byte()) if valid else None

            bmp_req, aud_req = self.bmp.req, self.aud.req
            bmp_own, aud_own = self.arb.bmp_own, self.arb.aud_own
            sec_read = self.arb.sec_read(bmp_req, aud_req)
            sec_addr = self.arb.sec_addr(self.bmp.addr, self.aud.addr)

            if self.gate:
                bmp_dv = raw if (valid and bmp_own) else None
                aud_dv = raw if (valid and aud_own) else None
                bmp_en, aud_en = bool(end and bmp_own), bool(end and aud_own)
            else:
                bmp_dv = aud_dv = raw
                bmp_en = aud_en = bool(end)

            # --- 2. observe -------------------------------------------------
            # Misdelivery is counted incrementally: a stop_when predicate runs
            # every cycle, so it must not rescan the received byte logs.
            if bmp_dv is not None and bmp_dv[0] not in self.bmp_asked:
                self.bad_count += 1
                if len(self.bad_first) < 3:
                    self.bad_first.append(("bmp", c, bmp_dv[0]))
            if aud_dv is not None and aud_dv[0] not in self.aud_asked:
                self.bad_count += 1
                if len(self.bad_first) < 3:
                    self.bad_first.append(("aud", c, aud_dv[0]))

            if bmp_own:
                self.bmp_asked.add(self.bmp.addr)
            if aud_own:
                self.aud_asked.add(self.aud.addr)
            if end and self.served:
                who = self.served[-1][2]
                (self.bmp_served if who == "bmp" else self.aud_served).append(
                    self.served[-1][1])

            if self.reader.state == R_WAIT and sec_read:
                owner = "aud" if aud_own else ("bmp" if bmp_own else None)
                if owner is None:
                    self.ungranted_count += 1
                    if len(self.ungranted_latch) < 3:
                        self.ungranted_latch.append((c, sec_addr))
                else:
                    self.sector_owner[sec_addr] = owner
                    self.served.append((c, sec_addr, owner))
            elif self.reader.in_flight and (bmp_own or aud_own):
                want = self.aud.addr if aud_own else self.bmp.addr
                if self.reader.addr != want:
                    self.mismatch_count += 1
                    if len(self.addr_mismatch) < 3:
                        self.addr_mismatch.append((c, self.reader.addr, want))

            # grant latency: cycles from a request rising to ownership, holding
            # the rise across the one-cycle boundary gap so the number is the
            # wait for THIS sector rather than for the previous one.
            if bmp_own:
                if self._bmp_pending is not None:
                    self.bmp_latency.append(c - self._bmp_pending)
                    self._bmp_pending = None
            elif bmp_req and self._bmp_pending is None:
                self._bmp_pending = c
            if aud_own:
                if self._aud_pending is not None:
                    self.aud_latency.append(c - self._aud_pending)
                    self._aud_pending = None
            elif aud_req and self._aud_pending is None:
                self._aud_pending = c
            if not (bmp_own or aud_own) and (bmp_req or aud_req) \
                    and self.reader.state == R_WAIT:
                self.arb_idle_with_req += 1

            # --- 3. next state, all from the snapshot -----------------------
            self.reader.step(sec_read, sec_addr)
            self.bmp.step(bmp_dv, bmp_en, abort=abort)
            self.aud.step(aud_dv, aud_en, self.audio_phase, c,
                          fault_inject=fault)
            self.arb.step(end, bmp_req, aud_req)

            # audio_phase, exactly as sd_card_bmp drives it: armed only while
            # music_req is high, de-armed the cycle music_req falls (MUSC 0).
            # gate_arm_on_music=False reproduces the pre-change law, which had
            # neither gate -- audio armed once and never let go.
            arm_ok = self.music_req or not self.gate_arm_on_music
            if not self.audio_phase and arm_ok and self.bmp.committed >= 1:
                self.audio_phase = 1
                self.audio_arm_cycle = c
            elif self.audio_phase and self.gate_arm_on_music \
                    and not self.music_req:
                self.audio_phase = 0
                self.audio_dearm_cycle = c
            if abort:
                self.aborted = True

            self.cycle += 1
            if stop_when is not None and stop_when():
                self.stopped_at = c
                return c
        self.stopped_at = None
        return None

    def real_ms(self, cycles):
        """Convert compressed model cycles back to real milliseconds."""
        scale = self.cfg["REAL_BYTE_CYCLES"] / self.byte_cycles
        return cycles * scale / (self.cfg["SD_CLK_HZ"] / 1000.0)

    def bad_delivery(self):
        """Bytes that reached a consumer which never asked the port for that
        address. O(1): counted in the observe phase, first three kept."""
        if not self.bad_count:
            return []
        return self.bad_first + [("total", self.bad_count)]

    def expected_bmp_bytes(self, addrs):
        out = []
        for a in addrs:
            out.extend(card_sector(a))
        return out


def scaled_images(factor, base=200000):
    """The real demo-card geometry, sector counts divided by `factor` so the
    passes finish in seconds while the contention ratio holds exactly."""
    imgs = real_images()
    if not imgs:
        imgs = [("a.bmp", 0, 1801), ("b.bmp", 0, 451),
                ("c.bmp", 0, 1013), ("d.bmp", 0, 2813)]
    return [(base + i * 4000, max(2, n // factor))
            for i, (_name, _size, n) in enumerate(imgs)]


def fresh(cfg, factor=8, **kw):
    return Bench(cfg, scaled_images(factor), **kw)


# --------------------------------------------------------------------------
# Passes.
# --------------------------------------------------------------------------
def pass_a_ownership(cfg):
    print("\n[A] grant exclusivity: every byte reaches the consumer the "
          "arbiter granted")
    b = fresh(cfg, factor=8)
    b.run(CYC_PER_SEC, stop_when=lambda: not b.bmp.busy)
    check(b.stopped_at is not None, "run finished all four pictures",
          "hit the cycle cap")
    bad = b.bad_delivery()
    check(not bad, "no byte reached a consumer that never asked for its address",
          "first offenders %s" % (bad,))
    check(not b.ungranted_count,
          "the reader never latched a sector the arbiter had not granted",
          "%d ungranted latches, first %s" % (b.ungranted_count,
                                              b.ungranted_latch))
    check(not b.mismatch_count,
          "the in-flight address always equalled the grantee's address",
          "%d mismatches, first %s" % (b.mismatch_count, b.addr_mismatch))
    check(b.arb.switches_midsector == 0,
          "ownership never changed while a sector was in flight",
          "%d mid-sector switches" % b.arb.switches_midsector)
    note("sectors served: bmp=%d aud=%d; grants bmp=%d aud=%d"
         % (len(b.bmp_served), len(b.aud_served),
            b.arb.grants["bmp"], b.arb.grants["aud"]))
    return b


def pass_b_bmp_integrity(cfg, b):
    print("\n[B] picture byte stream is intact across the interleaving")
    check(b.bmp.committed == len(b.bmp.images),
          "all %d pictures read to completion" % len(b.bmp.images),
          "committed=%d" % b.bmp.committed)
    want_addrs = b.bmp.expected_addrs()
    first_diff = next((i for i, (x, y) in
                       enumerate(zip(b.bmp_served, want_addrs)) if x != y), -1)
    check(b.bmp_served == want_addrs,
          "sectors served to bmp_read are exactly its request order, no skip "
          "and no duplicate",
          "served %d vs expected %d, first diff at %d"
          % (len(b.bmp_served), len(want_addrs), first_diff))
    got, want = b.bmp.recv, b.expected_bmp_bytes(want_addrs)
    bdiff = next((i for i, (x, y) in enumerate(zip(got, want)) if x != y), -1)
    check(len(got) == len(want) and got == want,
          "bmp_read received the exact byte content of those sectors, in order",
          "got %d bytes vs %d expected, first diff at %d"
          % (len(got), len(want), bdiff))
    data_sectors = sum(n for _s, n in b.bmp.images)
    check(b.bmp.ends_seen == data_sectors,
          "one gated end pulse per data sector",
          "ends=%d expected=%d" % (b.bmp.ends_seen, data_sectors))


def pass_c_audio_latency(cfg, b):
    print("\n[C] audio grant latency stays inside the FIFO's slack")
    slack_ms = cfg["FIFO_SLACK_MS"]
    worst_ms = b.real_ms(max(b.aud_latency) if b.aud_latency else 0)
    check(len(b.aud_latency) > 0, "audio was granted at all")
    check(worst_ms < slack_ms,
          "worst audio request->grant %.3f ms < FIFO slack %.2f ms"
          % (worst_ms, slack_ms), "underrun guaranteed")
    note("worst audio grant latency %d model cycles = %.3f ms; worst picture "
         "grant latency %d cycles"
         % (max(b.aud_latency) if b.aud_latency else 0, worst_ms,
            max(b.bmp_latency) if b.bmp_latency else 0))

    first, arm = b.aud.first_byte_cycle, b.audio_arm_cycle
    check(first is not None and arm is not None and first > arm,
          "audio bytes start flowing after the arm event")
    if first is not None and arm is not None:
        note("music armed on the first picture commit at cycle %d; first PCM "
             "byte %.3f ms later, i.e. that is the whole audio/video offset"
             % (arm, b.real_ms(first - arm)))

    aud_bytes = len(b.aud_served) * SECTOR
    tot_bytes = len(b.served) * SECTOR
    note("audio took %d of %d port sectors = %.1f%% of served bytes "
         "(bandwidth estimate was ~7%%)"
         % (len(b.aud_served), len(b.served), 100.0 * aud_bytes / max(1, tot_bytes)))
    note("audio FIFO underran for %d model cycles = %.3f ms of silence "
         "(startup fill only; audio_valid keeps running so ACR never gaps)"
         % (b.aud.underrun_cycles, b.real_ms(b.aud.underrun_cycles)))

    # Slowdown against the default power-up state: same geometry, music_req low
    # (the test tone is selected), so the arbiter can only ever grant pictures.
    ref = fresh(cfg, factor=8)
    ref.run(CYC_PER_SEC, music_req=0, stop_when=lambda: not ref.bmp.busy)
    check(ref.stopped_at is not None, "tone-mode reference run finished")
    if ref.stopped_at is not None and b.stopped_at is not None:
        slow = 100.0 * (b.stopped_at - ref.stopped_at) / ref.stopped_at
        note("picture load: %d cycles with music selected, %d with the test tone "
             "-> MUSC 1 costs %.1f%%; on the real card %.2f s vs %.2f s"
             % (b.stopped_at, ref.stopped_at, slow,
                b.real_ms(b.stopped_at) / 1000.0 * 8,
                b.real_ms(ref.stopped_at) / 1000.0 * 8))
        check(slow <= 15.0,
              "picture load slowdown %.1f%% stays near the 7%% bandwidth share"
              % slow, "music is taking more of the port than it should")


def pass_d_retry(cfg):
    print("\n[D] a sector retry holds the grant for the whole retry budget")
    probe = fresh(cfg, factor=16)
    hit = {}

    def mark(_b):
        if probe.audio_phase and probe.arb.bmp_own and not hit:
            hit["c"] = probe.cycle
        return False

    probe.run(CYC_PER_SEC, abort_when=mark,
              stop_when=lambda: not probe.bmp.busy)
    check("c" in hit, "found a picture sector to corrupt after music started")
    if "c" not in hit:
        return

    b = fresh(cfg, factor=16)
    b.run(CYC_PER_SEC, miss_at=hit["c"] + 1,
          stop_when=lambda: not b.bmp.busy)
    check(b.reader.gaps_entered > 0,
          "S_RETRY_GAP was actually entered (%d times)" % b.reader.gaps_entered)
    check(b.stopped_at is not None, "all four pictures still finished")
    check(not b.bad_delivery(), "no byte was misdelivered across the retry")
    check(not b.addr_mismatch, "the retried sector kept its grantee's address")
    check(b.arb.switches_midsector == 0,
          "ownership held through the retry, no mid-sector switch")
    worst_ms = b.real_ms(max(b.aud_latency) if b.aud_latency else 0)
    check(worst_ms < cfg["FIFO_SLACK_MS"],
          "audio grant latency %.3f ms still inside the %.2f ms FIFO slack "
          "with retries injected" % (worst_ms, cfg["FIFO_SLACK_MS"]))
    note("the modelled gap is %d cycles, but on silicon each attempt waits up "
         "to READ_TIMEOUT_MAX = %d cycles = %.0f ms, so %d attempts block "
         "music for up to %.0f ms -- an audible dropout, and the honest cost "
         "of sharing one port"
         % (b.reader.gap_cycles, cfg["READ_TIMEOUT_MAX"],
            cfg["READ_TIMEOUT_MS"], cfg["RD_RETRY_MAX"] + 1,
            cfg["READ_TIMEOUT_MS"] * (cfg["RD_RETRY_MAX"] + 1)))


def pass_e_abort(cfg):
    print("\n[E] load_abort on the exact cycle pictures are granted: the "
          "sector still completes and the port is released")
    b = fresh(cfg, factor=16)
    b.run(CYC_PER_SEC, abort_when=Bench.abort_on_grant,
          stop_when=lambda: b.aborted)
    check(b.aborted, "load_abort fired")
    at_abort = len(b.served)
    aud_before = len(b.aud_served)
    b.run(100_000, stop_when=lambda: len(b.aud_served) >= aud_before + 3)
    check(len(b.aud_served) > aud_before,
          "music was still served after pictures withdrew -- no lockout",
          "audio sectors %d -> %d" % (aud_before, len(b.aud_served)))
    check(not b.bad_delivery(), "no misdelivered byte around the withdrawal")
    check(b.arb.switches_midsector == 0, "no mid-sector switch")
    wasted = len(b.served) - at_abort
    note("sectors served up to the abort: %d; %d more after it. The granted "
         "sector ran to its end pulse even though bmp_read had gone back to "
         "ST_IDLE, which is what releases the arbiter."
         % (at_abort, wasted))
    return b


def pass_e_fault(cfg):
    print("\n[E2] sd_audio_stream rejects the RIFF magic mid-sector: pictures "
          "keep loading")
    b = fresh(cfg, factor=16)
    b.run(CYC_PER_SEC, fault_when=Bench.fault_on_grant,
          stop_when=lambda: b.aud.fault)
    check(b.aud.fault, "S_FAULT was entered while audio owned the port")
    bmp_before = len(b.bmp_served)
    b.run(CYC_PER_SEC, stop_when=lambda: len(b.bmp_served) >= bmp_before + 3
          or not b.bmp.busy)
    check(len(b.bmp_served) > bmp_before or not b.bmp.busy,
          "bmp_read was still served after the streamer withdrew -- no lockout",
          "picture sectors %d -> %d" % (bmp_before, len(b.bmp_served)))
    check(not b.bad_delivery(), "no misdelivered byte around S_FAULT")
    note("the abandoned sector completed and released the arbiter; "
         "sd_audio_stream ignores its bytes because fault stops it counting "
         "frames")


def pass_f_scan_regression(cfg):
    print("\n[F] tone mode (music_req low): the port behaves like the old level mux")
    b = fresh(cfg, factor=8)
    b.run(CYC_PER_SEC, music_req=0, stop_when=lambda: not b.bmp.busy)
    check(b.audio_phase == 0, "audio_phase stayed low for the whole load")
    check(b.arb.grants["aud"] == 0, "the arbiter never granted audio")
    check(len(b.aud.recv) == 0, "sd_audio_stream received no byte")
    check(b.bmp_served == b.bmp.expected_addrs(),
          "bmp_read was served its exact request order")
    check(b.bmp.recv == b.expected_bmp_bytes(b.bmp.expected_addrs()),
          "bmp_read's byte stream is identical to the pre-change level mux")
    n = max(1, len(b.served))
    overhead = b.arb_idle_with_req / float(n)
    check(overhead <= 2.0,
          "arbiter adds %.2f idle cycles per served sector (<= 2)" % overhead,
          "more than the one-cycle grant register plus the boundary gap")
    real_sectors = sum(x[2] for x in real_images()) + len(real_images())
    load_s = (b.real_ms(b.stopped_at) * 8 / 1000.0) if b.stopped_at else float("nan")
    note("%.2f idle cycles/sector over %d model sectors; the real card has "
         "%d sectors, so the arbiter costs %.1f ms against a projected %.1f s "
         "load (factor=8 compression)"
         % (overhead, n, real_sectors,
            b.real_ms(overhead * real_sectors), load_s))


def pass_g_music_req(cfg):
    print("\n[G] MUSC 0/1 live: the source select arms and de-arms the port")
    b = fresh(cfg, factor=8)
    phase = [0]
    marks = {}
    snap = {}

    def req(bench):
        """0 music selected -> 1 MUSC 0 -> 2 MUSC 1 again, all in one run."""
        if phase[0] == 0:
            # Drop MUSC 0 while audio genuinely owns an in-flight sector, so the
            # hand-back measured below is the real worst case (finish that
            # sector) rather than a lucky boundary-aligned landing.
            if len(bench.aud_served) >= 3 and bench.aud.state == A_READ \
                    and bench.arb.aud_own and bench.reader.state == R_READ \
                    and 64 < bench.reader.byte_idx < SECTOR - 64:
                phase[0] = 1
                marks["off"] = bench.cycle
                snap["aud_grants_off"] = bench.arb.grants["aud"]
                snap["aud_recv_off"] = len(bench.aud.recv)
                snap["bmp_off"] = len(bench.bmp_served)
                snap["reader_idx_off"] = bench.reader.byte_idx
        elif phase[0] == 1:
            if bench.aud.state == A_IDLE and not bench.arb.aud_own:
                phase[0] = 2
                marks["free"] = bench.cycle
                snap["aud_grants_free"] = bench.arb.grants["aud"]
                snap["aud_recv_free"] = len(bench.aud.recv)
        elif "rearm_addr" not in snap and bench.aud.state == A_READ:
            # First cycle back in S_READ: A_IDLE just reloaded the address and
            # the request is still low, so no byte of the new pass has landed.
            snap["rearm_addr"] = bench.aud.addr
            snap["rearm_frames"] = bench.aud.frame_cnt
        return 0 if phase[0] == 1 else 1

    b.run(CYC_PER_SEC, music_req=req, stop_when=lambda: not b.bmp.busy)

    check(phase[0] == 2 and b.stopped_at is not None,
          "the run reached all three phases and the picture load still finished",
          "phase=%d stopped_at=%s" % (phase[0], b.stopped_at))
    check(b.audio_dearm_cycle is not None and b.aud.withdraw_cycle is not None,
          "MUSC 0 de-armed audio_phase on cycle %s and the streamer retired on "
          "cycle %s" % (b.audio_dearm_cycle, b.aud.withdraw_cycle))
    if "off" in marks and "free" in marks:
        note("port handed back %d model cycles = %.2f ms after MUSC 0, which is "
             "just the remainder of the one in-flight sector; the bound is a "
             "drop right at the start of a sector, i.e. one whole sector = "
             "%.2f ms"
             % (marks["free"] - marks["off"],
                b.real_ms(marks["free"] - marks["off"]),
                b.real_ms(SECTOR + 21)))

    check(0 < snap.get("reader_idx_off", -1) < SECTOR,
          "MUSC 0 landed with %d of 512 payload bytes already delivered, so the "
          "sector really was in flight" % snap.get("reader_idx_off", -1))
    tail = snap.get("aud_recv_free", 0) - snap.get("aud_recv_off", 0)
    check(0 < tail <= SECTOR - snap.get("reader_idx_off", SECTOR),
          "the streamer finished that sector rather than abandoning it: %d more "
          "bytes, at most the %d remaining in it, and then stopped"
          % (tail, SECTOR - snap.get("reader_idx_off", SECTOR)))
    check(snap.get("aud_grants_free") == snap.get("aud_grants_off"),
          "the arbiter granted audio ZERO further sectors while withdrawn "
          "(%d -> %d)"
          % (snap.get("aud_grants_off", -1), snap.get("aud_grants_free", -2)))
    check(snap.get("aud_grants_off", 0) > 0 and snap.get("aud_recv_off", 0) > 0,
          "audio really was contending for the port before MUSC 0 (%d sectors, "
          "%d bytes), so the freeze above is not vacuous"
          % (snap.get("aud_grants_off", 0), snap.get("aud_recv_off", 0)))

    check(snap.get("rearm_addr") == b.aud.wav_sector
          and snap.get("rearm_frames") == 0,
          "MUSC 1 reloaded the address with wav_sector (%d) and frame_cnt 0 -> "
          "the track restarts from the top instead of resuming mid-song"
          % b.aud.wav_sector,
          "addr=%s frames=%s" % (snap.get("rearm_addr"), snap.get("rearm_frames")))
    rearmed = [a for (a, _byte) in b.aud.recv[snap.get("aud_recv_free", 0):]]
    check(bool(rearmed) and rearmed[0] == b.aud.wav_sector
          and b.arb.grants["aud"] > snap.get("aud_grants_free", 0),
          "and the first byte of the re-armed pass really came from wav_sector "
          "(%d more bytes over %d more grants)"
          % (len(rearmed), b.arb.grants["aud"] - snap.get("aud_grants_free", 0)))
    check(len(b.bmp_served) > snap.get("bmp_off", 0)
          and b.bmp_served == b.bmp.expected_addrs(),
          "pictures kept loading through the whole toggle and were served their "
          "exact request order (%d sectors)" % len(b.bmp_served))
    check(not b.bad_delivery() and not b.mismatch_count
          and not b.ungranted_count and not b.arb.switches_midsector,
          "no misdelivered byte, no address mismatch, no ungranted latch and no "
          "mid-sector ownership switch anywhere across the toggling",
          "bad=%s mismatch=%d ungranted=%d switches=%d"
          % (b.bad_delivery(), b.mismatch_count, b.ungranted_count,
             b.arb.switches_midsector))


# --------------------------------------------------------------------------
# Negative controls. Each MUST break something, or the passes above prove
# nothing.
# --------------------------------------------------------------------------
def control_n1(cfg):
    print("\n[N1] control: sd_sec_read passed through from the consumer's "
          "request instead of the owner flags")
    b = fresh(cfg, factor=16, arb=Arbiter(req_mode="passthrough"))
    b.run(CYC_PER_SEC,
          stop_when=lambda: b.bad_count or b.mismatch_count or not b.bmp.busy)
    expect_fail(not b.ungranted_count,
                "passthrough starts sectors the arbiter never granted")
    note("%d sectors were latched by the reader with no owner flag set. In "
         "this traffic pattern the arbiter happened to grant the same consumer "
         "one cycle later every time, so no byte was actually misdelivered -- "
         "that is luck, not design, and N1b below removes the luck by forcing "
         "the contention" % b.ungranted_count)


def control_n1_directed():
    """The one-cycle contention the traffic pattern above rarely hits: arbiter
    idle, reader idle, both consumers requesting."""
    print("\n[N1b] control, directed: both consumers request from an idle port")
    bmp_addr, aud_addr = 200000, 900000
    obs = {}
    for mode in ("passthrough", "owner"):
        arb = Arbiter(req_mode=mode)
        rd = SdReader(1)
        # cycle 0: own=(0,0), reader in S_WAIT_READ_WRITE, both requests high
        sec_read = arb.sec_read(1, 1)
        sec_addr = arb.sec_addr(bmp_addr, aud_addr)
        latch0 = bool(rd.state == R_WAIT and sec_read)
        addr0 = sec_addr
        rd.step(sec_read, sec_addr)
        arb.step(False, 1, 1)
        # cycle 1: the grant now belongs to audio; who owns the sector that is
        # already in flight?
        in_flight = rd.in_flight
        stolen = bool(latch0 and in_flight and arb.aud_own
                      and addr0 != aud_addr)
        latch1 = bool(rd.state == R_WAIT and arb.sec_read(1, 1))
        obs[mode] = {"latch0": latch0, "addr0": addr0, "stolen": stolen,
                     "latch1": latch1, "granted": "aud" if arb.aud_own else "bmp"}
    expect_fail(not obs["passthrough"]["stolen"],
                "passthrough commits the reader to the loser's address on the "
                "cycle the grant goes to the winner, so the winner swallows "
                "the loser's bytes")
    check(not obs["owner"]["latch0"] and obs["owner"]["latch1"]
          and not obs["owner"]["stolen"],
          "owner-driven sd_sec_read latches nothing before the grant, then "
          "latches the grantee's address",
          str(obs["owner"]))
    note("passthrough: latch on cycle 0 at addr %d, grant went to %s -> the "
         "audio streamer would parse picture pixels"
         % (obs["passthrough"]["addr0"], obs["passthrough"]["granted"]))
    note("owner: cycle 0 idle, cycle 1 latches the grantee's address")


def control_n2(cfg):
    print("\n[N2] control: data_valid/end broadcast to both consumers "
          "(today's wiring) while the arbiter still switches")
    b = fresh(cfg, factor=16, arb=Arbiter(gate=False))
    b.run(CYC_PER_SEC, stop_when=lambda: b.bad_count > 2000 or not b.bmp.busy)
    expect_fail(not b.bad_count,
                "ungated responses feed audio bytes to bmp_read and vice versa")
    got, want = b.bmp.recv, b.expected_bmp_bytes(b.bmp_served)
    expect_fail(len(got) == len(want) and got == want,
                "bmp_read's byte stream is corrupted without the gate")
    note("bmp_read received %d bytes for the %d sectors it was granted (%d of "
         "them from addresses it never asked for); on silicon bmp_len_cnt "
         "would count the extra bytes into the file length and shift every "
         "later pixel" % (len(got), len(b.bmp_served), b.bad_count))


def control_n3(cfg):
    print("\n[N3] control: no !own guard, so the arbiter re-arbitrates "
          "mid-sector")
    b = fresh(cfg, factor=16, arb=Arbiter(guard=False))
    b.run(CYC_PER_SEC,
          stop_when=lambda: b.mismatch_count or b.bad_count or not b.bmp.busy)
    expect_fail(b.arb.switches_midsector == 0,
                "ownership changes while a sector is in flight")
    expect_fail(not b.mismatch_count and not b.bad_count,
                "a mid-sector switch retargets the bytes of a sector the "
                "reader already latched")
    note("%d mid-sector switches, %d address mismatches, misdelivered bytes %s"
         % (b.arb.switches_midsector, b.mismatch_count, b.bad_delivery()))


def control_n4(cfg):
    print("\n[N4] control: audio_phase armed without music_req (the pre-change law)")
    gated = fresh(cfg, factor=16)
    gated.run(CYC_PER_SEC, music_req=0, stop_when=lambda: not gated.bmp.busy)
    old = fresh(cfg, factor=16, gate_arm_on_music=False)
    old.run(CYC_PER_SEC, music_req=0, stop_when=lambda: not old.bmp.busy)

    check(gated.arb.grants["aud"] == 0 and len(gated.aud.recv) == 0,
          "shipped: with the test tone selected audio is never armed, so it "
          "never enters the arbiter")
    expect_fail(old.arb.grants["aud"] == 0,
                "the old law arms the streamer anyway and takes %d sectors away "
                "from the picture load" % old.arb.grants["aud"])
    expect_fail(len(old.aud.recv) == 0,
                "and reads %d bytes of MUSIC.WAV that the output mux is throwing "
                "away" % len(old.aud.recv))
    if gated.stopped_at and old.stopped_at:
        saving = 100.0 * (old.stopped_at - gated.stopped_at) / old.stopped_at
        check(saving > 0.0,
              "the tone-mode picture load finishes %.1f%% sooner than the old "
              "always-armed behaviour (%d vs %d model cycles)"
              % (saving, gated.stopped_at, old.stopped_at))
        note("on the real card that is %.2f s against %.2f s"
             % (gated.real_ms(gated.stopped_at) / 1000.0 * 16,
                old.real_ms(old.stopped_at) / 1000.0 * 16))


def main():
    cfg = parse_rtl()
    print("sim_sd_arbiter: SD sector-read port arbiter model")
    print("  parsed from RTL: SCAN_TARGET_COUNT=%d "
          "AUDIO_START_ON_FIRST_IMAGE=%d HDR_LEN=%d PAUSE_THRESH=%d "
          "FIFO_DEPTH=%d RD_RETRY_MAX=%d"
          % (cfg["SCAN_TARGET_COUNT"], cfg["AUDIO_START_ON_FIRST_IMAGE"],
             cfg["HDR_LEN"], cfg["PAUSE_THRESH"], cfg["FIFO_DEPTH"],
             cfg["RD_RETRY_MAX"]))
    print("  SD clock %.0f MHz, SPI SCK %.1f MHz, one byte %d sys_clk, "
          "FIFO slack %.2f ms"
          % (cfg["SD_CLK_HZ"] / 1e6, cfg["SCK_HZ"] / 1e6,
             cfg["REAL_BYTE_CYCLES"], cfg["FIFO_SLACK_MS"]))
    imgs = real_images()
    if imgs:
        print("  judge-demo card: " + ", ".join(
            "%s %d sectors" % (n, s) for n, _b, s in imgs))
    else:
        print("  WARNING: %s has no BMPs, falling back to remembered sizes"
              % STAGE)

    b = pass_a_ownership(cfg)
    pass_b_bmp_integrity(cfg, b)
    pass_c_audio_latency(cfg, b)
    pass_d_retry(cfg)
    pass_e_abort(cfg)
    pass_e_fault(cfg)
    pass_f_scan_regression(cfg)
    pass_g_music_req(cfg)

    control_n1(cfg)
    control_n1_directed()
    control_n2(cfg)
    control_n3(cfg)
    control_n4(cfg)

    print("")
    if FAILURES:
        print("FAILED %d of %d checks:" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("ALL %d CHECKS PASSED" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
