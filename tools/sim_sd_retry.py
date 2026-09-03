#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle-accurate model of the SD read stack, built to prove the fix2 retry path.

Why this tool exists
--------------------
The board played only two of the four BMPs. The chain, confirmed by reading the
code and by git history:

  1. cccee9d added a ~100ms read-token timeout to sd_card_cmd.v plus error
     forwarding in sd_card_sec_read_write.v, because some cards never drive the
     0xFE data start token at 25MHz SPI.
  2. ca37a8a raised SPI back to 25MHz, and its own commit message says the
     reason that was safe is the 100ms timeout.
  3. 15b93d8, titled "docs", deleted the timeout and the error forwarding and
     hard coded spi_clk_div <= 16'd0, so the safety net vanished while the
     25MHz that depended on it stayed.

With no timeout, one missed token parks sd_card_cmd in S_READ_WAIT forever.
That parks sd_card_sec_read_write in S_READ, which never pulses
sd_sec_read_end, so bmp_read never leaves ST_LOAD_DATA. The one second
load_stall_cnt in sd_card_bmp.v then aborts the load -- but the abort only
resets bmp_read. The SPI layer underneath is still stuck, so every later load
dies the same way, next_load_idx has already advanced past each of them, and
img_loaded_count freezes at whatever loaded before the first miss. Two here.

What is being verified
----------------------
The fix restores the timeout and adds something cccee9d did not have: a retry of
the same sector. Retry is the right recovery because a missed start token means
not one payload byte was delivered, so re-issuing CMD17 loses nothing and
duplicates nothing. Skipping, which is what cccee9d did, loses 512 bytes and
shifts every later pixel, and loses the whole image if it happens during
ST_LOAD_HDR, where a headerless sector fails header_match_r.

The interesting risk is a handshake race, and it is the reason this model is
cycle accurate rather than transaction level. When sd_card_cmd times out it
walks S_ERR -> S_END -> S_WAIT. In S_WAIT it tests cmd_req first and
block_read_req second. sd_card_sec_read_write must therefore drop
block_read_req in the very cycle it observes cmd_req_error, because one cycle
later it has moved to S_CMD17 where cmd_req is still low -- and if
block_read_req is still high at that moment, sd_card_cmd re-enters S_READ_WAIT
and burns a whole second timeout that nobody asked for. Pass G checks this, and
checks that the check has teeth by also running a variant with the drop removed.

Honest limits of this model
---------------------------
* MISO is modelled one byte per SPI transaction, not one bit per clock edge.
  That is exactly the granularity sd_card_cmd observes, since it only samples
  data_recv when spi_wr_ack is high, so nothing is lost -- but this model cannot
  tell you anything about bit level signal integrity, which is the physical
  cause of a missed token in the first place.
* The 100ms timeout is used verbatim in the healthy passes, where it never
  fires, so those passes measure the real margin. The miss passes scale it down,
  because simulating 10_000_000 cycles per attempt in Python is not practical.
  Pass H runs the same miss scenario at two different scaled values to show the
  retry behaviour does not depend on the magnitude, only on the counter crossing
  it. The wall clock budget is then computed analytically in Pass I from the
  cycles-per-byte that Pass A measures, not from the scaled run.
* The card model is a protocol model, not a device model. It never does anything
  a real card would not do under the SD SPI spec, but it also has no internal
  timing of its own beyond the token delay you give it.

Usage
-----
    python tools/sim_sd_retry.py
    python tools/sim_sd_retry.py --verbose
"""

import argparse
import sys

CLK_HZ = 100_000_000              # sd_card_clk, the sys_clk of this whole stack
REAL_TIMEOUT_MAX = 10_000_000     # READ_TIMEOUT_MAX in sd_card_cmd.v, 100ms
SECTOR_BYTES = 512
RD_RETRY_MAX = 2                  # must match sd_card_sec_read_write.v
STALL_WATCHDOG_CYCLES = CLK_HZ    # load_stall_cnt in sd_card_bmp.v, 1 second

CPOL = 1
CPHA = 1


# --------------------------------------------------------------------------
# spi_master.v
# --------------------------------------------------------------------------
class SpiMaster(object):
    IDLE, DCLK_EDGE, DCLK_IDLE, ACK, LAST_HALF_CYCLE, ACK_WAIT = 0, 1, 2, 3, 4, 5
    NAMES = {0: "IDLE", 1: "DCLK_EDGE", 2: "DCLK_IDLE", 3: "ACK",
             4: "LAST_HALF_CYCLE", 5: "ACK_WAIT"}

    def __init__(self):
        self.state = self.IDLE
        self.clk_cnt = 0
        self.clk_edge_cnt = 0
        self.dclk = 0
        self.mosi_shift = 0
        self.data_out = 0          # stands in for MISO_shift, one byte per transaction

    @property
    def wr_ack(self):
        return self.state == self.ACK

    def _next_state(self, clk_div, wr_req):
        s = self.state
        if s == self.IDLE:
            return self.DCLK_IDLE if wr_req else self.IDLE
        if s == self.DCLK_IDLE:
            return self.DCLK_EDGE if self.clk_cnt == clk_div else self.DCLK_IDLE
        if s == self.DCLK_EDGE:
            return self.LAST_HALF_CYCLE if self.clk_edge_cnt == 15 else self.DCLK_IDLE
        if s == self.LAST_HALF_CYCLE:
            return self.ACK if self.clk_cnt == clk_div else self.LAST_HALF_CYCLE
        if s == self.ACK:
            return self.ACK_WAIT
        if s == self.ACK_WAIT:
            return self.IDLE
        return self.IDLE

    def step(self, clk_div, wr_req, data_in, card_byte):
        """Advance one sys_clk. Returns the MOSI byte if a transaction started."""
        started = None
        ns = self._next_state(clk_div, wr_req)

        if self.state == self.IDLE:
            self.dclk = CPOL
        elif self.state == self.DCLK_EDGE:
            self.dclk = 1 - self.dclk

        if self.state in (self.DCLK_IDLE, self.LAST_HALF_CYCLE):
            self.clk_cnt += 1
        else:
            self.clk_cnt = 0

        if self.state == self.DCLK_EDGE:
            self.clk_edge_cnt = (self.clk_edge_cnt + 1) & 0x1F
        elif self.state == self.IDLE:
            self.clk_edge_cnt = 0

        if self.state == self.IDLE and wr_req:
            self.mosi_shift = data_in
            # Byte granularity: the card drives one constant byte for the whole
            # transaction, and sd_card_cmd only looks at data_recv on wr_ack.
            self.data_out = card_byte & 0xFF
            started = data_in & 0xFF

        self.state = ns
        return started


def cycles_per_byte(clk_div):
    """Ack-to-ack period of one SPI byte transaction, counted off spi_master.v.

        IDLE              1
        16 x DCLK_IDLE    16 * (div + 1)      clk_cnt runs 0..div then leaves
        16 x DCLK_EDGE    16 * 1
        LAST_HALF_CYCLE   div + 1
        ACK               1
        ACK_WAIT          1
        -------------------------------
                          17 * div + 36

    The 16 DCLK_EDGE cycles are the only ones that toggle DCLK, so SCK is
    100MHz * 8 / (16 * (div + 2)) = clk / ((div + 2) * 2), which is the formula
    the module header documents. The four cycles of IDLE/LAST_HALF/ACK/ACK_WAIT
    are per byte overhead that the formula does not mention, which is why a byte
    costs 36 cycles at div=0 rather than the 32 that 25MHz would suggest.
    """
    return 17 * clk_div + 36


def measure_periods(ack_log):
    """Smallest ack-to-ack delta per clk_div, i.e. the contiguous stream period.

    Larger deltas are the gaps where sd_card_cmd sits in S_WAIT with spi_wr_req
    low, so taking the minimum isolates the real byte period instead of the
    protocol overhead around it.
    """
    grouped = {}
    for i in range(1, len(ack_log)):
        (c0, d0), (c1, d1) = ack_log[i - 1], ack_log[i]
        if d0 != d1:
            continue                     # divider changed mid gap, not comparable
        grouped.setdefault(d0, []).append(c1 - c0)
    return {d: min(v) for d, v in grouped.items()}, \
           {d: len(v) for d, v in grouped.items()}


# --------------------------------------------------------------------------
# sd_card_cmd.v
# --------------------------------------------------------------------------
(S_IDLE, S_WAIT, S_INIT, S_CMD_PRE, S_CMD, S_CMD_DATA, S_READ_WAIT, S_READ,
 S_READ_ACK, S_WRITE_TOKEN, S_WRITE_DATA_0, S_WRITE_DATA_1, S_WRITE_CRC,
 S_WRITE_SUC, S_WRITE_BUSY, S_WRITE_ACK, S_ERR, S_END) = range(18)

CMD_NAMES = {
    S_IDLE: "S_IDLE", S_WAIT: "S_WAIT", S_INIT: "S_INIT", S_CMD_PRE: "S_CMD_PRE",
    S_CMD: "S_CMD", S_CMD_DATA: "S_CMD_DATA", S_READ_WAIT: "S_READ_WAIT",
    S_READ: "S_READ", S_READ_ACK: "S_READ_ACK", S_ERR: "S_ERR", S_END: "S_END",
}


class SdCardCmd(object):
    """use_fix=False reproduces HEAD as 15b93d8 left it: no timeout at all."""

    def __init__(self, use_fix=True, timeout_max=REAL_TIMEOUT_MAX):
        self.use_fix = use_fix
        self.timeout_max = timeout_max
        self.reset()

    def reset(self):
        self.state = S_IDLE
        self.cs_reg = 1
        self.spi_wr_req = 0
        self.byte_cnt = 0
        self.clk_div = 0
        self.send_data = 0xFF
        self.cmd_req_error = 0
        self.wr_data_cnt = 0
        self.read_timeout_cnt = 0
        self.block_read_valid = 0
        self.block_read_data = 0
        # instrumentation
        self.peak_timeout_cnt = 0
        self.timeout_fires = 0
        self.read_wait_entries = 0

    @property
    def cmd_req_ack(self):
        return int(self.state == S_END)

    @property
    def block_read_req_ack(self):
        return int(self.state == S_READ_ACK)

    def step(self, spi_wr_ack, data_recv, cmd_req, cmd_r1, cmd_data_len, cmd,
             block_read_req, spi_clk_div):
        prev = self.state
        nxt = self.state
        cs = self.cs_reg
        wr_req = self.spi_wr_req
        byte_cnt = self.byte_cnt
        send = self.send_data
        err = self.cmd_req_error
        rtc = self.read_timeout_cnt
        clk_div = self.clk_div

        if self.state == S_IDLE:
            nxt = S_INIT
            clk_div = spi_clk_div
            cs = 1
        elif self.state == S_INIT:
            if spi_wr_ack:
                if self.byte_cnt >= 10:
                    byte_cnt = 0
                    wr_req = 0
                    nxt = S_WAIT
                else:
                    byte_cnt = self.byte_cnt + 1
                # The RTL has a second, unconditional `begin byte_cnt <= byte_cnt
                # + 1; end` here rather than an else, so on the exiting cycle it
                # overrides the zero and enters S_WAIT holding 11. Modelled as an
                # else because S_CMD_PRE re-zeroes byte_cnt before anything reads
                # it, so the two are indistinguishable on every reachable path.
            else:
                wr_req = 1
                send = 0xFF
        elif self.state == S_WAIT:
            err = 0
            self.wr_data_cnt = 0
            if cmd_req:
                nxt = S_CMD_PRE
            elif block_read_req:
                nxt = S_READ_WAIT
                if self.use_fix:
                    rtc = 0
            clk_div = spi_clk_div
        elif self.state == S_CMD_PRE:
            if spi_wr_ack:
                nxt = S_CMD
                wr_req = 0
                byte_cnt = 0
            else:
                wr_req = 1
                cs = 1
                send = 0xFF
        elif self.state == S_CMD:
            if spi_wr_ack:
                if self.byte_cnt == 0xFFFF or (data_recv != cmd_r1 and not (data_recv & 0x80)):
                    nxt = S_ERR
                    wr_req = 0
                    byte_cnt = 0
                elif data_recv == cmd_r1:
                    wr_req = 0
                    if cmd_data_len != 0:
                        nxt = S_CMD_DATA
                    else:
                        nxt = S_END
                    byte_cnt = 0
                else:
                    byte_cnt = self.byte_cnt + 1
            else:
                wr_req = 1
                cs = 0
                b = self.byte_cnt
                if b == 0:
                    send = ((cmd >> 40) & 0xFF) | 0x40
                elif b <= 5:
                    send = (cmd >> (8 * (5 - b))) & 0xFF
                else:
                    send = 0xFF
        elif self.state == S_CMD_DATA:
            if spi_wr_ack:
                if self.byte_cnt == cmd_data_len - 1:
                    nxt = S_END
                    wr_req = 0
                    byte_cnt = 0
                else:
                    byte_cnt = self.byte_cnt + 1
            else:
                wr_req = 1
                send = 0xFF
        elif self.state == S_READ_WAIT:
            if self.use_fix:
                rtc = self.read_timeout_cnt + 1
            if spi_wr_ack and data_recv == 0xFE:
                wr_req = 0
                nxt = S_READ
                byte_cnt = 0
                if self.use_fix:
                    rtc = 0
            elif self.use_fix and self.read_timeout_cnt > self.timeout_max:
                self.timeout_fires += 1
                nxt = S_ERR
                wr_req = 0
                rtc = 0
            else:
                wr_req = 1
                send = 0xFF
        elif self.state == S_READ:
            if self.use_fix:
                rtc = self.read_timeout_cnt + 1
            if spi_wr_ack:
                if self.byte_cnt == 513:
                    nxt = S_READ_ACK
                    wr_req = 0
                    byte_cnt = 0
                    if self.use_fix:
                        rtc = 0
                else:
                    byte_cnt = self.byte_cnt + 1
            elif self.use_fix and self.read_timeout_cnt > self.timeout_max:
                self.timeout_fires += 1
                nxt = S_ERR
                wr_req = 0
                rtc = 0
            else:
                wr_req = 1
                send = 0xFF
        elif self.state == S_ERR:
            nxt = S_END
            err = 1
        elif self.state in (S_READ_ACK, S_WRITE_ACK, S_END):
            nxt = S_WAIT
        else:
            nxt = S_IDLE

        # block_read_valid / block_read_data, separate always blocks
        if prev == S_READ and self.byte_cnt < SECTOR_BYTES:
            new_valid = int(spi_wr_ack)
        else:
            new_valid = 0
        new_data = data_recv if (prev == S_READ and spi_wr_ack) else self.block_read_data

        if self.use_fix:
            self.peak_timeout_cnt = max(self.peak_timeout_cnt, rtc)
        if nxt == S_READ_WAIT and prev != S_READ_WAIT:
            self.read_wait_entries += 1

        self.state = nxt
        self.cs_reg = cs
        self.spi_wr_req = wr_req
        self.byte_cnt = byte_cnt & 0xFFFF
        self.send_data = send & 0xFF
        self.cmd_req_error = err
        self.read_timeout_cnt = rtc & 0xFFFFFF
        self.clk_div = clk_div & 0xFFFF
        self.block_read_valid = new_valid
        self.block_read_data = new_data & 0xFF


# --------------------------------------------------------------------------
# sd_card_sec_read_write.v
# --------------------------------------------------------------------------
(W_IDLE, W_CMD0, W_CMD8, W_CMD55, W_CMD41, W_CMD16, W_CMD17, W_READ, W_CMD24,
 W_WRITE, W_ERR, W_WRITE_END, W_READ_END, W_WAIT_RW, W_RETRY_GAP) = range(15)

W_NAMES = {
    W_IDLE: "S_IDLE", W_CMD0: "S_CMD0", W_CMD8: "S_CMD8", W_CMD55: "S_CMD55",
    W_CMD41: "S_CMD41", W_CMD16: "S_CMD16", W_CMD17: "S_CMD17", W_READ: "S_READ",
    W_CMD24: "S_CMD24", W_WRITE: "S_WRITE", W_READ_END: "S_READ_END",
    W_WAIT_RW: "S_WAIT_READ_WRITE", W_RETRY_GAP: "S_RETRY_GAP",
}

# Mirrors sd_card_sec_read_write.v. The RTL counter saturates and the state exits
# on its top bit, so the gap is exactly 2^(GAP_AW-1) sys_clk.
GAP_AW = 14
GAP_CYCLES = 1 << (GAP_AW - 1)


class SecReadWrite(object):
    """use_fix=False is HEAD: no error forwarding, spi_clk_div hard coded to 0.

    drop_req_on_error=False is the deliberately broken variant used by Pass G to
    prove the race check can actually fail.

    use_gap=False drops the S_RETRY_GAP drain state, which is the variant that
    re-issues CMD17 straight away. It still recovers, because CS does go high for
    the tail of the byte that was in flight, but no complete SPI transaction runs
    with CS high -- the deselect is about 6 SCK periods instead of a full byte,
    under the 8 clocks the SD spec allows the card to keep driving the line.
    """

    def __init__(self, use_fix=True, drop_req_on_error=True,
                 low_div=248, high_div=0, use_gap=True):
        self.use_fix = use_fix
        self.drop_req_on_error = drop_req_on_error
        self.low_div = low_div
        self.high_div = high_div
        self.use_gap = use_gap
        self.reset()

    def reset(self):
        self.state = W_IDLE
        self.cmd_req = 0
        self.cmd_data_len = 0
        self.cmd_r1 = 0
        self.cmd = 0
        self.spi_clk_div = self.low_div
        self.block_write_req = 0
        self.block_read_req = 0
        self.sec_addr = 0
        self.sd_init_done = 0
        self.rd_retry = 0
        self.rd_gap = 0
        # instrumentation
        self.retry_events = 0
        self.skip_events = 0
        self.gap_entries = 0

    @property
    def sd_sec_read_end(self):
        return int(self.state == W_READ_END)

    def sd_sec_read_data_valid(self, cmd):
        return int(self.state == W_READ and cmd.block_read_valid)

    def step(self, cmd_req_ack, cmd_req_error, block_read_req_ack,
             sd_sec_read, sd_sec_read_addr):
        nxt = self.state
        cmd_req = self.cmd_req
        cmd = self.cmd
        cmd_r1 = self.cmd_r1
        cmd_data_len = self.cmd_data_len
        read_req = self.block_read_req
        div = self.spi_clk_div
        retry = self.rd_retry
        gap = self.rd_gap
        init_done = self.sd_init_done

        def arm_cmd17():
            nonlocal cmd_req, cmd, cmd_r1, cmd_data_len
            cmd_req = 1
            cmd_data_len = 0
            cmd_r1 = 0x00
            cmd = (17 << 40) | ((self.sec_addr & 0xFFFFFFFF) << 8) | 0xFF

        if self.state == W_IDLE:
            nxt = W_CMD0
            init_done = 0
            div = self.low_div
        elif self.state == W_CMD0:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_CMD8
                cmd_req = 0
            else:
                cmd_req = 1
                cmd_data_len = 0
                cmd_r1 = 0x01
                cmd = (0 << 40) | 0x95
        elif self.state == W_CMD8:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_CMD55
                cmd_req = 0
            else:
                cmd_req = 1
                cmd_data_len = 4
                cmd_r1 = 0x01
                cmd = (8 << 40) | (0x000001AA << 8) | 0x87
        elif self.state == W_CMD55:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_CMD41
                cmd_req = 0
            else:
                cmd_req = 1
                cmd_data_len = 0
                cmd_r1 = 0x01
                cmd = (55 << 40) | 0xFF
        elif self.state == W_CMD41:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_CMD16
                cmd_req = 0
                init_done = 1
                div = self.high_div
            else:
                cmd_req = 1
                cmd_data_len = 0
                cmd_r1 = 0x00
                cmd = (41 << 40) | (0x40000000 << 8) | 0xFF
        elif self.state == W_CMD16:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_WAIT_RW
                cmd_req = 0
                init_done = 1
                div = self.high_div
            else:
                cmd_req = 1
                cmd_data_len = 0
                cmd_r1 = 0x00
                cmd = (16 << 40) | (512 << 8) | 0xFF
        elif self.state == W_WAIT_RW:
            if sd_sec_read:
                nxt = W_CMD17
                self.sec_addr = sd_sec_read_addr
                if self.use_fix:
                    retry = RD_RETRY_MAX
            # The line this whole fix turns on: the parameter, never a literal.
            div = self.high_div if self.use_fix else 0
        elif self.state == W_CMD17:
            if cmd_req_ack and not cmd_req_error:
                nxt = W_READ
                cmd_req = 0
            elif self.use_fix and cmd_req_ack:
                cmd_req = 0
                read_req = 0
                if retry != 0:
                    retry -= 1
                    self.retry_events += 1
                    if self.use_gap:
                        gap = 0
                        self.gap_entries += 1
                        nxt = W_RETRY_GAP
                else:
                    nxt = W_READ_END
                    self.skip_events += 1
            else:
                arm_cmd17()
        elif self.state == W_READ:
            if block_read_req_ack:
                nxt = W_READ_END
                read_req = 0
            elif self.use_fix and cmd_req_error:
                if self.drop_req_on_error:
                    read_req = 0
                if retry != 0:
                    retry -= 1
                    self.retry_events += 1
                    if self.use_gap:
                        gap = 0
                        self.gap_entries += 1
                        nxt = W_RETRY_GAP
                    else:
                        nxt = W_CMD17
                else:
                    nxt = W_READ_END
                    self.skip_events += 1
            else:
                read_req = 1
        elif self.state == W_RETRY_GAP:
            cmd_req = 0
            read_req = 0
            if gap & (1 << (GAP_AW - 1)):
                nxt = W_CMD17
            else:
                gap += 1
        elif self.state in (W_WRITE_END, W_READ_END):
            nxt = W_WAIT_RW
        else:
            nxt = W_IDLE

        self.state = nxt
        self.cmd_req = cmd_req
        self.cmd = cmd
        self.cmd_r1 = cmd_r1
        self.cmd_data_len = cmd_data_len
        self.block_read_req = read_req
        self.spi_clk_div = div & 0xFFFF
        self.rd_retry = retry
        self.rd_gap = gap
        self.sd_init_done = init_done


# --------------------------------------------------------------------------
# card
# --------------------------------------------------------------------------
class Card(object):
    """Protocol model of an SDHC card in SPI mode.

    miss_attempts holds 1-based CMD17 attempt numbers that never produce the
    0xFE start token, which is the physical failure cccee9d described.

    The card leaves a stalled transfer the way a real one does: CS going high
    terminates it. sd_card_cmd raises CS in S_CMD_PRE before every command, so a
    re-issued CMD17 re-frames the bus cleanly and the card is back in its
    command state to accept it. Without modelling that, the retry looks broken
    in simulation even though it is legal on the wire -- which is exactly the
    mistake this model made on its first run.
    """

    def __init__(self, sectors, token_delay=3, miss_attempts=()):
        self.sectors = sectors
        self.token_delay = token_delay
        self.miss_attempts = set(miss_attempts)
        self.mode = "COLLECT"
        self.cmd_buf = []
        self.txq = []
        self.after_txq = "COLLECT"
        self.cmd17_attempt = 0
        self.sector_arg = 0
        self.byte_idx = 0
        self.ff_count = 0
        self.cmd_log = []
        self.tokens_sent = 0
        self.tokens_withheld = 0
        self.cs_aborts = 0
        self.cs_midbyte_aborts = 0
        self.cs_high_txns = 0
        self.prev_cs_low = None

    def _abort(self):
        """Drop whatever transfer is in flight and go back to taking commands."""
        self.mode = "COLLECT"
        self.cmd_buf = []
        self.txq = []
        self.byte_idx = 0

    def note_cs(self, cs_low):
        """CS is asynchronous to SCK, so a real card sees it deassert in the middle
        of a byte as well and aborts on the spot. byte() is only consulted when a
        transaction STARTS and therefore cannot see a deselect that lands inside
        an in-flight byte -- which is how the first run of this model reported "no
        deselect" for a CS pulse that was genuinely on the wire. Edge triggered so
        one long deselect counts once.
        """
        if self.prev_cs_low is not None and self.prev_cs_low and not cs_low:
            if self.mode != "COLLECT":
                self._abort()
                self.cs_midbyte_aborts += 1
        self.prev_cs_low = cs_low

    def _dispatch(self, idx, arg):
        self.cmd_log.append((idx, arg))
        r1 = 0x01 if idx in (0, 8, 55) else 0x00
        self.txq = [r1]
        if idx == 8:
            self.txq += [0x00, 0x00, 0x01, 0xAA]
        self.mode = "TXQ"
        if idx == 17:
            self.cmd17_attempt += 1
            self.sector_arg = arg
            self.after_txq = "TOKEN"
        else:
            self.after_txq = "COLLECT"

    def byte(self, mosi, cs_low):
        if not cs_low:
            # A whole transaction that starts with CS deasserted. Counting these is
            # what makes "the deselect was a full byte" checkable: it stays zero
            # unless S_CMD_PRE really transmits, and S_CMD_PRE only transmits when
            # spi_master is idle on entry, which is what S_RETRY_GAP guarantees.
            self.cs_high_txns += 1
        if not cs_low and self.mode != "COLLECT":
            # CS deasserted at a transaction boundary: the block is over, whether or
            # not the host ever saw it. This is what makes the retry possible.
            self._abort()
            self.cs_aborts += 1
            return 0xFF
        if self.mode == "COLLECT":
            if cs_low:
                if not self.cmd_buf:
                    # A command frame starts 0b01xxxxxx. Ignoring anything else is
                    # what keeps the host's 0xFF polling bytes from being parsed
                    # as commands once the card is back in this state.
                    if (mosi & 0xC0) == 0x40:
                        self.cmd_buf.append(mosi & 0xFF)
                else:
                    self.cmd_buf.append(mosi & 0xFF)
                    if len(self.cmd_buf) == 6:
                        idx = self.cmd_buf[0] & 0x3F
                        arg = int.from_bytes(bytes(self.cmd_buf[1:5]), "big")
                        self.cmd_buf = []
                        self._dispatch(idx, arg)
            else:
                self.cmd_buf = []          # CS high drops a partial frame
            return 0xFF
        if self.mode == "TXQ":
            out = self.txq.pop(0)
            if not self.txq:
                self.mode = self.after_txq
                self.ff_count = 0
            return out
        if self.mode == "TOKEN":
            if self.cmd17_attempt in self.miss_attempts:
                self.ff_count += 1
                if self.ff_count == self.token_delay + 1:
                    self.tokens_withheld += 1
                return 0xFF
            if self.ff_count >= self.token_delay:
                self.mode = "DATA"
                self.byte_idx = 0
                self.ff_count = 0
                self.tokens_sent += 1
                return 0xFE
            self.ff_count += 1
            return 0xFF
        if self.mode == "DATA":
            if self.byte_idx < SECTOR_BYTES:
                out = self.sectors[self.sector_arg][self.byte_idx]
                self.byte_idx += 1
                return out
            self.byte_idx += 1
            if self.byte_idx >= SECTOR_BYTES + 2:
                self.mode = "COLLECT"
                self.cmd_buf = []
                self.byte_idx = 0
            return 0xAA          # CRC, dont care
        return 0xFF


# --------------------------------------------------------------------------
# system
# --------------------------------------------------------------------------
class System(object):
    def __init__(self, sectors, use_fix=True, timeout_max=REAL_TIMEOUT_MAX,
                 token_delay=3, miss_attempts=(), high_div=0,
                 drop_req_on_error=True, use_gap=True):
        self.spi = SpiMaster()
        self.cmd = SdCardCmd(use_fix=use_fix, timeout_max=timeout_max)
        self.srw = SecReadWrite(use_fix=use_fix,
                                drop_req_on_error=drop_req_on_error,
                                high_div=high_div, use_gap=use_gap)
        self.card = Card(sectors, token_delay=token_delay,
                         miss_attempts=miss_attempts)
        self.cycle = 0
        self.error_events = 0
        self.prev_error = 0
        self.bytes_seen = []
        self.ends = 0
        self.trace = []
        self.ack_log = []
        # Snapshot so the deselect can be measured as a delta. cs_high_txns counts
        # every byte S_CMD_PRE transmits, and S_CMD_PRE runs in front of EVERY
        # command including the whole init sequence, so the raw total says nothing
        # about the one deselect that the retry depends on.
        self.cs_high_at_first_error = None

    def run(self, limit, addr=None, nsect=1, verbose=False):
        """Read nsect consecutive sectors from addr. True if all of them ended.

        The harness drives sd_sec_read the way bmp_read's ST_LOAD_DATA does: hold
        it high with the current address, drop it on the cycle sd_sec_read_end
        fires, re-assert one cycle later once sd_card_sec_read_write is back in
        S_WAIT_READ_WRITE. That one cycle gap is part of the measured per-sector
        cost, so it has to be modelled rather than optimised away.
        """
        sd_sec_read = 0
        sd_sec_read_addr = addr if addr is not None else 0
        pending = 0 if addr is None else nsect

        while self.cycle < limit:
            # ---- combinational, from current registers ----
            wr_ack = int(self.spi.wr_ack)
            data_recv = self.spi.data_out
            if wr_ack:
                self.ack_log.append((self.cycle, self.cmd.clk_div))
            cmd_req_ack = self.cmd.cmd_req_ack
            cmd_req_error = self.cmd.cmd_req_error
            block_read_req_ack = self.cmd.block_read_req_ack
            cs_low = not self.cmd.cs_reg
            # Every cycle, not just at transaction boundaries: a card reacts to CS
            # as soon as it moves, and the interesting deselect in this design is
            # one that lands while a byte is still shifting.
            self.card.note_cs(cs_low)

            if cmd_req_error and not self.prev_error:
                self.error_events += 1
                if self.cs_high_at_first_error is None:
                    self.cs_high_at_first_error = self.card.cs_high_txns
            self.prev_error = cmd_req_error

            # The card drives the byte for a transaction starting this cycle. It
            # is only consulted when one actually starts, so its state advances
            # once per SPI byte and never during an idle gap.
            card_byte = self.card.byte(self.cmd.send_data, cs_low) \
                if (self.spi.state == SpiMaster.IDLE and self.cmd.spi_wr_req) \
                else self.spi.data_out

            if self.srw.sd_sec_read_data_valid(self.cmd):
                self.bytes_seen.append(self.cmd.block_read_data)

            if self.srw.sd_sec_read_end:
                self.ends += 1
                sd_sec_read = 0
                pending -= 1

            if verbose:
                # cs and the spi state are in the trace because the retry's
                # legality on the wire depends on both: sd_card_cmd only raises CS
                # in S_CMD_PRE's else branch, so if a stale wr_ack from the byte
                # that was in flight when the timeout fired shortcuts S_CMD_PRE on
                # its first cycle, the CS deselect never happens and the card is
                # never re-framed. That cannot be seen without these two columns.
                self.trace.append((self.cycle, W_NAMES.get(self.srw.state, "?"),
                                   CMD_NAMES.get(self.cmd.state, "?"),
                                   self.srw.rd_retry, cmd_req_error,
                                   self.cmd.cs_reg,
                                   SpiMaster.NAMES.get(self.spi.state, "?"),
                                   self.cmd.spi_wr_req, int(self.spi.wr_ack),
                                   self.card.mode))

            # ---- harness: issue the next sector like bmp_read would ----
            if pending > 0 and not sd_sec_read and self.srw.sd_init_done \
                    and self.srw.state == W_WAIT_RW:
                sd_sec_read_addr = addr + (nsect - pending)
                sd_sec_read = 1

            # ---- registered updates ----
            # Snapshot sd_card_sec_read_write's outputs BEFORE stepping it. Both
            # modules are clocked by the same edge, so sd_card_cmd must see the
            # values that were on the wire during this cycle, not the ones
            # sd_card_sec_read_write computes at its end. Calling srw.step first
            # and then reading srw.block_read_req makes that path zero delay and
            # hides exactly the race this model exists to check: whether
            # block_read_req is still asserted when sd_card_cmd lands in S_WAIT.
            srw_cmd_req = self.srw.cmd_req
            srw_cmd_r1 = self.srw.cmd_r1
            srw_cmd_data_len = self.srw.cmd_data_len
            srw_cmd = self.srw.cmd
            srw_block_read_req = self.srw.block_read_req
            srw_spi_clk_div = self.srw.spi_clk_div
            self.spi.step(self.cmd.clk_div, self.cmd.spi_wr_req,
                          self.cmd.send_data, card_byte)
            self.srw.step(cmd_req_ack, cmd_req_error, block_read_req_ack,
                          sd_sec_read, sd_sec_read_addr)
            self.cmd.step(wr_ack, data_recv, srw_cmd_req, srw_cmd_r1,
                          srw_cmd_data_len, srw_cmd,
                          srw_block_read_req, srw_spi_clk_div)
            self.cycle += 1

            if addr is not None and pending <= 0 and self.ends >= nsect:
                return True
        return False


def make_sectors(n=8):
    """Deterministic, self identifying sector contents."""
    out = {}
    for i in range(n):
        addr = 100 + i
        out[addr] = bytes(((addr * 7 + j * 13) & 0xFF) for j in range(SECTOR_BYTES))
    return out


class Result(object):
    def __init__(self):
        self.rows = []
        self.failed = 0

    def add(self, ok, name, detail):
        self.rows.append((ok, name, detail))
        if not ok:
            self.failed += 1
        print("  [%s] %-30s %s" % ("PASS" if ok else "FAIL", name, detail))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    res = Result()
    sectors = make_sectors()

    print("=" * 78)
    print("A. byte period and per sector cost, measured out of the model")
    print("=" * 78)
    # spi_clk_div starts at SPI_LOW_SPEED_DIV = 248 and only switches to the high
    # divider at CMD41, so a single healthy read exercises both. Two runs measure
    # all three dividers of interest without paying for a 2.2M cycle low speed
    # sector read that would tell us nothing new.
    measured_period = {}
    sector_cycles = {}
    for high_div, label in ((0, "25MHz, what ships"),
                            (4, "8.3MHz, the cccee9d fallback")):
        limit = 4_000_000 if high_div == 0 else 12_000_000
        one = System(sectors, high_div=high_div, timeout_max=REAL_TIMEOUT_MAX)
        ok1 = one.run(limit, addr=100, nsect=1)
        five = System(sectors, high_div=high_div, timeout_max=REAL_TIMEOUT_MAX)
        ok5 = five.run(limit * 3, addr=100, nsect=5)
        per, counts = measure_periods(one.ack_log)
        measured_period.update(per)
        res.add(ok1 and ok5 and five.ends == 5,
                "healthy stream, %s" % label,
                "1 sector in %d cycles, 5 in %d, %d read_end pulses, %d acks logged"
                % (one.cycle, five.cycle, five.ends, len(one.ack_log)))
        if ok1 and ok5:
            # Differencing removes the whole init sequence, leaving the steady
            # state cost of one sector: CMD17 framing, token wait, 514 bytes, and
            # the one cycle gap bmp_read leaves between sectors.
            sector_cycles[high_div] = (five.cycle - one.cycle) / 4.0

    for div in sorted(measured_period):
        got = measured_period[div]
        want = cycles_per_byte(div)
        sck = CLK_HZ / (2.0 * (div + 2))
        res.add(got == want, "div=%d byte period" % div,
                "measured %d cycles between acks, %d counted off spi_master.v, "
                "SCK %.3f MHz" % (got, want, sck / 1e6))

    for div in sorted(sector_cycles):
        meas = sector_cycles[div]
        payload = 514 * cycles_per_byte(div)
        res.add(meas > payload, "div=%d per sector cost" % div,
                "measured %.0f cycles (%.2f us) against %.0f for the 514 byte "
                "payload alone; the %.0f cycle difference is CMD17 framing plus "
                "the inter sector gap" % (meas, meas * 1e6 / CLK_HZ, payload,
                                          meas - payload))

    print("")
    print("=" * 78)
    print("B. healthy read: byte exactness and the real 100ms timeout margin")
    print("=" * 78)
    for div, label in ((0, "25MHz"), (4, "8.3MHz")):
        sysm = System(sectors, high_div=div, timeout_max=REAL_TIMEOUT_MAX)
        limit = 4_000_000 if div == 0 else 12_000_000
        ok = sysm.run(limit, addr=100)
        got = bytes(sysm.bytes_seen)
        res.add(ok and got == sectors[100] and sysm.ends == 1,
                "payload exact, %s" % label,
                "%d bytes delivered, %d read_end pulse(s), byte for byte match=%s"
                % (len(got), sysm.ends, got == sectors[100]))
        peak = max(1, sysm.cmd.peak_timeout_cnt)
        res.add(sysm.cmd.timeout_fires == 0, "timeout silent, %s" % label,
                "%d fires; peak read_timeout_cnt %d against READ_TIMEOUT_MAX %d "
                "= %.0fx margin, so a healthy read cannot trip it"
                % (sysm.cmd.timeout_fires, peak, REAL_TIMEOUT_MAX,
                   REAL_TIMEOUT_MAX / peak))
        res.add(sysm.srw.spi_clk_div == div, "divider knob is live, %s" % label,
                "spi_clk_div settled at %d, which is SPI_HIGH_SPEED_DIV -- the "
                "hard coded 16'd0 that 15b93d8 left behind would read 0 here "
                "whatever the parameter said" % sysm.srw.spi_clk_div)

    print("")
    print("=" * 78)
    print("C. one missed start token, then a good retry")
    print("=" * 78)
    # Scaled timeout: it has to stay above a healthy S_READ at div=0, which is
    # 514 byte periods, or the retry that follows the miss would trip it too.
    healthy_sread = 514 * cycles_per_byte(0)
    scaled = 40_000
    res.add(scaled > healthy_sread, "scaled timeout is safe",
            "scaled %d > a healthy S_READ of %d cycles at div=0, so the retry "
            "cannot trip it. The real 10_000_000 is used verbatim in passes A "
            "and B, where it never fires, and that is where the margin is "
            "measured." % (scaled, healthy_sread))
    sysm = System(sectors, timeout_max=scaled, miss_attempts=(1,))
    ok = sysm.run(2_000_000, addr=100)
    got = bytes(sysm.bytes_seen)
    res.add(ok, "read completed", "%d cycles, %d read_end pulse(s)" % (sysm.cycle, sysm.ends))
    res.add(got == sectors[100], "payload exact after retry",
            "%d bytes, match=%s -- a missed token delivers nothing, so the "
            "retry neither loses nor duplicates" % (len(got), got == sectors[100]))
    res.add(sysm.error_events == 1 and sysm.srw.retry_events == 1
            and sysm.srw.skip_events == 0,
            "exactly one error, one retry",
            "%d cmd_req_error event(s), %d retry(ies), %d skip(s)"
            % (sysm.error_events, sysm.srw.retry_events, sysm.srw.skip_events))
    res.add(sysm.cmd.read_wait_entries == sysm.card.cmd17_attempt,
            "one S_READ_WAIT per CMD17",
            "%d S_READ_WAIT entries for %d CMD17(s) the card accepted -- a retry "
            "legitimately enters once per attempt, so the invariant is one entry "
            "per command, not one entry total. An entry with no matching CMD17 "
            "means block_read_req was still asserted when sd_card_cmd reached "
            "S_WAIT, so it re-armed itself and burned a timeout nobody asked for"
            % (sysm.cmd.read_wait_entries, sysm.card.cmd17_attempt))
    reframe = sysm.card.cs_aborts + sysm.card.cs_midbyte_aborts
    res.add(reframe >= 1 and sysm.card.tokens_withheld == 1,
            "card re-framed by the CS pulse",
            "%d token(s) withheld, %d abort(s) (%d at a byte boundary, %d inside a "
            "byte): CS went high before the retry, which is what puts the card back "
            "in its command state so the re-issued CMD17 is accepted rather than "
            "shouted into a stalled transfer"
            % (sysm.card.tokens_withheld, reframe, sysm.card.cs_aborts,
               sysm.card.cs_midbyte_aborts))
    deselects = sysm.card.cs_high_txns - (sysm.cs_high_at_first_error or 0)
    res.add(deselects >= 1, "deselect is a full byte",
            "%d complete SPI transaction(s) ran with CS high between the timeout "
            "and the end of the read, and S_RETRY_GAP was entered %d time(s). The "
            "SD spec gives a card 8 clocks to release the line after deselect and "
            "a transaction is 8 clocks, so this is the difference between an "
            "unambiguous deselect and a marginal one"
            % (deselects, sysm.srw.gap_entries))
    # That check has to be able to fail, or it proves nothing. Drop the drain state
    # and the retry STILL completes -- CS does go high, just not for a whole byte,
    # because S_CMD_PRE consumes the ack of the byte that was in flight when the
    # timeout fired and never transmits one of its own. So this separates
    # "recovered" from "recovered the way the spec wants", which is the only thing
    # the drain state buys.
    sysm_ng = System(sectors, timeout_max=scaled, miss_attempts=(1,),
                     use_gap=False)
    ok_ng = sysm_ng.run(2_000_000, addr=100)
    ng_deselects = sysm_ng.card.cs_high_txns - (sysm_ng.cs_high_at_first_error or 0)
    res.add(ok_ng and ng_deselects == 0
            and sysm_ng.srw.gap_entries == 0
            and sysm_ng.card.cs_midbyte_aborts >= 1,
            "no-drain variant is caught",
            "without S_RETRY_GAP the retry still finished (%s) on %d in-byte CS "
            "abort(s), but %d full-byte deselect(s) followed the timeout against "
            "%d with the drain -- the marginal one is 6 SCK periods of a leftover "
            "byte tail, under the 8 the spec allows"
            % (ok_ng, sysm_ng.card.cs_midbyte_aborts, ng_deselects, deselects))
    # Pass F compares against these, so keep the reference numbers from the build
    # that actually shipped rather than restating them as literals there.
    ref_errors, ref_entries = sysm.error_events, sysm.cmd.read_wait_entries

    print("")
    print("=" * 78)
    print("D. retries exhausted: skip the sector, never hang")
    print("=" * 78)
    sysm = System(sectors, timeout_max=scaled, miss_attempts=(1, 2, 3))
    ok = sysm.run(2_000_000, addr=100)
    res.add(ok and sysm.ends == 1, "read_end still fires",
            "%d cycles, %d read_end pulse(s), %d retry(ies), %d skip(s) -- "
            "bmp_read advances instead of parking"
            % (sysm.cycle, sysm.ends, sysm.srw.retry_events, sysm.srw.skip_events))
    res.add(len(sysm.bytes_seen) == 0, "no payload from a dead sector",
            "%d bytes delivered" % len(sysm.bytes_seen))
    res.add(sysm.error_events == RD_RETRY_MAX + 1, "error count matches budget",
            "%d events for %d attempts" % (sysm.error_events, RD_RETRY_MAX + 1))

    print("")
    print("=" * 78)
    print("E. PRE-FIX model reproduces the board symptom: permanent hang")
    print("=" * 78)
    sysm = System(sectors, use_fix=False, miss_attempts=(1,))
    limit = 3_000_000
    ok = sysm.run(limit, addr=100)
    res.add(not ok, "pre-fix hangs",
            "no read_end in %d cycles (%.0f ms of sim time); sd_card_cmd parked "
            "in %s, sd_card_sec_read_write parked in %s"
            % (limit, limit * 1000.0 / CLK_HZ,
               CMD_NAMES.get(sysm.cmd.state, "?"),
               W_NAMES.get(sysm.srw.state, "?")))
    sysm2 = System(sectors, timeout_max=scaled, miss_attempts=(1,))
    ok2 = sysm2.run(limit, addr=100)
    res.add(ok2, "post-fix recovers",
            "same card, same missed token, read_end after %d cycles" % sysm2.cycle)

    print("")
    print("=" * 78)
    print("F. the race check has teeth: remove the block_read_req drop")
    print("=" * 78)
    bad = System(sectors, timeout_max=scaled, miss_attempts=(1,),
                 drop_req_on_error=False)
    bad.run(4_000_000, addr=100)
    res.add(bad.error_events > ref_errors or bad.cmd.read_wait_entries > ref_entries,
            "broken variant is caught",
            "%d error event(s) and %d S_READ_WAIT entries for ONE withheld token, "
            "against %d and %d in the fixed build -- the extra timeout is the race, "
            "and this pass would go red if the check were vacuous"
            % (bad.error_events, bad.cmd.read_wait_entries,
               ref_errors, ref_entries))

    print("")
    print("=" * 78)
    print("G. scaled timeout invariance: the handshake does not care how big it is")
    print("=" * 78)
    seen = []
    for tmax in (scaled, 60_000, 90_000):
        s = System(sectors, timeout_max=tmax, miss_attempts=(1,))
        s.run(4_000_000, addr=100)
        seen.append((s.error_events, s.srw.retry_events, s.srw.skip_events,
                     s.cmd.read_wait_entries, bytes(s.bytes_seen) == sectors[100]))
    res.add(len(set(seen)) == 1, "identical across three timeouts",
            "timeout_max in {%d, 60000, 90000} all gave %s, so the handshake "
            "depends on the counter crossing the limit and not on where the "
            "limit sits" % (scaled, seen[0]))

    print("")
    print("=" * 78)
    print("H. wall clock budget, from the real timeout and measured sector cost")
    print("=" * 78)
    timeout_ms = REAL_TIMEOUT_MAX * 1000.0 / CLK_HZ
    watchdog_ms = STALL_WATCHDOG_CYCLES * 1000.0 / CLK_HZ
    gap_ms = GAP_CYCLES * 1000.0 / CLK_HZ
    attempts = RD_RETRY_MAX + 1
    nsec = 1801                          # a 640x480 24bpp BMP on the tested card
    for div, label in ((0, "25MHz, ships"), (4, "8.3MHz, fallback")):
        cyc = sector_cycles.get(div)
        if cyc is None:
            res.add(False, "no measurement for div=%d" % div, "pass A did not run")
            continue
        sector_ms = cyc * 1000.0 / CLK_HZ
        # One drain gap before each re-issue, so there is one fewer of them than
        # there are attempts: the last attempt either reads or skips.
        worst_ms = attempts * timeout_ms + (attempts - 1) * gap_ms + sector_ms
        res.add(worst_ms < watchdog_ms,
                "one dead sector fits the watchdog, %s" % label,
                "%d attempts x %.0fms + %d drain gaps x %.3fms + %.2fms transfer "
                "= %.1fms against the %.0fms load_stall_cnt, %.2fx margin"
                % (attempts, timeout_ms, attempts - 1, gap_ms, sector_ms,
                   worst_ms, watchdog_ms, watchdog_ms / worst_ms))
        consecutive = int(watchdog_ms / worst_ms)
        res.add(consecutive >= 1,
                "back to back dead sectors, %s" % label,
                "%d sectors that burn every retry fit inside the watchdog, so "
                "an image only aborts past that -- and unlike the pre-fix build "
                "the abort is recoverable, because sd_card_cmd is back in S_WAIT "
                "instead of parked in S_READ_WAIT" % consecutive)
        clean_s = nsec * sector_ms / 1000.0
        res.add(True, "clean image load, %s" % label,
                "%d sectors x %.3fms = %.2f s per image, %.1f s for four. "
                "Measured, so it includes CMD17 framing and the inter sector gap."
                % (nsec, sector_ms, clean_s, clean_s * 4))
        # What a realistic miss rate costs, since each one burns a full timeout
        # before the retry succeeds.
        for rate_label, rate in (("1 in 10000 sectors", 1e-4),
                                 ("1 in 1000 sectors", 1e-3),
                                 ("1 in 100 sectors", 1e-2)):
            hits = nsec * rate
            extra = hits * timeout_ms / 1000.0
            res.add(clean_s + extra < 60.0, "miss rate %s, %s" % (rate_label, label),
                    "%.1f retry(ies) per image, +%.2f s, total %.2f s" %
                    (hits, extra, clean_s + extra))

    print("")
    print("=" * 78)
    if res.failed == 0:
        print("VERDICT: %d/%d checks passed." % (len(res.rows), len(res.rows)))
        print("  The retry path is byte exact, the race is closed and provably")
        print("  detectable, retries are bounded inside the existing watchdog,")
        print("  and the pre-fix model reproduces the board symptom.")
    else:
        print("VERDICT: %d of %d checks FAILED." % (res.failed, len(res.rows)))
    return 1 if res.failed else 0


if __name__ == "__main__":
    sys.exit(main())
