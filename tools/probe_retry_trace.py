#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print the cycles around a read-token timeout, to see whether CS ever goes high.

Why this probe exists
---------------------
sim_sd_retry.py pass C reports "0 CS abort(s)" after a retry, which means the
card model never saw CS deasserted. Either the model is wrong or the RTL never
produces the deselect. The two have opposite consequences: a wrong model only
needs fixing in the model, while a missing deselect means the re-issued CMD17 is
shouted at a card that is still inside the aborted read transaction, which is a
real defect that would make the retry useless on the board.

sd_card_cmd raises CS in exactly one place, the else branch of S_CMD_PRE:

    S_CMD_PRE:
        if(spi_wr_ack == 1'b1) begin
            state <= S_CMD;  spi_wr_req <= 1'b0;  byte_cnt <= 16'd0;
        end else begin
            spi_wr_req <= 1'b1;  CS_reg <= 1'b1;  send_data <= 8'hff;
        end

so CS is only raised on a cycle where spi_wr_ack is LOW. The read timeout fires
on an arbitrary cycle of S_READ_WAIT -- read_timeout_cnt counts every sys_clk
while spi_wr_ack comes only once per byte -- so when it fires there is normally a
byte still in flight inside spi_master, and that byte's ack lands a few cycles
later. S_ERR, S_END and S_WAIT are one cycle each, so S_CMD_PRE is entered while
that stale ack may still be high. If it is, S_CMD_PRE takes the first branch on
its very first cycle and never assigns cs, so the deselect is skipped.

This probe prints the cycle-by-cycle evidence for or against that.

It reuses sim_sd_retry.System.run() verbatim. Nothing about the model is
reimplemented here, because reimplementing the thing under suspicion is how you
end up proving your own assumption. The only substitution is the trace container:
run() appends one row per cycle and a 2M cycle pass would not fit in memory, so
the rows are filtered down to a window around the error as they arrive.

The post window is collapsed, not truncated
-------------------------------------------
The first version kept a fixed number of cycles after the error rise. That was
written before sd_card_sec_read_write grew S_RETRY_GAP, which idles for 2^13 =
8192 cycles before re-issuing CMD17. A fixed budget therefore has to choose
between drowning in the gap and stopping before the re-issued S_CMD_PRE -- which
is the only cycle range this probe exists to inspect. So rows after the rise are
kept selectively: every cycle where sd_card_cmd is in one of the four states the
verdict is decided from (S_CMD_PRE, S_ERR, S_END, S_WAIT), plus every cycle where
a state or the card mode changes. The 8192 cycle gap and the 514 byte payload
read each collapse to a handful of rows and the collapse count is printed.

Usage
-----
    python tools/probe_retry_trace.py
    python tools/probe_retry_trace.py --pre 24 --post 60000
    python tools/probe_retry_trace.py --no-gap
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim_sd_retry import (System, make_sectors, cycles_per_byte,
                          REAL_TIMEOUT_MAX, W_NAMES, CMD_NAMES)

# Row layout produced by System.run(verbose=True):
#   0 cycle, 1 srw state, 2 cmd state, 3 rd_retry, 4 cmd_req_error,
#   5 cs_reg, 6 spi state, 7 spi_wr_req, 8 spi_wr_ack, 9 card mode
COL = ("cycle", "sec_rw", "sd_cmd", "retry", "err", "CS", "spi", "wrq", "ack",
       "card")

# sd_card_cmd states the verdict is decided from. These are kept cycle by cycle
# no matter how long they run; everything else is kept only on a change.
DETAIL_STATES = ("S_CMD_PRE", "S_ERR", "S_END", "S_WAIT")


class ErrorWindow(list):
    """Keeps only a window of trace rows around the first cmd_req_error rise.

    run() appends to self.trace once per cycle and has no hook to stop early, so
    the filtering has to happen on append. A bounded deque holds the `pre` rows
    seen so far; the rising edge flushes it and then rows are kept selectively
    until `post` cycles have passed.
    """

    def __init__(self, pre, post, windows=1):
        super(ErrorWindow, self).__init__()
        self.pre_cap = pre
        self.post_cap = post
        self.post_left = post
        self.ring = []
        self.prev_err = 0
        self.windows_left = windows
        self.last = None
        self.pending_skip = 0
        # cycle -> how many collapsed cycles preceded it, for the printout
        self.notes = {}

    def append(self, row):
        err = row[4]
        rise = bool(err and not self.prev_err)
        self.prev_err = err
        if rise and self.windows_left > 0:
            list.extend(self, self.ring)
            list.append(self, row)
            self.ring = []
            self.windows_left -= 1
            self.post_left = self.post_cap
            self.last = row
        elif self.post_left > 0:
            # Draining the post window takes priority over refilling the pre ring,
            # otherwise the rows right after a rise land in the ring and are thrown
            # away unless a second rise happens to flush them.
            self.post_left -= 1
            if self._keep(row):
                list.append(self, row)
                if self.pending_skip:
                    self.notes[row[0]] = self.pending_skip
                    self.pending_skip = 0
                self.last = row
            else:
                self.pending_skip += 1
        elif self.windows_left > 0:
            self.ring.append(row)
            if len(self.ring) > self.pre_cap:
                del self.ring[0]

    def _keep(self, row):
        """Decide whether a post-rise row earns a line of output."""
        if row[2] in DETAIL_STATES:
            return True
        if self.last is None:
            return True
        # A state or card mode change is the boundary of something that happened;
        # the spi state changes every cycle by design and would defeat the whole
        # collapse, so it is deliberately not part of this test.
        return ((row[1], row[2]) != (self.last[1], self.last[2])
                or row[9] != self.last[9])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pre", type=int, default=30,
                    help="cycles to keep before the error rise (default 30)")
    ap.add_argument("--post", type=int, default=40_000,
                    help="cycles after the error rise to keep watching (default "
                         "40000). Needs to cover the 8192 cycle S_RETRY_GAP plus "
                         "the re-issued CMD17 and a whole 514 byte read; rows are "
                         "collapsed so a big budget does not mean a big printout")
    ap.add_argument("--timeout", type=int, default=40_000,
                    help="scaled READ_TIMEOUT_MAX so the window arrives quickly "
                         "(default 40000; the shipped value is %d)"
                         % REAL_TIMEOUT_MAX)
    ap.add_argument("--no-drop-req", action="store_true",
                    help="break the block_read_req drop in S_READ's error branch, "
                         "the variant Pass F uses to prove the race check bites")
    ap.add_argument("--no-gap", action="store_true",
                    help="drop the S_RETRY_GAP drain state, i.e. re-issue CMD17 "
                         "straight away the way the first version of the fix did")
    ap.add_argument("--windows", type=int, default=1,
                    help="how many cmd_req_error rises to capture (default 1)")
    args = ap.parse_args()

    sectors = make_sectors()
    healthy_sread = 514 * cycles_per_byte(0)
    if args.timeout <= healthy_sread:
        print("refusing: timeout %d <= a healthy S_READ of %d cycles, so the "
              "retry itself would trip it and the window would show the wrong "
              "failure" % (args.timeout, healthy_sread))
        return 2

    sysm = System(sectors, timeout_max=args.timeout, miss_attempts=(1,),
                  drop_req_on_error=not args.no_drop_req,
                  use_gap=not args.no_gap)
    sysm.trace = ErrorWindow(args.pre, args.post, windows=args.windows)
    # Generous limit: the window fills on the first error, and everything after
    # it is dropped by ErrorWindow, so memory stays flat however long this runs.
    returned = sysm.run(2_000_000, addr=100, verbose=True)
    print("run returned %s at cycle %d" % (returned, sysm.cycle))
    print("final: sec_rw=%s sd_cmd=%s card=%s cmd17_attempt=%d tokens_sent=%d "
          "tokens_withheld=%d rd_retry=%d"
          % (W_NAMES.get(sysm.srw.state, "?"), CMD_NAMES.get(sysm.cmd.state, "?"),
             sysm.card.mode, sysm.card.cmd17_attempt, sysm.card.tokens_sent,
             sysm.card.tokens_withheld, sysm.srw.rd_retry))
    print("")

    rows = list(sysm.trace)
    if not rows:
        print("no cmd_req_error rise happened at all -- nothing to show")
        return 1

    print("read token timeout window, scaled READ_TIMEOUT_MAX = %d cycles"
          % args.timeout)
    print("CS is sd_card_cmd's CS_reg: 1 = deselected, 0 = card selected")
    print("")
    print("%8s %-16s %-12s %5s %3s %2s %-14s %3s %3s %-8s" % COL)
    edge = None
    prev_err = 0
    notes = sysm.trace.notes
    for r in rows:
        if r[4] and not prev_err:
            edge = r[0]
        prev_err = r[4]
        mark = ""
        if r[0] == edge:
            mark = "   <== cmd_req_error rises"
        elif r[0] in notes:
            mark = "   (+%d collapsed cycles)" % notes[r[0]]
        print("%8d %-16s %-12s %5d %3d %2d %-14s %3d %3d %s%s"
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
                 r[9], mark))

    # ---- the verdict, decided from the rows rather than asserted ----
    print("")
    print("=" * 78)
    cmd_pre = [r for r in rows if r[2] == "S_CMD_PRE"]
    cs_high_in_pre = [r for r in cmd_pre if r[5] == 1]
    ack_high_in_pre = [r for r in cmd_pre if r[8] == 1]
    cs_ever_high_after = any(r[5] == 1 for r in rows if r[0] >= edge)

    print("S_CMD_PRE cycles in window        : %d" % len(cmd_pre))
    print("  of them with CS already high    : %d" % len(cs_high_in_pre))
    print("  of them with a stale wr_ack high: %d" % len(ack_high_in_pre))
    print("CS high at any point after the error: %s" % cs_ever_high_after)
    print("card CS aborts counted by the model : %d (%d at a byte boundary, "
          "%d inside a byte)"
          % (sysm.card.cs_aborts + sysm.card.cs_midbyte_aborts,
             sysm.card.cs_aborts, sysm.card.cs_midbyte_aborts))
    print("complete transactions with CS high after the error: %d"
          % (sysm.card.cs_high_txns - (sysm.cs_high_at_first_error or 0)))
    print("S_RETRY_GAP entries                 : %d" % sysm.srw.gap_entries)
    print("card tokens withheld                : %d" % sysm.card.tokens_withheld)
    print("cmd_req_error events                : %d" % sysm.error_events)
    print("S_READ_WAIT entries                 : %d" % sysm.cmd.read_wait_entries)
    print("CMD17 attempts the card accepted    : %d" % sysm.card.cmd17_attempt)
    print("retries / skips                     : %d / %d"
          % (sysm.srw.retry_events, sysm.srw.skip_events))
    print("read_end pulses                     : %d" % sysm.ends)
    print("")

    # Both counters describe a card that saw CS deassert and gave up on a stalled
    # transfer; they differ only in whether the edge landed on a byte boundary or
    # inside one. Checking cs_aborts alone is what made this print "inconclusive"
    # for a run that had in fact re-framed the card cleanly -- the deselect rises
    # while the byte that was in flight at the timeout is still shifting, so it
    # lands inside a byte and cs_aborts stays zero.
    aborts = sysm.card.cs_aborts + sysm.card.cs_midbyte_aborts
    deselects = sysm.card.cs_high_txns - (sysm.cs_high_at_first_error or 0)

    if sysm.error_events > sysm.card.cmd17_attempt - sysm.card.tokens_withheld:
        print("VERDICT: %d timeout(s) for %d withheld token(s), and "
              "S_READ_WAIT was entered %d time(s) for %d accepted CMD17(s). The "
              "extra entry means sd_card_cmd reached S_WAIT with block_read_req "
              "still asserted and re-armed itself, burning a whole timeout nobody "
              "asked for. That is the race the drop in S_READ's error branch "
              "exists to close."
              % (sysm.error_events, sysm.card.tokens_withheld,
                 sysm.cmd.read_wait_entries, sysm.card.cmd17_attempt))
        return 1
    if cmd_pre and not cs_high_in_pre:
        print("VERDICT: S_CMD_PRE was entered but never held CS high, so the "
              "deselect byte was skipped. The stale wr_ack column above says why. "
              "The re-issued CMD17 goes out with CS still low, at a card that is "
              "still inside the aborted read -- a real RTL defect, not a model "
              "artifact.")
        return 1
    if aborts >= 1 and deselects == 0:
        print("VERDICT: CS does go high and the card does abort the stalled "
              "transfer, but no complete SPI transaction runs with CS high. The "
              "deselect is only the tail of the byte that was in flight when the "
              "timeout fired -- about 6 SCK periods at div=0, under the 8 clocks "
              "the SD spec lets a card keep driving the line after deselect. The "
              "retry would probably work and would be relying on luck. This is "
              "exactly what S_RETRY_GAP exists to remove.")
        return 1
    if aborts >= 1 and deselects >= 1:
        print("VERDICT: the card aborted the stalled transfer on the CS edge AND "
              "%d complete transaction(s) ran with CS high afterwards, so the "
              "re-issued CMD17 is preceded by an unambiguous full byte of "
              "deselect. The rows above show S_CMD_PRE entering on an idle "
              "spi_master and transmitting its own byte rather than consuming the "
              "ack of the one that was in flight at the timeout." % deselects)
        return 0
    print("VERDICT: inconclusive from this window; widen --pre/--post and look "
          "again rather than guessing.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
