#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle-accurate model of sd_card_bmp.v's load scheduler, built to check the
picture-level retry added for the "only three of four pictures play" defect.

Why a model rather than a simulator
-----------------------------------
There is no iverilog or verilator on this machine, and the defect lives in a
pure scheduling decision -- which index gets loaded next, and what happens
when an attempt dies -- rather than in any datapath. The card-side facts that
decision depends on are already established by tools/sim_dir_scan.py against
the real card: the scanner finds all four BMPs, at clusters 3/116/229/342, so
img_found_count is 4 and the missing picture is a load that died, not a file
that was never seen.

Two rules this model has to respect to mean anything
----------------------------------------------------
1. Every right-hand side reads the PRE-cycle state. The RTL is one
   `always @(posedge clk)` block of non-blocking assignments, so `load_busy`
   on the right of the arming condition is the registered value even when the
   failure branch above has already scheduled it to clear. Reading an updated
   value here would let the model arm a new load in the same cycle a failure
   landed, which the hardware cannot do.
2. The two writers of next_load_idx -- the rollback on failure and the
   increment on arming -- must never both fire. They are exclusive because
   load_gave_up implies load_busy while arming requires !load_busy; the model
   asserts it instead of trusting it.

The 1s stall watchdog is modelled as STALL_LIMIT = 20 cycles rather than
100_000_000 - 1. Only the constant is scaled; the comparison, the progress
reset and the resulting load_abort are transcribed unchanged.

Scenarios
---------
  A  all four loads succeed
  B  picture 2 dies once then succeeds          -- the reported defect
  C  picture 3 dies on every attempt
  D  picture 0 dies on every attempt
  E  picture 1 dies on the stall watchdog instead of load_failed
  F  CONTROL  same single failure as B, pre-fix scheduler with no retry

Run:  python tools/sim_load_retry.py
"""

SCAN_TARGET_COUNT = 4
LOAD_MAX_RETRY = 3
STALL_LIMIT = 20
CYCLES_PER_LOAD = 50

# sector_lut(), from the four img_sectorN registers the scan fills in. These
# are the real absolute LBAs sim_dir_scan.py read off the card.
IMG_SECTORS = [34832, 36640, 38448, 40256]

REGS = ('img_found_count img_loaded_count next_load_idx img_idx disp_buf_idx '
        'load_buf_idx write_buf_idx load_idx load_sector load_busy '
        'load_retry_cnt load_stall_cnt source_done_seen write_done_seen '
        'scan_kicked first_image_committed display_valid scan_done '
        'scan_raw_only raw_fallback_started load_abort').split()


def next_index_limited(cur, count):
    if count <= 1:
        return 0
    if count == 2:
        return 0 if cur == 1 else (cur + 1) & 3
    if count == 3:
        return 0 if cur == 2 else (cur + 1) & 3
    return 0 if cur == 3 else (cur + 1) & 3


class Design:
    """sd_card_bmp's scheduler plus just enough bmp_read to drive it."""

    def __init__(self, fail_plan=None, stall_plan=None, retry_enabled=True):
        # fail_plan[pic] = how many leading attempts of that picture fail
        self.fail_plan = dict(fail_plan or {})
        # stall_plan[pic] = set of attempt numbers that go silent instead
        self.stall_plan = stall_plan or {}
        # False reproduces the pre-fix scheduler, where a dead attempt cleared
        # load_busy and nothing else: next_load_idx had already advanced, so
        # the picture was lost for good. Used as a negative control.
        self.retry_enabled = retry_enabled

        for r in REGS:
            setattr(self, r, False if r.endswith(('busy', 'seen', 'kicked',
                                                  'committed', 'valid',
                                                  'done', 'only', 'started',
                                                  'abort'))
                    else 0)

        self.bmp_state = 'IDLE'
        self.bmp_age = 0
        self.cur_pic = None
        self.attempts = {}
        self.stalling = False
        self.pending_load_start = False
        self.pending_load_abort = False

        self.trace = []
        self.armed = []
        self.aborts = 0
        self.abort_run = 0
        self.max_abort_run = 0
        self.auto_play_en = False

    # ---- bmp_read -------------------------------------------------------
    def _fails(self, pic, attempt, stall):
        if stall:
            return attempt in self.stall_plan.get(pic, set())
        # attempt is 1-based, so "the first N attempts fail" is attempt <= N.
        return attempt <= self.fail_plan.get(pic, 0)

    def step_bmp_read(self):
        """Returns (bmp_ready, load_failed, write_finish, load_progress).

        Consumes the pulses sd_card_bmp registered last cycle, which is the
        one-cycle delay the RTL has between load_start_pulse and load_start.
        """
        ready = failed = wfinish = progress = False

        if self.pending_load_abort:
            self.aborts += 1
            self.bmp_state = 'IDLE'
            self.stalling = False
            ready = True
        elif self.bmp_state == 'IDLE':
            ready = True
            if self.pending_load_start:
                pic = IMG_SECTORS.index(self.load_sector)
                self.attempts[pic] = self.attempts.get(pic, 0) + 1
                self.cur_pic = pic
                self.bmp_state = 'LOADING'
                self.bmp_age = 0
                self.stalling = False
                ready = False
        elif self.bmp_state == 'LOADING':
            pic, att = self.cur_pic, self.attempts[self.cur_pic]
            if self.stalling or self._fails(pic, att, stall=True):
                # Silent: no bmp_data_wr_en, no write_finish, so the watchdog
                # in sd_card_bmp is the only thing that can end this.
                self.stalling = True
            else:
                progress = True
                self.bmp_age += 1
                if self.bmp_age >= CYCLES_PER_LOAD:
                    if self._fails(pic, att, stall=False):
                        failed = True
                        self.bmp_state = 'IDLE'
                    else:
                        self.bmp_state = 'DONE'
                        ready = True
                        wfinish = True
        elif self.bmp_state == 'DONE':
            self.bmp_state = 'IDLE'
            ready = True

        self.pending_load_start = False
        return ready, failed, wfinish, progress

    # ---- sd_card_bmp ----------------------------------------------------
    def step(self, scan_found_valid=False, auto_tick=False):
        old = {r: getattr(self, r) for r in REGS}
        nxt = dict(old)

        # Defaults driven every cycle at the top of the RTL's else branch,
        # which later assignments in the same block override. load_abort is
        # defaulted here, not in the idle branch: that default is what makes
        # the stall path's load_abort <= load_stall_hit a one-cycle pulse.
        self.load_start_pulse = False
        nxt['load_abort'] = False

        bmp_ready, load_failed, write_finish, progress = self.step_bmp_read()

        if scan_found_valid and old['img_found_count'] < 4:
            nxt['img_found_count'] = old['img_found_count'] + 1

        if old['load_busy'] and bmp_ready:
            nxt['source_done_seen'] = True
        if old['load_busy'] and write_finish:
            nxt['write_done_seen'] = True

        source_done_now = old['source_done_seen'] or (old['load_busy'] and bmp_ready)
        write_done_now = old['write_done_seen'] or (old['load_busy'] and write_finish)
        load_complete_now = old['load_busy'] and source_done_now and write_done_now
        load_stall_hit = (old['load_busy'] and not progress and
                          old['load_stall_cnt'] >= STALL_LIMIT - 1)
        load_gave_up = (old['load_busy'] and load_failed) or load_stall_hit

        if load_gave_up:
            pic = old['next_load_idx'] - 1
            nxt['load_busy'] = False
            nxt['source_done_seen'] = False
            nxt['write_done_seen'] = False
            nxt['load_stall_cnt'] = 0
            nxt['load_abort'] = load_stall_hit
            if self.retry_enabled and old['load_retry_cnt'] < LOAD_MAX_RETRY:
                nxt['load_retry_cnt'] = old['load_retry_cnt'] + 1
                nxt['next_load_idx'] = old['next_load_idx'] - 1
                self.trace.append(
                    f'    picture {pic} attempt {self.attempts.get(pic)} died'
                    f'{" (stall)" if load_stall_hit else ""} -> retry '
                    f'{nxt["load_retry_cnt"]}/{LOAD_MAX_RETRY}, next_load_idx '
                    f'{old["next_load_idx"]}->{nxt["next_load_idx"]}')
            else:
                nxt['load_retry_cnt'] = 0
                if self.retry_enabled:
                    self.trace.append(
                        f'    picture {pic} exhausted its '
                        f'{LOAD_MAX_RETRY + 1} attempts -> DROPPED')
                else:
                    self.trace.append(
                        f'    picture {pic} died -> DROPPED immediately '
                        f'(pre-fix behaviour, no retry)')
        elif load_complete_now:
            nxt['load_busy'] = False
            nxt['source_done_seen'] = False
            nxt['write_done_seen'] = False
            nxt['load_stall_cnt'] = 0
            nxt['load_retry_cnt'] = 0
            if old['img_loaded_count'] < SCAN_TARGET_COUNT:
                nxt['img_loaded_count'] = old['img_loaded_count'] + 1
            if not old['first_image_committed'] and old['load_buf_idx'] == 0:
                nxt['disp_buf_idx'] = old['load_buf_idx']
                nxt['img_idx'] = old['load_buf_idx']
                nxt['display_valid'] = True
                nxt['first_image_committed'] = True
            self.trace.append(
                f'    picture {self.cur_pic} into buffer {old["load_buf_idx"]}'
                f', img_loaded_count={nxt["img_loaded_count"]}')
        elif old['load_busy']:
            nxt['load_stall_cnt'] = 0 if progress else old['load_stall_cnt'] + 1
        else:
            nxt['load_stall_cnt'] = 0

        if not old['scan_kicked'] and bmp_ready:
            nxt['scan_kicked'] = True
            nxt['first_image_committed'] = False
            nxt['img_found_count'] = 0
            nxt['img_loaded_count'] = 0
            nxt['next_load_idx'] = 0
            nxt['load_retry_cnt'] = 0
        else:
            if (self.auto_play_en and old['first_image_committed'] and
                    old['img_loaded_count'] > 1 and auto_tick):
                n = next_index_limited(old['img_idx'], old['img_loaded_count'])
                nxt['img_idx'] = n
                nxt['disp_buf_idx'] = n

            arm = (old['scan_done'] and bmp_ready and not old['load_busy'] and
                   old['next_load_idx'] < old['img_found_count'] and
                   old['img_loaded_count'] < SCAN_TARGET_COUNT)
            if arm:
                assert not load_gave_up, \
                    'rollback and arming both wrote next_load_idx this cycle'
                idx = old['next_load_idx']
                nxt['load_idx'] = idx & 3
                nxt['load_buf_idx'] = old['img_loaded_count'] & 3
                nxt['load_sector'] = IMG_SECTORS[idx & 3]
                nxt['write_buf_idx'] = old['img_loaded_count'] & 3
                nxt['next_load_idx'] = idx + 1
                nxt['load_busy'] = True
                nxt['source_done_seen'] = False
                nxt['write_done_seen'] = False
                nxt['load_stall_cnt'] = 0
                self.load_start_pulse = True
                self.armed.append((idx, IMG_SECTORS[idx & 3],
                                   old['img_loaded_count'] & 3))

        for k, v in nxt.items():
            setattr(self, k, v)
        self.abort_run = self.abort_run + 1 if self.load_abort else 0
        self.max_abort_run = max(self.max_abort_run, self.abort_run)
        self.pending_load_start = self.load_start_pulse
        self.pending_load_abort = self.load_abort

    # ---- drivers --------------------------------------------------------
    def run(self, max_cycles=20000):
        self.step()                                   # kick the scan
        for sec in IMG_SECTORS:                       # four scan_found_valid
            self.step(scan_found_valid=True)
        self.scan_done = True
        for c in range(max_cycles):
            self.step()
            if (self.scan_done and not self.load_busy and
                    self.next_load_idx >= self.img_found_count):
                return c
        raise AssertionError('scheduler never settled -- runaway loop')

    def rotation(self, ticks=12):
        if self.img_loaded_count < 2 or not self.display_valid:
            return []
        self.auto_play_en = True
        out = []
        for _ in range(ticks):
            self.step(auto_tick=True)
            out.append(self.img_idx + 1)
        return out


def report(name, design, expect_loaded, expect_rot, expect_arms=None):
    design.run()
    rot = design.rotation()
    ok = design.img_loaded_count == expect_loaded and rot == expect_rot
    if expect_arms is not None and len(design.armed) != expect_arms:
        ok = False
    # bmp_read holds itself in ST_IDLE while load_abort is high, so the abort
    # has to be a single pulse. Measured against the waveform the RTL's
    # top-of-block default produces, not assumed.
    abort_ok = design.max_abort_run <= 1
    if not abort_ok:
        ok = False
    print(f'{name}')
    print(f'  img_found_count   {design.img_found_count}')
    print(f'  img_loaded_count  {design.img_loaded_count}'
          f'   expected {expect_loaded}')
    print(f'  display_valid     {design.display_valid}')
    print(f'  OSD rotation      {rot}   expected {expect_rot}')
    print(f'  attempts          {len(design.armed)}'
          f'{f"   expected {expect_arms}" if expect_arms is not None else ""}'
          f'   {[(p, s, b) for p, s, b in design.armed]}')
    print(f'  stall aborts      {design.aborts}'
          f'   longest load_abort high run {design.max_abort_run} cycle(s)'
          f'{"" if abort_ok else "  <-- bmp_read frozen, not a pulse"}')
    for line in design.trace:
        print(line)
    print(f'  -> {"PASS" if ok else "FAIL"}')
    print()
    return ok


def main():
    four = [2, 3, 4, 1] * 3
    three = [2, 3, 1] * 4
    results = [
        report('A  all four loads succeed',
               Design(), 4, four, expect_arms=4),
        report('B  picture 2 dies once then succeeds  (the reported defect)',
               Design(fail_plan={2: 1}), 4, four, expect_arms=5),
        report('C  picture 3 dies on every attempt',
               Design(fail_plan={3: 99}), 3, three, expect_arms=7),
        report('D  picture 0 dies on every attempt',
               Design(fail_plan={0: 99}), 3, three, expect_arms=7),
        report('E  picture 1 dies on the 1s stall watchdog then succeeds',
               Design(stall_plan={1: {1}}), 4, four, expect_arms=5),
        # Negative control. Same single transient failure as B, but against
        # the pre-fix scheduler. It has to reproduce exactly what the user
        # reported -- three pictures playing, four never appearing -- or the
        # PASS on B above does not mean anything.
        report('F  CONTROL  picture 2 dies once, pre-fix scheduler (no retry)',
               Design(fail_plan={2: 1}, retry_enabled=False), 3, three,
               expect_arms=4),
    ]
    print('=' * 64)
    verdict = 'ALL SCENARIOS PASS' if all(results) else 'FAILURES PRESENT'
    print(f'{verdict}   ({sum(results)}/{len(results)})')
    return 0 if all(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
