#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Summarise an Anlogic TD final_timing.rpt.

Two things this exists for. First, the per path group table: the Clock Summary
at the top of the report folds intra and inter clock slack into one number per
clock, which hides WHERE the worst path actually is, and reading the raw report
for that means scrolling twelve thousand lines. Second, and more important for
a staged project like this one, the endpoint scan at the bottom: it says whether
the worst paths belong to the logic just written or to logic that was already
verified in an earlier stage. A stage that only ever tightens its OWN paths is
fine; a stage whose worst path is someone else's register is telling you the
number moved for placement reasons, and those two facts need different answers.

Usage:  python tools/rpt_summary.py <path to final_timing.rpt> [label]
        defaults to the phy_1 report of this project, labelled stage4.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RPT = os.path.normpath(os.path.join(
    HERE, os.pardir, 'src', 'td_project',
    'HDMI1.4b_Transmitter_v1.0_Runs', 'phy_1', 'final_timing.rpt'))


def main():
    rpt = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_RPT
    label = sys.argv[2] if len(sys.argv) > 2 else 'current'
    with open(rpt, 'r', encoding='utf-8', errors='replace') as f:
        d = f.read().split('\n')

    print("report : %s" % rpt)
    print("label  : %s" % label)
    for l in d[:30]:
        if re.search(r'STA coverage|SWNS:|HWNS:|Generated|Confidence', l):
            print("  " + l.strip())
    print()

    # ---- clock summary table
    print("=== Clock Summary (intra + inter folded together) ===")
    started = False
    for l in d[30:80]:
        if 'Name' in l and 'SWNS' in l:
            started = True
            print("%-6s %-9s %-9s %-9s %9s %9s" %
                  ('id', 'C-Period', 'R-Period', 'R-Freq', 'SWNS', 'STNS'))
            continue
        if started:
            m = re.match(r'\s*(clk\d+)\s+\S+\s+([-\d.]+)\s+([-\d.]+)\s+'
                         r'([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+'
                         r'([-\d.]+)\s+([-\d.]+)', l)
            if m:
                print("%-6s %-9s %-9s %-9s %9s %9s" %
                      (m.group(1), m.group(4), m.group(6), m.group(7),
                       m.group(8), m.group(9)))
            elif l.strip().startswith('Notes'):
                break
    print()

    # ---- per path group
    print("=== Path Groups, worst first ===")
    grp = None
    rows = []
    for l in d:
        s = l.strip()
        if s.startswith('Path Group'):
            grp = s.split(':', 1)[1].strip()
            continue
        m = re.match(r"Max\s*:\s*SWNS\s*([-\d.]+)ns,\s*STNS\s*([-\d.]+)ns,"
                     r"\s*(\d+)\s*Viol Endpoints,\s*(\d+)\s*Total Endpoints,"
                     r"\s*(\d+)\s*Paths", s)
        if m and grp:
            rows.append((float(m.group(1)), grp, int(m.group(3)),
                         int(m.group(4)), int(m.group(5))))
            grp = None
    rows = sorted(set(rows))
    print("%-40s %9s %8s %9s %11s" %
          ('group', 'SWNS', 'ViolEP', 'TotalEP', 'Paths'))
    for sw, g, v, t, pa in rows:
        print("%-40s %9.3f %8d %9d %11d" % (g, sw, v, t, pa))
    print()

    # ---- the single worst path, in full
    worst = rows[0] if rows else None
    if worst:
        print("=== worst path group: %s  SWNS %.3f ===" % (worst[1], worst[0]))
        for i, l in enumerate(d):
            if l.strip().startswith('Path Group') and \
                    l.split(':', 1)[1].strip() == worst[1]:
                for j in range(i, min(i + 400, len(d))):
                    s = d[j].strip()
                    if s.startswith('Slack') and j > i + 10:
                        for k in range(j - 1, min(j + 14, len(d))):
                            print("  " + d[k].rstrip()[:150])
                        break
                break
    print()

    # ---- who owns the endpoints of the reported worst paths
    print("=== endpoint ownership of every reported Max path ===")
    owners = {}
    ends = []
    for i, l in enumerate(d):
        s = l.strip()
        if s.startswith('Begin Point') or s.startswith('End Point'):
            m = re.match(r'(Begin|End) Point\s*:\s*(\S+)', s)
            if m:
                ends.append((i, m.group(1), m.group(2)))
    for i, kind, name in ends:
        # first hierarchy segment that is not the top level instance
        parts = name.split('/')
        owner = parts[1] if len(parts) > 1 else parts[0]
        owner = re.sub(r'_reg.*$|_syn_\d+.*$|\..*$', '', owner)
        owners[owner] = owners.get(owner, 0) + 1
    for o, c in sorted(owners.items(), key=lambda kv: -kv[1])[:20]:
        print("  %-46s %5d" % (o, c))
    print()

    # ---- does any of THIS stage's logic appear as an endpoint at all
    print("=== stage 4 identifiers appearing as a reported path endpoint ===")
    pat = re.compile(r'video_transition|wipe_pos|grp_to_cross|burst_in_grp|'
                     r'wipe_delta|read_addr_index_top|trans_bot_idx|'
                     r'trans_top_idx|trans_img_idx|trans_fade_level')
    hits = [(i, l.strip()[:150]) for i, l in enumerate(d)
            if pat.search(l) and ('Begin Point' in l or 'End Point' in l)]
    if not hits:
        print("  none. No stage 4 register is the start or end of any reported")
        print("  worst path, so the stage 4 logic is not what limits Fmax.")
    else:
        print("  %d occurrence(s):" % len(hits))
        for i, l in hits[:20]:
            print("   L%d %s" % (i, l))
    return 0


if __name__ == '__main__':
    sys.exit(main())
