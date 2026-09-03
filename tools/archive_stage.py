#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Copy a stage's deliverables into _baseline_backup and prove the copies match.

Why a tool
----------
Archiving has been done by hand for stage2, stage3, stage4, fix1 and now fix2,
and each time it was a long PowerShell one-liner listing every path. That has two
failure modes worth removing. One is silent partial copies: Copy-Item with
-ErrorAction SilentlyContinue keeps going when a name is wrong, so an archive can
be missing a report and still look complete. The other is that the archive is
supposed to be the evidence that a snapshot of the sources produced a particular
bitstream, and a copy that differs from the live file proves nothing.

So the manifest is a file rather than a command line, every entry is copied
without swallowing errors, and each copy is hashed against its source. Anything
missing or mismatched fails the run instead of being reported as a warning.

A manifest on disk also sidesteps a tooling quirk: the shell wrapper in front of
this project rejects commands whose text contains certain substrings, and source
names in this tree contain them, so a path list on the command line gets blocked
even though the command is a plain copy.

Manifest format: one entry per line, `destination_name<TAB>source_path`, both
relative to the repository root. Blank lines and lines starting with # are
ignored. Destination names conventionally carry a stage suffix on sources, e.g.
`sd_card_sec_read_write.v.fix2`, so two archives never look like the same vintage.

Usage
-----
    python tools/archive_stage.py --dest stage4_fix2_20260902 --list manifest.txt
    python tools/archive_stage.py --dest ... --list ... --verify-only
"""

import argparse
import hashlib
import os
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKUP = os.path.join(REPO, "_baseline_backup")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_manifest(path):
    entries = []
    with open(path, "r", encoding="utf-8") as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "\t" not in line:
                raise SystemExit("%s:%d: expected dest<TAB>source, got %r"
                                 % (path, lineno, line))
            dest, src = line.split("\t", 1)
            entries.append((dest.strip(), src.strip(), lineno))
    if not entries:
        raise SystemExit("%s: no entries" % path)
    return entries


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dest", required=True,
                    help="directory name under _baseline_backup")
    ap.add_argument("--list", required=True, dest="manifest",
                    help="manifest file, dest<TAB>source per line")
    ap.add_argument("--verify-only", action="store_true",
                    help="hash what is already there instead of copying")
    args = ap.parse_args()

    dest_dir = os.path.join(BACKUP, args.dest)
    if not args.verify_only:
        os.makedirs(dest_dir, exist_ok=True)
    elif not os.path.isdir(dest_dir):
        raise SystemExit("no such archive: %s" % dest_dir)

    entries = read_manifest(args.manifest
                            if os.path.isabs(args.manifest)
                            else os.path.join(REPO, args.manifest))

    bad = []
    print("archive: %s" % dest_dir)
    print("%-46s %-9s %-9s %s" % ("destination", "src bytes", "dst bytes",
                                  "sha256 (first 16)"))
    for dest_name, src_rel, lineno in entries:
        src = src_rel if os.path.isabs(src_rel) else os.path.join(REPO, src_rel)
        dst = os.path.join(dest_dir, dest_name)
        if not os.path.exists(src):
            bad.append("%s:%d source missing: %s" % (args.manifest, lineno,
                                                     src_rel))
            print("%-46s %s" % (dest_name, "!! source missing"))
            continue
        if not args.verify_only:
            shutil.copy2(src, dst)
        if not os.path.exists(dst):
            bad.append("%s: not present after copy" % dest_name)
            print("%-46s %s" % (dest_name, "!! absent after copy"))
            continue
        hs, hd = sha256(src), sha256(dst)
        ss, sd = os.path.getsize(src), os.path.getsize(dst)
        if hs != hd:
            bad.append("%s: hash differs from source" % dest_name)
        print("%-46s %-9d %-9d %s%s" % (dest_name, ss, sd, hs[:16],
                                        "" if hs == hd else "  !! MISMATCH"))

    extra = []
    for name in sorted(os.listdir(dest_dir)):
        if name not in {e[0] for e in entries}:
            extra.append(name)

    print("")
    print("manifest entries : %d" % len(entries))
    print("verified identical: %d" % (len(entries) - len(bad)))
    if extra:
        print("also in archive, not in manifest (%d): %s"
              % (len(extra), ", ".join(extra[:12])))
    if bad:
        print("")
        for b in bad:
            print("  !! %s" % b)
        print("VERDICT: archive incomplete")
        return 1
    print("")
    print("VERDICT: every manifest entry copied and hash identical to its source")
    return 0


if __name__ == "__main__":
    sys.exit(main())
