#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check that an edit did not change a source file's byte level conventions.

Why per file rather than one project wide rule
----------------------------------------------
The first version of this tool assumed every source here is ASCII, LF only and
Tab indented, and reported 21 failures across 12 files. Almost all of them were
the assumption being wrong, not the files:

    timing.sdc                  GBK Chinese prose, CRLF
    bmp_read.v, sd_card_bmp.v   ASCII, LF, space indented
    frame_fifo_*.v              GBK comments, CRLF, Tab indented
    sd_card_cmd.v               ASCII, LF, Tab indented
    top_tf_hdmi_audio.v         GBK comments, CRLF, space indented
    scaler_nn.v                 ASCII, CRLF, space indented

There is no single convention to enforce, and inventing one would mean
re-encoding files that have been building and running on the board for weeks.
Re-encoding timing.sdc in particular would risk the GBK comments the SDC parser
is already reading, and would bury the real change under a whole file diff.

So the rule is comparative: an edit must leave the file's own conventions where
it found them. The committed version from git HEAD is the baseline; the working
copy is compared against it on line endings, indentation style, whether non
ASCII text is present at all, and the trailing newline.

Line endings are compared as git would store them
-------------------------------------------------
This repository runs with core.autocrlf=true and no .gitattributes, so every
blob is stored LF while the working tree is CRLF, LF or mixed depending on which
tool last wrote each line. `git ls-files --eol` shows i/lf for every source here
including w/mixed for Anlogic IP files nobody has ever edited, so a mixed working
tree is the normal state of this checkout rather than something an edit caused.
Comparing raw working tree bytes therefore reports a line ending change for
almost every file and means nothing. The comparison folds CRLF to LF the way git
would on commit, and a bare CR still counts because autocrlf does not touch one.
The working tree style is printed alongside, as information.

Untracked files have no baseline, so they are only checked for internal
consistency -- which is the right scope, since a new file gets to pick.

Usage
-----
    python tools/check_encoding.py                     # files git reports dirty
    python tools/check_encoding.py path/to/a.v ...     # explicit list
"""

import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Only extensions whose byte level form matters to a tool downstream. .log and
# the generated reports are deliberately absent: they are outputs, and their
# encoding follows whatever produced them.
EXTS = (".v", ".py", ".sdc", ".adc", ".ps1", ".tcl")

# A brand new file in these extensions must be pure ASCII. That is not the global
# rule -- frame_fifo_read.v and timing.sdc carry Chinese prose and build fine --
# but those files are GBK, which is what the TD install on this machine reads. A
# new file written by a tool here would be UTF-8, so allowing non ASCII in one
# would put a third character set into the tree for no benefit. An existing file
# is judged against its own HEAD instead, so it may keep whatever it already had.
ASCII_WHEN_NEW = (".v", ".sdc", ".adc", ".tcl", ".ps1")


def git(*args, binary=False):
    out = subprocess.run(["git"] + list(args), cwd=REPO, capture_output=True)
    if out.returncode != 0:
        return None
    return out.stdout if binary else out.stdout.decode("utf-8", "replace")


def autocrlf():
    """core.autocrlf, which decides whether working tree EOL means anything."""
    val = git("config", "--get", "core.autocrlf")
    return (val or "").strip().lower() in ("true", "input")


def as_committed(raw, normalising):
    """The bytes git would store, which is what a comparison has to use.

    With core.autocrlf on, a CRLF in the working tree is stored as LF, so a
    working tree CRLF is not a change to the file as far as the repository is
    concerned. Asking git for the index copy instead does not work here: nothing
    is staged during this kind of check, so `git show :path` just hands back the
    HEAD blob and the comparison becomes a tautology.

    A bare CR is deliberately NOT folded away. autocrlf only translates CRLF, so
    a lone CR survives into the blob and would be a real change.
    """
    return raw.replace(b"\r\n", b"\n") if normalising else raw


def dirty_sources():
    """Sources git reports as modified or untracked."""
    text = git("status", "--porcelain")
    if text is None:
        return []
    paths = []
    for line in text.splitlines():
        if len(line) < 4:
            continue
        rel = line[3:].strip().strip('"')
        if os.path.splitext(rel)[1].lower() in EXTS:
            paths.append(rel.replace("/", os.sep))
    return paths


def profile(raw):
    """Describe the conventions a body of bytes actually uses."""
    crlf = raw.count(b"\r\n")
    cr_total = raw.count(b"\r")
    lf_total = raw.count(b"\n")
    bare_cr = cr_total - crlf
    bare_lf = lf_total - crlf

    if crlf and bare_lf:
        eol = "mixed CRLF+LF"
    elif crlf:
        eol = "CRLF"
    elif bare_lf:
        eol = "LF"
    elif bare_cr:
        eol = "CR"
    else:
        eol = "single line"

    nonascii = sum(1 for b in raw if b > 0x7F)

    tab_led = space_led = 0
    for ln in raw.decode("utf-8", "replace").split("\n"):
        body = ln.rstrip("\r")
        if not body.strip():
            continue
        ws = body[:len(body) - len(body.lstrip())]
        if not ws:
            continue
        if ws[0] == "\t":
            tab_led += 1
        elif ws[0] == " ":
            space_led += 1

    if tab_led and not space_led:
        indent = "Tab"
    elif space_led and not tab_led:
        indent = "space"
    elif tab_led > space_led:
        indent = "Tab led"
    elif space_led > tab_led:
        indent = "space led"
    else:
        indent = "none"

    return dict(eol=eol, crlf=crlf, bare_lf=bare_lf, bare_cr=bare_cr,
                mixed=bool(crlf and (bare_lf or bare_cr)),
                nonascii=nonascii, tab_led=tab_led, space_led=space_led,
                indent=indent, ends_nl=bool(raw) and raw.endswith(b"\n"),
                bytes=len(raw))


def added_lines(rel):
    """The '+' side of git diff, without the marker. None if diff failed."""
    text = git("diff", "--", rel)
    if text is None:
        return None
    return [ln[1:] for ln in text.split("\n")
            if ln.startswith("+") and not ln.startswith("+++")]


def check(path):
    rel = os.path.relpath(path if os.path.isabs(path)
                          else os.path.join(REPO, path), REPO)
    rel_git = rel.replace(os.sep, "/")
    ext = os.path.splitext(rel)[1].lower()
    problems = []
    notes = []

    with open(os.path.join(REPO, rel), "rb") as f:
        raw = f.read()
    normalising = autocrlf()
    now = profile(raw)
    # What the repository would record, and therefore what the line ending
    # comparison has to be made against.
    committed = profile(as_committed(raw, normalising))

    base_raw = git("show", "HEAD:" + rel_git, binary=True)
    base = profile(base_raw) if base_raw is not None else None

    notes.append("worktree: %d bytes, %s, %s, nonascii=%d, ends with newline=%s"
                 % (now["bytes"], now["eol"], now["indent"], now["nonascii"],
                    now["ends_nl"]))
    if normalising:
        notes.append("as git stores it: %s (core.autocrlf folds CRLF to LF)"
                     % committed["eol"])
    if base is None:
        notes.append("HEAD    : untracked, no committed version to compare")
    else:
        notes.append("HEAD    : %d bytes, %s, %s, nonascii=%d, ends with "
                     "newline=%s"
                     % (base["bytes"], base["eol"], base["indent"],
                        base["nonascii"], base["ends_nl"]))

    if now["mixed"]:
        msg = ("worktree line endings are mixed: %d CRLF, %d bare LF"
               % (now["crlf"], now["bare_lf"]))
        if normalising and not committed["mixed"]:
            # Not a defect here. git folds the CRLF side on commit, and
            # `git ls-files --eol` reports w/mixed for Anlogic IP files nobody
            # has ever edited, so this is what an autocrlf checkout looks like.
            notes.append(msg + "; core.autocrlf folds it to %s on commit"
                         % committed["eol"])
        else:
            problems.append(msg + ", and it stays mixed in what git would store")

    if committed["bare_cr"]:
        problems.append("%d bare CR byte(s), which autocrlf does not fold away so "
                        "they would be committed" % committed["bare_cr"])

    if base is None and ext in ASCII_WHEN_NEW and now["nonascii"]:
        problems.append("new %s file carries %d non ASCII byte(s): the Chinese "
                        "prose already in this tree is GBK, which is what TD here "
                        "reads, and a new file would be UTF-8 -- a third character "
                        "set for no benefit" % (ext, now["nonascii"]))

    if base is not None:
        if base["eol"] != committed["eol"]:
            problems.append("committed line ending style changed from %s to %s"
                            % (base["eol"], committed["eol"]))
        if base["indent"] != now["indent"]:
            problems.append("indentation style changed from %r to %r"
                            % (base["indent"], now["indent"]))
        if base["nonascii"] == 0 and now["nonascii"]:
            problems.append("file was pure ASCII at HEAD and now carries %d non "
                            "ASCII byte(s)" % now["nonascii"])
        if base["ends_nl"] != now["ends_nl"]:
            problems.append("trailing newline was %s at HEAD and is %s now"
                            % (base["ends_nl"], now["ends_nl"]))

    # Added lines are compared to the file they landed in, not to a global rule,
    # and only when that file has one rule to compare against. A file that mixes
    # Tab led statements with space led continuation alignment -- frame_fifo_read.v
    # has nine such lines at HEAD -- has no single convention, so enforcing one on
    # a new line would be inventing a rule the file never had.
    added = added_lines(rel_git)
    if added is None:
        notes.append("git diff unavailable, added lines not checked")
    elif not added:
        notes.append("git diff: no added lines")
    else:
        style = now["indent"]
        tabs = sum(1 for ln in added if ln.lstrip("\r").startswith("\t"))
        if style in ("Tab", "space"):
            want = "\t" if style == "Tab" else " "
            bad = []
            for ln in added:
                body = ln.rstrip("\r")
                if not body.strip():
                    continue
                ws = body[:len(body) - len(body.lstrip())]
                if ws and ws[0] != want:
                    bad.append(ln)
            notes.append("git diff: +%d lines, %d Tab led, file is uniformly %s "
                         "indented" % (len(added), tabs, style))
            if bad:
                problems.append("%d added line(s) indented against the file's "
                                "uniform %s style, e.g. %r"
                                % (len(bad), style, bad[0][:60]))
        else:
            notes.append("git diff: +%d lines, %d Tab led; file mixes %d Tab led "
                         "and %d space led lines so added line indentation is "
                         "reported, not enforced"
                         % (len(added), tabs, now["tab_led"], now["space_led"]))
        if base is not None and base["nonascii"] == 0:
            na = [ln for ln in added if any(ord(c) > 127 for c in ln)]
            if na:
                problems.append("%d added line(s) carry non ASCII text into a "
                                "file that was pure ASCII, e.g. %r"
                                % (len(na), na[0][:60]))

    return problems, notes


def main():
    args = sys.argv[1:]
    files = args if args else dirty_sources()
    if not files:
        print("nothing to check: git reports no dirty sources and no files were "
              "named on the command line")
        return 0

    failed = 0
    checked = 0
    for path in files:
        full = path if os.path.isabs(path) else os.path.join(REPO, path)
        if not os.path.exists(full):
            print("%-52s MISSING" % path)
            failed += 1
            continue
        checked += 1
        problems, notes = check(full)
        print("%-52s %s" % (path, "FAIL" if problems else "ok"))
        for n in notes:
            print("%4s %s" % ("", n))
        for p in problems:
            print("%4s !! %s" % ("", p))
        failed += len(problems)

    print("")
    if failed:
        print("VERDICT: %d problem(s) across %d file(s)" % (failed, checked))
        return 1
    print("VERDICT: %d file(s) clean -- every edit left its file's line endings, "
          "indentation, character set and trailing newline the way it found them"
          % checked)
    return 0


if __name__ == "__main__":
    sys.exit(main())
