#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare two Anlogic ASCII bitstreams and say precisely what differs.

A rebuild of an unchanged design is NOT byte identical, because the ASCII
header carries a `# Date:` line with minute resolution. Judging reproducibility
on the whole file hash therefore always reports a false failure. What actually
matters is the configuration body and the `# Bitstream CRC:` field the tool
computes over it. This script separates the two.

Usage:  python tools/cmp_bit.py <a.bit> <b.bit>
Exit 0 when the configuration bodies are identical.
"""

import sys

MARKER = b'# USER CO'          # first header line after the CRC field


def split(path):
    raw = open(path, 'rb').read()
    i = raw.find(MARKER)
    if i < 0:
        raise SystemExit('not an Anlogic ASCII bitstream: %s' % path)
    return raw, raw[:i], raw[i:]


def field(header, name):
    for line in header.split(b'\n'):
        if line.startswith(name):
            return line.strip().decode('ascii', 'replace')
    return '(absent)'


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    ra, ha, ba = split(sys.argv[1])
    rb, hb, bb = split(sys.argv[2])

    diff = [i for i in range(min(len(ra), len(rb))) if ra[i] != rb[i]]
    print('file A            : %s  (%d bytes)' % (sys.argv[1], len(ra)))
    print('file B            : %s  (%d bytes)' % (sys.argv[2], len(rb)))
    print('whole-file diff   : %d byte(s)%s' % (
        len(diff), ('  offsets ' + str(diff[:16])) if diff else ''))
    print('  A %s' % field(ha, b'# Date'))
    print('  B %s' % field(hb, b'# Date'))
    print('header CRC A      : %s' % field(ha, b'# Bitstream CRC'))
    print('header CRC B      : %s' % field(hb, b'# Bitstream CRC'))
    print('config body       : %d / %d bytes, %s' % (
        len(ba), len(bb),
        'IDENTICAL' if ba == bb else '*** DIFFERENT ***'))

    ok = (ba == bb) and field(ha, b'# Bitstream CRC') == field(hb, b'# Bitstream CRC')
    print('verdict           : %s' % (
        'same configuration, only the header date differs' if ok
        else 'genuinely different configurations'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
