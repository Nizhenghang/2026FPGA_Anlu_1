#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dump the FAT32 root directory of a volume in PHYSICAL entry order.

bmp_read.v walks root directory entries in order and stops after it has found
SCAN_TARGET_COUNT BMPs, so the physical order of the entries -- not the sorted
order os.listdir() reports -- decides whether MUSIC.WAV's entry is ever seen.
This prints exactly what the scanner sees, marking LFN slots and deleted slots,
which os.listdir() hides.

For FAT volumes FSCTL_GET_RETRIEVAL_POINTERS reports cluster numbers relative to
the first cluster of the data area, so LCN n there is FAT cluster n+2. This
script parses the FAT directly instead and prints real cluster numbers.

Usage: python tools/dump_root_dir.py F:
"""

import ctypes
import struct
import sys
from ctypes import wintypes

GENERIC_READ = 0x80000000
FILE_SHARE_READ = 0x1
FILE_SHARE_WRITE = 0x2
OPEN_EXISTING = 3
INVALID_HANDLE = wintypes.HANDLE(-1).value

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.CreateFileW.restype = wintypes.HANDLE
k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                            ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                            wintypes.HANDLE]
k32.ReadFile.restype = wintypes.BOOL
k32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                         ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.SetFilePointerEx.restype = wintypes.BOOL
k32.SetFilePointerEx.argtypes = [wintypes.HANDLE, ctypes.c_longlong,
                                 ctypes.POINTER(ctypes.c_longlong), wintypes.DWORD]
k32.CloseHandle.argtypes = [wintypes.HANDLE]


class Volume(object):
    def __init__(self, drive):
        letter = drive.rstrip("\\").rstrip(":")
        self.h = k32.CreateFileW("\\\\.\\" + letter + ":", GENERIC_READ,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                                 OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE or self.h is None:
            raise OSError(ctypes.get_last_error(),
                          "cannot open \\\\.\\%s: (try an elevated shell)" % letter)
        # The boot sector must be read with a literal 512 because self.bps is
        # only known once that sector has been parsed.
        off = ctypes.c_longlong(0)
        newpos = ctypes.c_longlong(0)
        if not k32.SetFilePointerEx(self.h, off, ctypes.byref(newpos), 0):
            raise OSError(ctypes.get_last_error())
        raw = ctypes.create_string_buffer(512)
        got = wintypes.DWORD(0)
        if not k32.ReadFile(self.h, raw, 512, ctypes.byref(got), None):
            raise OSError(ctypes.get_last_error())
        boot = raw.raw[:got.value]
        self.bps = struct.unpack_from("<H", boot, 11)[0]
        self.spc = boot[13]
        rsvd = struct.unpack_from("<H", boot, 14)[0]
        nfats = boot[16]
        self.fatsz = struct.unpack_from("<I", boot, 36)[0]
        self.root_cluster = struct.unpack_from("<I", boot, 44)[0]
        self.fat_sector = rsvd
        self.data_sector = rsvd + nfats * self.fatsz
        self.signature = boot[510:512]
        self.fs_id = (boot[82:90].rstrip(b"\x00 ").decode("latin1")
                      or boot[54:62].rstrip(b"\x00 ").decode("latin1"))

    def read_sectors(self, lba, count=1):
        off = ctypes.c_longlong(lba * self.bps)
        newpos = ctypes.c_longlong(0)
        if not k32.SetFilePointerEx(self.h, off, ctypes.byref(newpos), 0):
            raise OSError(ctypes.get_last_error())
        buf = ctypes.create_string_buffer(count * self.bps)
        got = wintypes.DWORD(0)
        if not k32.ReadFile(self.h, buf, count * self.bps, ctypes.byref(got), None):
            raise OSError(ctypes.get_last_error())
        return buf.raw[:got.value]

    def cluster_sectors(self, cluster):
        return self.data_sector + (cluster - 2) * self.spc

    def next_cluster(self, cluster):
        per_sector = self.bps // 4
        sec = self.fat_sector + cluster // per_sector
        data = self.read_sectors(sec, 1)
        return struct.unpack_from("<I", data, (cluster % per_sector) * 4)[0] & 0x0FFFFFFF

    def close(self):
        k32.CloseHandle(self.h)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    vol = Volume(argv[1])
    print("volume %s" % argv[1])
    print("  fs id           %r  signature %s" % (vol.fs_id, vol.signature.hex()))
    print("  bytes/sector    %d" % vol.bps)
    print("  sectors/cluster %d" % vol.spc)
    print("  FAT at sector   %d, %d sectors" % (vol.fat_sector, vol.fatsz))
    print("  data area at    sector %d (cluster 2)" % vol.data_sector)
    print("  root cluster    %d -> sector %d"
          % (vol.root_cluster, vol.cluster_sectors(vol.root_cluster)))

    print("\nroot directory, physical entry order (what bmp_read.v walks):")
    print("  idx  kind   8.3 name        attr    cluster      size  note")

    cluster = vol.root_cluster
    idx = 0
    lfn_parts = []
    bmp_seen = 0
    wav_entry = None
    first_bmp_entry = None
    end = False

    try:
        while cluster < 0x0FFFFFF8 and not end:
            data = vol.read_sectors(vol.cluster_sectors(cluster), vol.spc)
            for slot in range(0, len(data), 32):
                ent = data[slot:slot + 32]
                if len(ent) < 32:
                    break
                first = ent[0]
                if first == 0x00:
                    print("  %3d  --   <END OF DIRECTORY>" % idx)
                    end = True
                    break
                if first == 0xE5:
                    print("  %3d  --   <free/deleted slot>            0xE5" % idx)
                    idx += 1
                    continue

                attr = ent[11]
                if attr == 0x0F:
                    chars = ent[1:11] + ent[14:26] + ent[28:32]
                    frag = chars.decode("utf-16-le", "replace")
                    frag = frag.replace("\x00", "").replace("\uffff", "")
                    lfn_parts.append((first & 0x3F, frag))
                    print("  %3d  LFN  attr 0x0F                 "
                          "         REJECTED by dir_attr==0x0F, frag %r"
                          % (idx, frag))
                    idx += 1
                    continue

                name = ent[0:8].rstrip(b" ").decode("latin1")
                ext = ent[8:11].rstrip(b" ").decode("latin1")
                clo = struct.unpack_from("<H", ent, 26)[0]
                chi = struct.unpack_from("<H", ent, 20)[0]
                size = struct.unpack_from("<I", ent, 28)[0]
                lfn = "".join(f for _, f in sorted(lfn_parts, reverse=True))
                lfn_parts = []

                note = ""
                if attr & 0x10:
                    kind = "DIR"
                    note = "REJECTED by dir_attr[4]"
                else:
                    kind = "file"
                    if ext.upper() == "BMP":
                        bmp_seen += 1
                        if first_bmp_entry is None:
                            first_bmp_entry = idx
                        note = "<== BMP #%d, counted by the scanner" % bmp_seen
                    elif ext.upper() == "WAV":
                        wav_entry = idx
                        note = "<== WAV, captured by scan_found_wav_*"
                if lfn:
                    note += "  LFN=%r" % lfn

                print("  %3d  %-4s %-13s.%-3s 0x%02X  %9d  %9d  %s"
                      % (idx, kind, name, ext, attr, clo, size, note))
                idx += 1

                if bmp_seen >= 4:
                    print("\n  scanner has its 4 BMPs at entry %d and STOPS here."
                          % idx)
                    end = True
                    break
            if not end:
                cluster = vol.next_cluster(cluster)
                if cluster < 2 or cluster >= 0x0FFFFFF8:
                    break
    finally:
        vol.close()

    print("\nverdict")
    if wav_entry is None:
        print("  FAIL: no WAV entry was reached before the scan stopped.")
        print("        bmp_read never captures the WAV directory entry, so")
        print("        audio_phase never gets a valid start sector: SILENCE.")
        return 1
    if first_bmp_entry is None or wav_entry < first_bmp_entry:
        print("  PASS: MUSIC.WAV at entry %d precedes the first BMP at entry %d."
              % (wav_entry, first_bmp_entry))
        return 0
    print("  FAIL: MUSIC.WAV at entry %d comes AFTER the first BMP at entry %d."
          % (wav_entry, first_bmp_entry))
    print("        The scanner stops at 4 BMPs and never sees the WAV: SILENCE.")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
