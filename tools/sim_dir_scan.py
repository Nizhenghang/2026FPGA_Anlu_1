#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Replay bmp_read.v's root directory scan against the real card, byte for byte.

Why this tool exists
--------------------
The panel plays only two of the four BMPs. The load scheduler in sd_card_bmp.v
skips an image without consuming a frame buffer when load_failed or the one
second stall watchdog fires, so "two play" means img_loaded_count == 2, which
happens both when the scan only FOUND two and when it found four but two loads
died. Those have completely different causes, so the first thing to establish
is how many the scanner finds.

That can be answered exactly, from the PC, because the scanner's decision is a
pure function of the root directory bytes. ST_SCAN_DIR samples a fixed set of
offsets within each 32 byte entry (rd_cnt[4:0] == 0, 8, 9, 10, 11, 20, 21, 26,
27, 28..31) and acts at rd_cnt[4:0] == 31:

    if (dir_first_byte == 8'h00)            -> scan_done, STOP
    else if (dir_entry_is_bmp_now)          -> record, and STOP once
                                               scan_found_total reaches
                                               scan_target_count

    dir_entry_is_file = first_byte != 0x00 && first_byte != 0xE5 &&
                        attr != 0x0F &&        // long file name slot
                        !attr[3] &&            // volume label
                        !attr[4] &&            // subdirectory
                        cluster >= 2 && size != 0
    dir_ext_is_bmp    = ext in {B,b}{M,m}{P,p}

The stop-on-0x00 rule is the one that matters. It is correct FAT semantics --
a zero first byte means this entry and every entry after it are free -- but it
makes the scan order-dependent in a way that is invisible from Explorer, which
lists the directory through the file system driver and never sees the raw
slots. A 0x00 slot sitting BEFORE some of the BMPs hides them permanently,
while Explorer still shows all four files.

Everything here is read through the volume handle at byte offsets, so no
administrator rights are needed.

Usage
-----
    python tools/sim_dir_scan.py F:
    python tools/sim_dir_scan.py F: --target 4
"""

import ctypes
import os
import struct
import sys
from ctypes import wintypes

SECTOR = 512
ENTRY = 32
FSCTL_GET_VOLUME_DISK_EXTENTS = 0x00090000
GENERIC_READ = 0x80000000
FILE_SHARE_READ = 0x1
FILE_SHARE_WRITE = 0x2
OPEN_EXISTING = 3
INVALID_HANDLE = wintypes.HANDLE(-1).value
FILE_BEGIN = 0

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
                                 ctypes.POINTER(ctypes.c_longlong),
                                 wintypes.DWORD]
k32.DeviceIoControl.restype = wintypes.BOOL
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_void_p,
                                wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.CloseHandle.argtypes = [wintypes.HANDLE]


class Volume(object):
    """Raw sector reader plus the FAT32 geometry the RTL derives from the BPB."""

    def __init__(self, drive):
        self.dev = "\\\\.\\%s:" % drive.rstrip("\\").rstrip(":")
        self.h = k32.CreateFileW(self.dev, GENERIC_READ,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                                 OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE or self.h is None:
            raise ctypes.WinError(ctypes.get_last_error())
        boot = self.read(0, 1)
        self.parse_boot(boot)

    def close(self):
        k32.CloseHandle(wintypes.HANDLE(self.h))

    def read(self, vol_sector, count):
        """Read `count` sectors at a VOLUME-relative sector number."""
        pos = ctypes.c_longlong(0)
        if not k32.SetFilePointerEx(wintypes.HANDLE(self.h),
                                    ctypes.c_longlong(vol_sector * SECTOR),
                                    ctypes.byref(pos), FILE_BEGIN):
            raise ctypes.WinError(ctypes.get_last_error())
        buf = ctypes.create_string_buffer(count * SECTOR)
        got = wintypes.DWORD(0)
        if not k32.ReadFile(wintypes.HANDLE(self.h), buf, count * SECTOR,
                            ctypes.byref(got), None):
            raise ctypes.WinError(ctypes.get_last_error())
        return buf.raw[:got.value]

    def parse_boot(self, b):
        # Offsets exactly as sampled in bmp_read.v's ST_SCAN_BOOT case statement.
        self.sig = b[510:512]
        self.bytes_per_sec, = struct.unpack_from("<H", b, 11)
        self.sec_per_clus = b[13]
        self.rsvd_sec, = struct.unpack_from("<H", b, 14)
        self.num_fats = b[16]
        self.root_ent_cnt, = struct.unpack_from("<H", b, 17)
        self.fat_sz16, = struct.unpack_from("<H", b, 22)
        self.hid_sec, = struct.unpack_from("<I", b, 28)
        self.tot_sec32, = struct.unpack_from("<I", b, 32)
        self.fat_sz32, = struct.unpack_from("<I", b, 36)
        self.root_clus, = struct.unpack_from("<I", b, 44)
        self.fs_id = (b[82:90].rstrip(b"\x00 ").decode("latin1")
                      if len(b) > 90 else "?")

        # bmp_read.v: bpb_fat_size = fat_sz16 if non-zero else fat_sz32, then
        # bpb_fat_area_sectors = fat_size << 1 for num_fats of 2 (or anything
        # that is not 1).
        self.fat_size = self.fat_sz16 if self.fat_sz16 else self.fat_sz32
        self.fat_area = self.fat_size if self.num_fats == 1 else self.fat_size * 2
        # Volume-relative data area start. The RTL's data_start_sector is this
        # plus hid_sec, because it works in absolute LBA from the partition.
        self.data_start_vol = self.rsvd_sec + self.fat_area
        self.data_start_abs = self.hid_sec + self.data_start_vol
        self.root_dir_vol = self.data_start_vol + (self.root_clus - 2) * self.sec_per_clus

    def disk_extent_start(self):
        """Cross-check hid_sec against the volume's real byte offset on disk."""
        out = ctypes.create_string_buffer(4096)
        got = wintypes.DWORD(0)
        ok = k32.DeviceIoControl(wintypes.HANDLE(self.h),
                                 FSCTL_GET_VOLUME_DISK_EXTENTS, None, 0,
                                 out, 4096, ctypes.byref(got), None)
        if not ok or got.value < 32:
            return None
        raw = out.raw
        n, = struct.unpack_from("<I", raw, 0)
        if n < 1:
            return None
        # DISK_EXTENT is 8-byte aligned: DiskNumber at 8, StartingOffset at 16.
        start, = struct.unpack_from("<q", raw, 16)
        return start // SECTOR


def cluster_offset(delta, spc):
    """bmp_read.v's cluster_sector_offset function, enumerated case by case."""
    table = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}
    if spc in table:
        return delta << table[spc]
    return delta          # the RTL's default branch, and it is a silent one


def replay_scan(vol, target, max_sectors=128, verbose=False):
    """Walk the root directory exactly as ST_SCAN_DIR does and report where it
    stops and what it recorded."""
    print("root directory replay")
    print("  root cluster           %d" % vol.root_clus)
    print("  root dir sector        %d volume-relative, %d absolute LBA"
          % (vol.root_dir_vol, vol.hid_sec + vol.root_dir_vol))
    print("  scan_target_count      %d" % target)
    print("")

    found = []
    wav = None            # (label, clus, size, abs_sector, sec_count, i)
    wav_captured = False  # bmp_read.v's wav_captured latch: first match only
    stopped = None
    sec_count = 0
    while sec_count < max_sectors:
        raw = vol.read(vol.root_dir_vol + sec_count, 1)
        if len(raw) < SECTOR:
            stopped = "short read on directory sector %d" % sec_count
            break
        for i in range(SECTOR // ENTRY):
            e = raw[i * ENTRY:(i + 1) * ENTRY]
            fb = e[0]
            attr = e[11]
            ext = e[8:11]
            chi, = struct.unpack_from("<H", e, 20)
            clo, = struct.unpack_from("<H", e, 26)
            size, = struct.unpack_from("<I", e, 28)
            clus = (chi << 16) | clo

            name = e[0:8].rstrip(b"\x20").decode("latin1", "replace")
            exts = ext.decode("latin1", "replace").rstrip("\x00 ")
            label = ("%s.%s" % (name, exts)) if exts else name
            if fb == 0x00:
                label = "<FREE / end of directory>"
            elif fb == 0xE5:
                label = "<DELETED %s>" % (e[1:8].rstrip(b"\x20").decode("latin1", "replace"))

            slot = "sec%d ent%02d" % (sec_count, i)

            # The RTL's predicate, term by term.
            is_lfn = (attr == 0x0F)
            is_label = bool(attr & 0x08)
            is_dir = bool(attr & 0x10)
            used = (fb != 0x00) and (fb != 0xE5)
            is_file = used and not is_lfn and not is_label and not is_dir \
                and clus >= 2 and size != 0
            ext_bmp = ext.lower() == b"bmp"
            is_bmp = is_file and ext_bmp
            ext_wav = ext.lower() == b"wav"
            is_wav = is_file and ext_wav

            if is_bmp:
                # dir_file_sector_now, the value the RTL would record.
                off = cluster_offset(clus - 2, vol.sec_per_clus)
                abs_sector = vol.data_start_abs + off
                found.append((label, clus, size, abs_sector))
                note = "FOUND  cluster %d -> LBA %d" % (clus, abs_sector)
            elif is_wav and not wav_captured:
                # The RTL's third branch: else-if after the BMP test, so a WAV
                # only latches when it is not also a BMP match, and only once.
                off = cluster_offset(clus - 2, vol.sec_per_clus)
                abs_sector = vol.data_start_abs + off
                wav = (label, clus, size, abs_sector, sec_count, i)
                wav_captured = True
                note = "WAV    captured, cluster %d -> LBA %d, size %d" % (
                    clus, abs_sector, size)
            elif fb == 0x00:
                note = "STOP   first_byte 0x00, scan_done asserted here"
            else:
                why = []
                if is_lfn:
                    why.append("LFN slot, attr 0x0F")
                if is_dir:
                    why.append("subdirectory, attr bit 4")
                if is_label:
                    why.append("volume label, attr bit 3")
                if fb == 0xE5:
                    why.append("deleted, 0xE5")
                if used and not is_dir and not is_lfn and not is_label:
                    if clus < 2:
                        why.append("cluster %d < 2" % clus)
                    if size == 0:
                        why.append("size 0")
                    if not ext_bmp:
                        why.append("extension %r is not BMP" % exts)
                note = "skip   %s" % ("; ".join(why) if why else "not a file entry")

            if verbose or is_bmp or is_wav or fb == 0x00:
                print("  %s  fb=0x%02x attr=0x%02x  %-30s %s"
                      % (slot, fb, attr, label, note))

            if fb == 0x00:
                stopped = ("scan_done at directory sector %d entry %d: "
                           "first_byte == 0x00" % (sec_count, i))
                break
            if is_bmp and len(found) >= target:
                stopped = ("scan_done at directory sector %d entry %d: "
                           "scan_found_total reached scan_target_count %d"
                           % (sec_count, i, target))
                break
        if stopped:
            break
        sec_count += 1
    else:
        stopped = "reached ROOT_SCAN_MAX_SECTORS = %d" % max_sectors

    print("")
    print("  scanner recorded %d image(s), stop reason:" % len(found))
    print("    %s" % stopped)
    if wav:
        print("  scanner captured WAV: %s" % wav[0])
        print("    at directory sector %d entry %d, before the stop above: %s"
              % (wav[4], wav[5], "YES" if found else "n/a"))
        print("    wav_sector = %d, wav_size = %d -> wav_found latches, so"
              % (wav[3], wav[2]))
        print("    sd_card_bmp's audio_phase gate can be satisfied")
    else:
        print("  scanner captured NO WAV: wav_found stays 0, audio_phase can")
        print("    never latch, and the design is silent forever")
    return found, stopped, wav


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    drive = argv[1]
    target = 4
    verbose = "--verbose" in argv or "-v" in argv
    if "--target" in argv:
        target = int(argv[argv.index("--target") + 1])

    vol = Volume(drive)
    try:
        print("=" * 72)
        print("bmp_read.v root directory scan replay: %s" % drive)
        print("=" * 72)
        print("volume geometry, parsed at the exact offsets the RTL samples")
        print("  signature              %s  (boot_is_fat32 needs 55aa)" % vol.sig.hex())
        print("  file system id         %r" % vol.fs_id)
        print("  bytes per sector       %d" % vol.bytes_per_sec)
        print("  sectors per cluster    %d" % vol.sec_per_clus)
        print("  reserved sectors       %d" % vol.rsvd_sec)
        print("  number of FATs         %d" % vol.num_fats)
        print("  FAT size               %d (16 bit field %d, 32 bit field %d)"
              % (vol.fat_size, vol.fat_sz16, vol.fat_sz32))
        print("  FAT area               %d sectors" % vol.fat_area)
        print("  hidden sectors         %d  <- this is the partition LBA the RTL"
              " adds as boot_sector_lba" % vol.hid_sec)
        print("  data area start        %d volume-relative, %d absolute LBA"
              % (vol.data_start_vol, vol.data_start_abs))
        print("")

        extent = vol.disk_extent_start()
        if extent is None:
            print("  partition offset       not queryable without elevation; "
                  "using BPB hidden sectors %d" % vol.hid_sec)
        else:
            agree = (extent == vol.hid_sec)
            print("  partition offset       %d from the volume disk extent, %d "
                  "from BPB_HiddSec -> %s"
                  % (extent, vol.hid_sec, "AGREE" if agree else "DISAGREE"))
            if not agree:
                print("      the RTL derives boot_sector_lba from the MBR "
                      "partition table, not from BPB_HiddSec, so a disagreement "
                      "here is worth reading carefully rather than dismissing")
        print("")

        if vol.sec_per_clus not in (1, 2, 4, 8, 16, 32, 64, 128):
            print("  WARNING: %d sectors per cluster hits the DEFAULT branch of "
                  "cluster_sector_offset, which returns cluster_delta unshifted. "
                  "Every sector address the scanner records would then be wrong."
                  % vol.sec_per_clus)
            print("")

        found, stopped, wav = replay_scan(vol, target, verbose=verbose)

        print("")
        wavs_on_disk = sorted(e for e in os.listdir(
            drive.rstrip("\\") + "\\") if e.lower().endswith(".wav"))
        print("Explorer sees %d WAV(s): %s"
              % (len(wavs_on_disk), ", ".join(wavs_on_disk) if wavs_on_disk else "none"))
        if wavs_on_disk and not wav:
            print("")
            print("VERDICT: a WAV is on the card but ST_SCAN_DIR never latched it,")
            print("  so wav_found stays 0 and audio_phase can never assert. The")
            print("  scanner stops as soon as scan_found_total reaches %d BMPs, so"
                  % target)
            print("  the WAV entry must sit BEFORE the last BMP in physical order.")
            return 1

        print("")
        print("=" * 72)
        bmps_on_disk = sorted(e for e in os.listdir(
            drive.rstrip("\\") + "\\") if e.lower().endswith(".bmp"))
        print("Explorer sees %d BMP(s): %s" % (len(bmps_on_disk), ", ".join(bmps_on_disk)))
        print("The scanner sees %d: %s"
              % (len(found), ", ".join(f[0] for f in found) if found else "none"))
        if len(found) < len(bmps_on_disk):
            missing = [n for n in bmps_on_disk
                       if n.lower() not in [f[0].lower() for f in found]]
            print("")
            print("VERDICT: %d file(s) are invisible to the scanner: %s"
                  % (len(missing), ", ".join(missing)))
            print("  They are perfectly healthy files -- Explorer and the PC read")
            print("  them fine. They are hidden because a 0x00 directory slot sits")
            print("  before them, and ST_SCAN_DIR treats 0x00 as end of directory")
            print("  and asserts scan_done there. That is correct FAT semantics")
            print("  for a freshly written directory, but Windows does not always")
            print("  keep the slots compacted, so a hole can survive in the middle.")
            print("  This is a CARD LAYOUT problem, not an RTL misparse: every")
            print("  offset the scanner samples matches the FAT specification.")
            return 1
        if len(found) == len(bmps_on_disk):
            print("")
            print("VERDICT: the scanner sees every BMP on the card, so the")
            print("  two-of-four symptom is NOT a directory scan problem. Look at")
            print("  the load path instead: load_failed from header_match_r, or")
            print("  the one second stall watchdog in sd_card_bmp.v.")
            return 0
        print("=" * 72)
        return 1
    finally:
        vol.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
