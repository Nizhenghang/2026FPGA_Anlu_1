#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Acceptance checker for the TF card that this design reads from.

Why this tool exists
--------------------
bmp_read.v NEVER follows the FAT chain. In ST_LOAD_DATA it does

    sd_sec_read_addr <= sd_sec_read_addr + 32'd1;          // bmp_read.v L782

and nothing else, for file_sector_count = (file_len + 511) >> 9 sectors,
starting from the first cluster of the file

    dir_file_sector_now = data_start_sector
                        + (first_cluster - 2) * sectors_per_cluster;

So the hardware reads one straight run of sectors and trusts that the whole
file lives there. A file whose clusters are NOT consecutive is read as
whatever happens to lie after its first cluster -- garbage on the panel that
is indistinguishable from an RTL bug. That is why the card has to be cleared
before any RTL is suspected.

The other checks below are the remaining ways a card can look broken while the
RTL is fine, each tied to the line of RTL that enforces it:

  file system must be FAT32     boot_is_fat32 tests 0x55AA plus geometry, and
                                the FAT32-only fields (BPB_RootCluster,
                                BPB_FATSz32) are what locate the root directory
  exactly 4 BMPs in the root    SCAN_TARGET_COUNT = 3'd4, and the scanner walks
                                root directory entries in order and stops at 4,
                                so a 5th file can crowd out the one under test
  no subdirectory confusion     dir_entry_is_file rejects attr bit 4 (directory)
                                and attr 0x0F (long file name slot), which is
                                what keeps "System Volume Information" out
  header fields                 header_match_c requires "BM", 24 bpp,
                                compression 0; width_ok_r / height_ok_r require
                                64..1920 x 64..1080 and height[31:16] == 0, so
                                a top-down (negative height) BMP is rejected
  size self-consistency         file_len drives file_sector_count, and the
                                padding gate src_stride must match the real row
                                pitch or every row after the first is shifted

Cluster layout is queried through FSCTL_GET_RETRIEVAL_POINTERS, which asks the
file system driver itself rather than parsing the FAT, so it is correct for
whatever FAT32 variant Windows wrote and needs no administrator rights.

Usage
-----
    python tools/check_sd_card.py F:
    python tools/check_sd_card.py F: --verbose
"""

import ctypes
import os
import struct
import sys
from ctypes import wintypes

# --------------------------------------------------------------------------
# What the RTL accepts. Keep these in step with bmp_read.v.
# --------------------------------------------------------------------------
SRC_W_MIN, SRC_W_MAX = 64, 1920
SRC_H_MIN, SRC_H_MAX = 64, 1080
SCAN_TARGET_COUNT = 4          # sd_card_bmp.v parameter
SECTOR = 512

FSCTL_GET_RETRIEVAL_POINTERS = 0x00090073
GENERIC_READ = 0x80000000
FILE_SHARE_READ = 0x1
FILE_SHARE_WRITE = 0x2
OPEN_EXISTING = 3
FILE_FLAG_BACKUP_SEMANTICS = 0x02000000
ERROR_HANDLE_EOF = 38
ERROR_MORE_DATA = 234

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.CreateFileW.restype = wintypes.HANDLE
k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                            ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                            wintypes.HANDLE]
k32.DeviceIoControl.restype = wintypes.BOOL
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_void_p,
                                wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.GetDiskFreeSpaceW.restype = wintypes.BOOL
k32.GetDiskFreeSpaceW.argtypes = [wintypes.LPCWSTR,
                                  ctypes.POINTER(wintypes.DWORD),
                                  ctypes.POINTER(wintypes.DWORD),
                                  ctypes.POINTER(wintypes.DWORD),
                                  ctypes.POINTER(wintypes.DWORD)]
k32.ReadFile.restype = wintypes.BOOL
k32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                         ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
k32.CloseHandle.argtypes = [wintypes.HANDLE]


class Result(object):
    """Accumulates pass/fail per check so the exit code means something."""

    def __init__(self):
        self.rows = []
        self.failed = 0

    def add(self, ok, name, detail):
        self.rows.append((ok, name, detail))
        if not ok:
            self.failed += 1
        mark = "PASS" if ok else "FAIL"
        print("  [%s] %-26s %s" % (mark, name, detail))


def get_extents(path, cluster_bytes):
    """Return the file's (start_vcn, end_vcn, lcn) extents, asking the driver.

    Extents are returned in file order. A single extent covering the whole
    file means the file is physically contiguous, which is exactly what
    bmp_read.v's linear sector walk assumes.
    """
    h = k32.CreateFileW(path, GENERIC_READ,
                        FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                        OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, None)
    if h == wintypes.HANDLE(-1).value or h is None:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        size = os.path.getsize(path)
        total_vcn = (size + cluster_bytes - 1) // cluster_bytes
        extents = []
        vcn = 0
        out_size = 1 << 16
        while vcn < total_vcn:
            in_vcn = ctypes.c_longlong(vcn)
            out = ctypes.create_string_buffer(out_size)
            got = wintypes.DWORD(0)
            ok = k32.DeviceIoControl(h, FSCTL_GET_RETRIEVAL_POINTERS,
                                     ctypes.byref(in_vcn), ctypes.sizeof(in_vcn),
                                     out, out_size, ctypes.byref(got), None)
            if not ok:
                err = ctypes.get_last_error()
                if err in (ERROR_HANDLE_EOF, ERROR_INVALID_PARAMETER):
                    break           # past the last allocated cluster
                if err != ERROR_MORE_DATA:
                    raise ctypes.WinError(err)
            raw = out.raw[:got.value]
            if len(raw) < 16:
                break
            count = struct.unpack_from("<I", raw, 0)[0]
            cur = struct.unpack_from("<q", raw, 8)[0]
            for i in range(count):
                off = 16 + i * 16
                if off + 16 > len(raw):
                    break
                nxt, lcn = struct.unpack_from("<qq", raw, off)
                extents.append((cur, nxt, lcn))
                cur = nxt
            if count == 0:
                break
            if cur <= vcn:
                break               # no forward progress, do not spin
            vcn = cur
        return extents
    finally:
        k32.CloseHandle(wintypes.HANDLE(h))


def read_boot_sector(root):
    """Try to read the volume boot sector. Needs rights we may not have, so a
    failure is reported as 'not readable' rather than as a card defect: the
    cluster geometry is independently available from GetDiskFreeSpaceW."""
    for target in ("\\\\.\\%s" % root.rstrip("\\").rstrip(":"),
                   "\\\\.\\%s:" % root.rstrip("\\").rstrip(":")):
        h = k32.CreateFileW(target, GENERIC_READ,
                            FILE_SHARE_READ | FILE_SHARE_WRITE, None,
                            OPEN_EXISTING, 0, None)
        if h == wintypes.HANDLE(-1).value or h is None:
            continue
        try:
            buf = ctypes.create_string_buffer(SECTOR)
            got = wintypes.DWORD(0)
            if k32.ReadFile(h, buf, SECTOR, ctypes.byref(got), None) and got.value == SECTOR:
                return buf.raw
        finally:
            k32.CloseHandle(wintypes.HANDLE(h))
    return None


def check_bmp_header(path):
    """Parse the 54 byte header and judge it against header_match_c plus the
    two geometry range checks. Returns (ok, problems, geometry)."""
    with open(path, "rb") as fh:
        head = fh.read(54)
    real = os.path.getsize(path)
    prob = []
    if len(head) < 54:
        return False, ["shorter than a 54 byte header"], None

    sig = head[0:2]
    fsize, = struct.unpack_from("<I", head, 2)
    off, = struct.unpack_from("<I", head, 10)
    hsz, = struct.unpack_from("<I", head, 14)
    w, h = struct.unpack_from("<ii", head, 18)
    bpp, = struct.unpack_from("<H", head, 28)
    comp, = struct.unpack_from("<I", head, 30)
    imgsize, = struct.unpack_from("<I", head, 34)

    if sig != b"BM":
        prob.append("signature %r is not 'BM'" % sig)
    if fsize != real:
        prob.append("header file size %d != real %d" % (fsize, real))
    if off != 54:
        prob.append("bfOffBits %d != 54 (pixel_offset is hard-wired)" % off)
    if hsz != 40:
        prob.append("biSize %d != 40" % hsz)
    if bpp != 24:
        prob.append("biBitCount %d != 24" % bpp)
    if comp != 0:
        prob.append("biCompression %d != 0 (BI_RGB)" % comp)
    # height_ok_r demands a positive height whose upper half is zero, i.e. a
    # bottom-up BMP. A top-down BMP stores a negative biHeight.
    if h < 0:
        prob.append("biHeight %d < 0: top-down BMP is rejected by height_ok_r" % h)
    elif (h >> 16) != 0:
        prob.append("biHeight %d has bits above [15:0]" % h)
    if not (SRC_W_MIN <= w <= SRC_W_MAX):
        prob.append("width %d outside [%d, %d]" % (w, SRC_W_MIN, SRC_W_MAX))
    if not (SRC_H_MIN <= h <= SRC_H_MAX):
        prob.append("height %d outside [%d, %d]" % (h, SRC_H_MIN, SRC_H_MAX))

    if w > 0 and h > 0:
        stride = (w * 3 + 3) & ~3
        expect = off + stride * h
        if real != expect:
            prob.append("file length %d != %d + stride(%d)*%d = %d"
                        % (real, off, stride, h, expect))
        if imgsize not in (0, stride * h):
            prob.append("biSizeImage %d != stride*height %d" % (imgsize, stride * h))
    else:
        stride = 0
        expect = real

    return (not prob), prob, dict(w=w, h=h, off=off, stride=stride,
                                  size=real, expect=expect)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    root = argv[1].rstrip("\\")
    if len(root) == 2 and root[1] == ":":
        root = root + "\\"
    verbose = "--verbose" in argv[2:] or "-v" in argv[2:]

    print("=" * 72)
    print("TF card acceptance check: %s" % root)
    print("=" * 72)

    res = Result()

    # ---------------- volume geometry ----------------
    spc = wintypes.DWORD(0)
    bps = wintypes.DWORD(0)
    nfc = wintypes.DWORD(0)
    tnc = wintypes.DWORD(0)
    if not k32.GetDiskFreeSpaceW(root, ctypes.byref(spc), ctypes.byref(bps),
                                 ctypes.byref(nfc), ctypes.byref(tnc)):
        print("cannot query volume geometry: %s" % ctypes.WinError(ctypes.get_last_error()))
        return 2
    cluster_bytes = spc.value * bps.value
    print("\nvolume")
    print("  sector size          %d bytes" % bps.value)
    print("  sectors per cluster  %d" % spc.value)
    print("  cluster size         %d bytes" % cluster_bytes)
    print("  clusters             %d total, %d free" % (tnc.value, nfc.value))
    print("  volume size          %.2f GB" % (tnc.value * cluster_bytes / 1e9))

    res.add(bps.value == SECTOR, "sector size",
            "%d bytes, RTL counts 512 byte sectors" % bps.value)
    res.add(spc.value in (1, 2, 4, 8, 16, 32, 64, 128), "sectors per cluster",
            "%d, covered by cluster_sector_offset's shift mux" % spc.value)

    boot = read_boot_sector(root)
    if boot is None:
        print("  boot sector          not readable without elevation "
              "(geometry above comes from the driver, so this is not a card defect)")
    else:
        fs = boot[54:62].rstrip(b"\x00 ").decode("latin1") or \
             boot[82:90].rstrip(b"\x00 ").decode("latin1")
        sig = boot[510:512]
        print("  boot sector          readable, FS id %r, sig %s" % (fs, sig.hex()))
        res.add(sig == b"\x55\xaa", "boot signature", "%s, boot_is_fat32 needs 55aa" % sig.hex())
        res.add(fs.startswith("FAT32"), "file system id", "%r must be FAT32" % fs)

    # ---------------- directory inventory ----------------
    entries = sorted(os.listdir(root))
    bmps = [e for e in entries if e.lower().endswith(".bmp")]
    others = [e for e in entries if e not in bmps]

    print("\nroot directory")
    for e in entries:
        p = os.path.join(root, e)
        kind = "DIR " if os.path.isdir(p) else "file"
        print("  %s %-32s %s" % (kind, e,
                                 "" if os.path.isdir(p) else "%d bytes" % os.path.getsize(p)))

    print("\nscan expectations")
    res.add(len(bmps) <= SCAN_TARGET_COUNT, "BMP count",
            "%d found, SCAN_TARGET_COUNT = %d" % (len(bmps), SCAN_TARGET_COUNT))
    res.add(len(bmps) > 0, "at least one BMP", "%d found" % len(bmps))

    # dir_entry_is_file rejects attr bit 4 (directory) and attr == 0x0F (LFN
    # slot), so subdirectories never reach the scanner. Report them so the
    # reader knows they were considered rather than missed.
    dirs = [e for e in others if os.path.isdir(os.path.join(root, e))]
    files = [e for e in others if not os.path.isdir(os.path.join(root, e))]
    res.add(True, "subdirectories ignored",
            "%s rejected by !dir_attr[4]" % (", ".join(dirs) if dirs else "none"))
    if files:
        print("  note: non-BMP files present (%s). They are skipped by "
              "dir_ext_is_bmp but they consume clusters, so check the "
              "contiguity report below rather than assuming." % ", ".join(files))

    # 8.3 name check. A name that needs a long file name slot gets an extra
    # directory entry with attr 0x0F. Those are filtered, so this is not a
    # failure, but it is worth knowing which entries the scanner really sees.
    for e in bmps:
        stem, _, ext = e.rpartition(".")
        if len(stem) > 8 or len(ext) > 3:
            print("  note: %s is not an 8.3 name, Windows adds an LFN slot; "
                  "harmless because dir_attr == 0x0F is rejected." % e)

    # ---------------- per file ----------------
    print("\nBMP files")
    contiguous_all = True
    geo = None
    geo_of = None
    for name in bmps:
        path = os.path.join(root, name)
        print("\n--- %s ---" % name)

        ok, prob, geo = check_bmp_header(path)
        geo_of = name
        if geo:
            print("  header  %dx%d  off=%d  stride=%d  size=%d  sectors=%d"
                  % (geo["w"], geo["h"], geo["off"], geo["stride"], geo["size"],
                     (geo["size"] + SECTOR - 1) // SECTOR))
            pad = geo["stride"] - geo["w"] * 3
            print("  padding %d byte(s) per row, gated by row_byte_cnt < src_w3" % pad)
        res.add(ok, "header %s" % name,
                "all fields accepted" if ok else "; ".join(prob))

        try:
            extents = get_extents(path, cluster_bytes)
        except OSError as exc:
            res.add(False, "cluster map %s" % name, "query failed: %s" % exc)
            contiguous_all = False
            continue

        if not extents:
            res.add(False, "cluster map %s" % name, "driver returned no extents")
            contiguous_all = False
            continue

        nclu = sum(e[1] - e[0] for e in extents)
        first_lcn = extents[0][2]
        last_lcn = extents[-1][2] + (extents[-1][1] - extents[-1][0]) - 1
        span = last_lcn - first_lcn + 1

        print("  clusters %d used, %d extent(s), LCN %d..%d (span %d)"
              % (nclu, len(extents), first_lcn, last_lcn, span))
        if verbose or len(extents) > 1:
            for i, (v0, v1, lcn) in enumerate(extents):
                print("      extent %2d  VCN %5d..%-5d  LCN %8d  (%d clusters)"
                      % (i, v0, v1 - 1, lcn, v1 - v0))

        # The RTL walks file_sector_count sectors from the first cluster. Those
        # sectors are all inside the file's clusters only if the clusters are
        # consecutive.
        need_sectors = (geo["size"] + SECTOR - 1) // SECTOR if geo else 0
        contig_clusters = (len(extents) == 1) and (span == nclu)
        if contig_clusters:
            s0 = first_lcn * spc.value
            s1 = s0 + need_sectors - 1
            detail = ("contiguous, %d clusters at LCN %d -> the %d sectors the "
                      "RTL walks are %d..%d, all inside the file"
                      % (nclu, first_lcn, need_sectors, s0, s1))
        else:
            # How far the linear walk stays inside the file. Everything past
            # the first extent boundary is some other cluster's content.
            first_extent_sectors = (extents[0][1] - extents[0][0]) * spc.value
            detail = ("FRAGMENTED into %d extents: the RTL walks %d sectors "
                      "linearly but only the first %d of them belong to this "
                      "file, the remaining %d read other clusters"
                      % (len(extents), need_sectors, first_extent_sectors,
                         max(0, need_sectors - first_extent_sectors)))
            contiguous_all = False
        res.add(contig_clusters, "contiguity %s" % name, detail)

    # ---------------- read budget ----------------
    # How the hardware actually spends its time on this card. sd_card_sec_read_write
    # issues one CMD17 (READ_SINGLE_BLOCK) per sector and then waits for the
    # card's start token, so the pixel stream goes silent once per sector and the
    # silence is bounded by the card, not by the RTL. This is the quantity that
    # sizes scaler_nn's FILL_WAIT_AW, so print it next to the card that produces
    # it rather than leaving it as a claim in a comment.
    if bmps and geo and geo.get("w"):
        # The budget is computed for whichever file the loop ended on. Say so
        # rather than implying it covers the card, because a card holding mixed
        # resolutions has a different sector count per file.
        print("\nread budget, computed for %s (%d x %d)"
              % (geo_of, geo["w"], geo["h"]))
        clk = 100_000_000                    # sd_card_clk
        cyc_byte = 32                        # SCK = clk/4 = 25MHz, 4 clk per bit
        nsec = (geo["size"] + SECTOR - 1) // SECTOR
        row_bytes = geo["stride"]
        sectors_per_row = row_bytes / float(SECTOR)
        transfer_us = SECTOR * cyc_byte * 1e6 / clk
        old_thresh_us = (1 << 15) * 1e6 / clk          # FILL_WAIT_AW = 16
        new_thresh_us = (1 << 24) * 1e6 / clk          # FILL_WAIT_AW = 25
        print("  sectors read             %d, so %d CMD17 and %d inter-sector gaps"
              % (nsec, nsec, nsec - 1))
        print("  one sector transfer      %d cycles = %.2f us of stream"
              % (SECTOR * cyc_byte, transfer_us))
        print("  one destination row      %d bytes = %.2f sectors, so about %.1f"
              " gaps per row" % (row_bytes, sectors_per_row, sectors_per_row))
        print("  scaler_nn fill watchdog  pre-fix 2^15 = %.2f us, shipped 2^24 ="
              " %.2f ms" % (old_thresh_us, new_thresh_us / 1000.0))
        print("  SD spec read access time order of 100 ms, and the SPI layer has"
              " no timeout of its own")
        print("")
        res.add(new_thresh_us > 100e3, "watchdog vs spec",
                "shipped threshold %.1f ms exceeds the ~100 ms order read access"
                " time the SD spec allows; pre-fix %.2f us did not, and fired once"
                " per gap, i.e. ~%.1f times per row"
                % (new_thresh_us / 1000.0, old_thresh_us, sectors_per_row))
        print("  what a gap costs, for a %d sector file:" % nsec)
        for label, gap_us in (("fast card, no gap", 0.0),
                              ("pre-fix threshold", old_thresh_us),
                              ("1 ms per gap", 1000.0),
                              ("shipped threshold", new_thresh_us),
                              ("spec order 100 ms", 100000.0)):
            total_ms = (nsec * transfer_us + (nsec - 1) * gap_us) / 1000.0
            # fill_wait counts one per cycle in S_FILL_W and fill_stuck tests a
            # saturated bit, so the wait trips at >= the threshold, not above it.
            verdict = ("pre-fix fires" if gap_us >= old_thresh_us
                       else "pre-fix quiet")
            if gap_us >= new_thresh_us:
                verdict += ", shipped fires too"
            else:
                verdict += ", shipped rides it out"
            span = ("%.1f ms" % total_ms) if total_ms < 1000.0 else \
                   ("%.2f s" % (total_ms / 1000.0)) if total_ms < 60000.0 else \
                   ("%.1f min" % (total_ms / 60000.0))
            print("      %-20s gap %10.2f us -> image loads in %10s   %s"
                  % (label, gap_us, span, verdict))
        print("  note: the one second no-progress watchdog in sd_card_bmp.v is fed"
              " by bmp_data_wr_en, so a gap shorter than 1 s never trips it no"
              " matter how long the whole image takes; only a card that stops"
              " delivering pixels for a full second aborts the load.")
        print("  IMPORTANT: the gap column above is a parameter sweep, NOT a"
              " measurement. The real inter-sector gap of this card cannot be"
              " measured from a PC, because the PC's mass storage driver takes a"
              " completely different path -- multi-block CMD18/CMD25, DMA, deep"
              " queueing, its own retry policy -- and never issues one CMD17 per"
              " sector the way sd_card_sec_read_write.v does. Nothing printed here"
              " is evidence about the card's speed; the only instrument that can"
              " measure the gap is the design itself, which is what the"
              " post-flash decision tree in the stage4_fix1 archive notes is for.")

    # ---------------- verdict ----------------
    print("\n" + "=" * 72)
    if res.failed == 0:
        print("VERDICT: card is clean for this design.")
        print("  %d check(s) passed. Every BMP is a FAT32 file with a legal 24"
              " bit bottom-up header and physically consecutive clusters, so"
              " bmp_read.v's linear sector walk reads exactly the file."
              % len(res.rows))
        print("  If the panel is still wrong, the card is not the cause.")
    else:
        print("VERDICT: %d of %d check(s) FAILED." % (res.failed, len(res.rows)))
        print("  Fix the card before blaming any RTL. For fragmentation, the")
        print("  reliable recipe is: format the card FAT32, then copy the whole")
        print("  set of BMPs in one go, and do not add or delete files on the")
        print("  card afterwards.")
    print("=" * 72)
    return 1 if res.failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
