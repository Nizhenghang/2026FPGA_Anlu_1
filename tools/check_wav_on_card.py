#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Replay sd_audio_stream.v against the REAL bytes on the card, at the REAL LBA.

sim_audio_stream.py proves the streamer's logic is self-consistent on synthetic
sectors. That leaves a gap this closes: whether the sector address bmp_read's
scan actually latches points at a canonical WAV header on this particular card.
If it does not, sd_audio_stream's magic check fails once, latches S_FAULT, and
the design is silent forever with no error anywhere -- exactly the "flat
visualizer, no sound" symptom.

So this walks the same two steps the hardware does:

  1. replay the root directory scan (sim_dir_scan.replay_scan) to get the
     wav_sector / wav_size that sd_card_bmp would latch into wav_found;
  2. read the sectors from that LBA onward and push them byte by byte through a
     register-level model of sd_audio_stream's S_READ datapath -- hdr_skip,
     hdr_cnt, the RIFF/WAVE magic capture, byte_phase, pcm_cnt, fifo_we.

Reports whether magic_ok fires, whether the 4-byte frame alignment survives the
header skip and every sector boundary, and the amplitude of the frames actually
handed to the FIFO, so silence is distinguishable from a broken address.

Run:  python tools/check_wav_on_card.py F: [--sectors N]
"""
import contextlib
import io
import os
import struct
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

from sim_dir_scan import Volume, replay_scan, SECTOR  # noqa: E402

HDR_LEN = 44              # sd_audio_stream.v HDR_LEN
FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    # detail explains the failure mode, so it only belongs on a FAIL line
    CHECKS[0] += 1
    tag = "ok  " if cond else "FAIL"
    suffix = "" if cond or not detail else " -- " + detail
    print("    %s  %s%s" % (tag, label, suffix))
    if not cond:
        FAILURES.append(label)
    return cond


def s16(b):
    """HDMI audio path treats these as two's-complement 16-bit samples."""
    v = b & 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def stream_sectors(sectors, pcm_total, report_frames=8):
    """Register-level replay of sd_audio_stream.v's S_READ datapath.

    `sectors` is an iterable of 512-byte payloads in the order the streamer
    requests them (LBA +1 each time). Returns a dict of what the RTL would have
    done, mirroring the non-blocking assignment semantics: every register update
    computed from a byte is applied after that byte, and the values sampled by a
    comparison are the pre-update ones.
    """
    hdr_skip = HDR_LEN
    hdr_cnt = 0
    byte_phase = 0
    pcm_cnt = 0
    magic_done = False
    magic_ok = False
    state_fault = False
    riff = [0, 0, 0, 0]
    wave = [0, 0, 0, 0]
    l_lo = l_hi = r_lo = 0
    fifo_writes = []
    phase_at_sector_end = []
    sector_index = 0

    for payload in sectors:
        if state_fault:
            break
        for byte in payload:
            if hdr_skip != 0:
                if hdr_cnt < 4:
                    riff[hdr_cnt] = byte
                elif 8 <= hdr_cnt < 12:
                    wave[hdr_cnt - 8] = byte
                hdr_cnt += 1
                hdr_skip -= 1
                # RTL: if (hdr_skip == 6'd1 && !magic_done) -- evaluated with the
                # PRE-decrement value, i.e. on the last header byte.
                if hdr_skip == 0 and not magic_done:
                    magic_done = True
                    magic_ok = (bytes(riff) == b"RIFF" and bytes(wave) == b"WAVE")
                    if not magic_ok:
                        state_fault = True
                        break
            elif pcm_cnt < pcm_total:
                if byte_phase == 0:
                    l_lo = byte
                    byte_phase = 1
                elif byte_phase == 1:
                    l_hi = byte
                    byte_phase = 2
                elif byte_phase == 2:
                    r_lo = byte
                    byte_phase = 3
                else:
                    frame = (byte << 24) | (r_lo << 16) | (l_hi << 8) | l_lo
                    fifo_writes.append(frame)
                    byte_phase = 0
                    pcm_cnt += 4
        phase_at_sector_end.append(byte_phase)
        sector_index += 1

    return {
        "magic_done": magic_done,
        "magic_ok": magic_ok,
        "riff": bytes(riff),
        "wave": bytes(wave),
        "state_fault": state_fault,
        "fifo_writes": fifo_writes,
        "pcm_cnt": pcm_cnt,
        "phase_at_sector_end": phase_at_sector_end,
        "sectors": sector_index,
    }


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    drive = argv[1]
    n_sectors = 64
    if "--sectors" in argv:
        n_sectors = int(argv[argv.index("--sectors") + 1])

    vol = Volume(drive)
    try:
        print("=" * 72)
        print("sd_audio_stream.v replay against real card bytes: %s" % drive)
        print("=" * 72)

        # Step 1: what does the scan latch? replay_scan is chatty, and its own
        # report is not what this tool is about, so swallow it.
        with contextlib.redirect_stdout(io.StringIO()):
            _found, _stopped, wav = replay_scan(vol, 4)

        print("step 1: the scan's WAV capture")
        if not check(wav is not None, "ST_SCAN_DIR latched a WAV entry",
                     "no WAV captured, wav_found stays 0, audio_phase can never set"):
            return report()
        label, clus, size, abs_lba, dsec, dent = wav
        print("    %s at directory sector %d entry %d" % (label, dsec, dent))
        print("    cluster %d, size %d bytes" % (clus, size))
        print("    wav_sector = %d absolute LBA (= volume-relative %d)"
              % (abs_lba, abs_lba - vol.hid_sec))
        print("")

        check(size > HDR_LEN + 4, "wav_usable (wav_size > HDR_LEN + 4)",
              "sd_audio_stream would go straight to S_FAULT")

        # Step 2: read the sectors the streamer will request, in order. The
        # streamer advances LBA by +1 and never follows the FAT chain, so a
        # fragmented file reads as garbage after the first run -- worth saying
        # out loud because that failure mode is also silent.
        vol_start = abs_lba - vol.hid_sec
        need = min(n_sectors, (size + SECTOR - 1) // SECTOR)
        raw = vol.read(vol_start, need)
        check(len(raw) == need * SECTOR, "read %d sector(s) from LBA %d"
              % (need, abs_lba), "short read: got %d bytes" % len(raw))
        if len(raw) != need * SECTOR:
            return report()

        print("")
        print("step 2: byte-level replay of S_READ over those sectors")
        r = stream_sectors([raw[i * SECTOR:(i + 1) * SECTOR] for i in range(need)],
                           size - HDR_LEN)
        print("    header bytes 0..3  = %r" % r["riff"])
        print("    header bytes 8..11 = %r" % r["wave"])
        check(r["magic_done"], "magic capture ran (hdr_skip reached its last byte)")
        if not check(r["magic_ok"], "RIFF/WAVE magic verified",
                     "magic_ok is false -> S_FAULT, silent forever, no retry"):
            print("")
            print("    first 64 bytes at LBA %d:" % abs_lba)
            for off in range(0, 64, 16):
                chunk = raw[off:off + 16]
                print("      +%03d  %s  %s"
                      % (off, chunk.hex(" "),
                         "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)))
            return report()

        check(not r["state_fault"], "streamer never entered S_FAULT")

        # The design's stated invariant: 44 and 512 are both multiples of 4, so
        # byte_phase returns to 0 at every sector boundary and no cross-sector
        # frame bookkeeping is needed. If a card ever broke this, samples would
        # swap channels rather than go silent -- still worth catching here.
        bad = [i for i, p in enumerate(r["phase_at_sector_end"]) if p != 0]
        check(not bad, "byte_phase == 0 at all %d sector boundary(ies)" % need,
              "misaligned after sector(s) %s" % bad[:8])

        nw = len(r["fifo_writes"])
        check(nw > 0, "fifo_we pulsed %d time(s) over %d sector(s)"
              % (nw, r["sectors"]), "no PCM frames reached the FIFO write side")
        expected = (r["sectors"] * SECTOR - HDR_LEN) // 4
        check(nw == expected, "frame count matches (sectors*512 - 44) // 4 = %d"
              % expected, "model produced %d" % nw)

        print("")
        print("step 3: are those frames actual audio, or silence/garbage?")
        show = 8
        print("    first %d frames as {R,L} (fifo_di = {R[15:0], L[15:0]}):"
              % min(show, nw))
        for i, f in enumerate(r["fifo_writes"][:show]):
            print("      [%2d] fifo_di=0x%08x  L=%6d  R=%6d"
                  % (i, f, s16(f), s16(f >> 16)))

        samples = []
        for f in r["fifo_writes"]:
            samples.append(s16(f))
            samples.append(s16(f >> 16))
        mean = sum(samples) / float(len(samples))
        rms = (sum((s - mean) ** 2 for s in samples) / float(len(samples))) ** 0.5
        peak = max(abs(min(samples)), abs(max(samples)))
        print("    over %d samples: RMS %.1f, peak %d" % (len(samples), rms, peak))
        check(rms > 50.0, "PCM is not digital silence",
              "RMS %.1f is effectively zero -- the address is right but the"
              " payload is not the song" % rms)
        check(peak <= 32768, "samples are within 16-bit range",
              "peak %d overflows, byte order is suspect" % peak)

        return report()
    finally:
        vol.close()


def report():
    print("")
    print("=" * 72)
    if FAILURES:
        print("%d of %d checks FAILED: %s"
              % (len(FAILURES), CHECKS[0], "; ".join(FAILURES)))
        return 1
    print("all %d checks passed" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
