#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把任意音频（MP3 等）离线转成本工程 FPGA 可直接流式播放的标准 WAV。

为什么是 WAV 而不是 MP3：
    EG4S20 片内做真正的 MP3 解码（Huffman + IMDCT + 32 子带多相合成）不现实，
    与图片管线一致——图片是离线把 PNG/JPG 转成无压缩 BMP、FPGA 只读原始字节，
    音频同理：离线解码成无压缩 PCM，FPGA 只负责扇区流读、CDC 与采样节拍。

输出契约（FPGA RTL 依赖，勿改）：
    - 采样率 48000 Hz（HDMI 发射核固定 AUDIO_SAMPLE_RATE="48K"、ACR_N=6144）
    - 立体声、16-bit 小端、帧交错 L_lo,L_hi,R_lo,R_hi
    - 标准 44 字节 canonical 头：RIFF/WAVE/fmt (PCM)/data，头恰好 44 字节
      （手工拼头，不用 ffmpeg 的容器写出，避免它插入 LIST/INFO 等额外块，
        否则 data 块不在偏移 44，FPGA 的固定偏移解析会错位）
    - 默认文件名 MUSIC.WAV（8.3 短名，避免 LFN，扩展名固定 WAV）

本机依赖：ffmpeg（已验证 8.1.1）。Python 3 标准库即可，无需 numpy。
"""

import argparse
import os
import struct
import subprocess
import sys

SAMPLE_RATE = 48000
CHANNELS = 2
BITS_PER_SAMPLE = 16
WAV_HEADER_LEN = 44


def decode_to_raw_pcm(input_path):
    """用 ffmpeg 把输入音频解码为 48k/立体声/s16le 原始 PCM 字节，返回 bytes。"""
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"找不到输入音频: {input_path}")

    cmd = [
        "ffmpeg", "-v", "error",
        "-i", input_path,
        "-f", "s16le",          # 原始 PCM，无容器
        "-acodec", "pcm_s16le", # 16-bit 小端
        "-ar", str(SAMPLE_RATE),
        "-ac", str(CHANNELS),
        "-",                    # 输出到 stdout
    ]
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise RuntimeError("未找到 ffmpeg，请先安装并加入 PATH（本机应已装 ffmpeg 8.1.1）。")

    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 解码失败:\n{proc.stderr.decode('utf-8', 'replace')}")
    if len(proc.stdout) == 0:
        raise RuntimeError("ffmpeg 返回空 PCM，输入文件可能损坏或无音轨。")
    return proc.stdout


def build_canonical_wav(pcm_bytes):
    """给原始 PCM 拼一个恰好 44 字节的 canonical WAV 头，返回完整 WAV bytes。"""
    data_size = len(pcm_bytes)
    byte_rate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE // 8)
    block_align = CHANNELS * (BITS_PER_SAMPLE // 8)

    header = b"RIFF"
    header += struct.pack("<I", 36 + data_size)  # ChunkSize = 36 + data
    header += b"WAVE"
    header += b"fmt "
    header += struct.pack("<I", 16)              # Subchunk1Size (PCM = 16)
    header += struct.pack("<H", 1)               # AudioFormat = 1 (PCM)
    header += struct.pack("<H", CHANNELS)
    header += struct.pack("<I", SAMPLE_RATE)
    header += struct.pack("<I", byte_rate)
    header += struct.pack("<H", block_align)
    header += struct.pack("<H", BITS_PER_SAMPLE)
    header += b"data"
    header += struct.pack("<I", data_size)       # Subchunk2Size

    assert len(header) == WAV_HEADER_LEN, f"WAV 头必须恰好 {WAV_HEADER_LEN} 字节，实际 {len(header)}"
    return header + pcm_bytes


def convert_mp3_to_wav(input_path, output_path):
    """解码 + 拼头 + 写盘。返回 (pcm_len, total_len, seconds)。供 sync_to_sd.py 复用。"""
    pcm = decode_to_raw_pcm(input_path)
    # 4 字节（一个立体声帧）对齐，避免末尾半帧让 FPGA 组帧错位
    frame_bytes = CHANNELS * (BITS_PER_SAMPLE // 8)
    usable = (len(pcm) // frame_bytes) * frame_bytes
    if usable != len(pcm):
        pcm = pcm[:usable]
    wav = build_canonical_wav(pcm)

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(wav)

    byte_rate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE // 8)
    seconds = usable / float(byte_rate)
    return len(pcm), len(wav), seconds


def main():
    ap = argparse.ArgumentParser(description="MP3/音频 -> 本工程 FPGA 可流式播放的标准 WAV(48k/立体声/16bit)")
    ap.add_argument("input", help="输入音频文件（如 MP3）")
    ap.add_argument("output", nargs="?", default="MUSIC.WAV",
                    help="输出 WAV 路径或盘符（默认当前目录 MUSIC.WAV；给盘符如 F: 则写 F:\\MUSIC.WAV）")
    args = ap.parse_args()

    out = args.output
    if len(out) == 2 and out[1] == ":":  # 形如 F: 的盘符
        out = os.path.join(out + os.sep, "MUSIC.WAV")

    print(f"输入音频 : {args.input}")
    print(f"输出 WAV : {out}")
    pcm_len, total_len, seconds = convert_mp3_to_wav(args.input, out)
    mm, ss = divmod(int(seconds), 60)
    print("-" * 46)
    print(f"格式     : {SAMPLE_RATE} Hz / {CHANNELS} ch / {BITS_PER_SAMPLE}-bit PCM (canonical 44B 头)")
    print(f"时长     : {mm:02d}:{ss:02d}  ({seconds:.2f} s)")
    print(f"PCM 字节 : {pcm_len}")
    print(f"文件字节 : {total_len}  (= 44 + {pcm_len})")
    print(f"FPGA 校验: PCM 长度应为 文件字节-44 = {total_len - WAV_HEADER_LEN}")
    print("完成。可在 PC 播放器直接试听确认；再按 README 用 sync_to_sd.py 写入 TF 卡（WAV 须先写）。")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[错误] {e}", file=sys.stderr)
        sys.exit(1)
