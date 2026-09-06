import os
import sys
import glob
import shutil
import argparse

# 让本脚本无论从哪运行都能 import 同目录的 convert_audio_to_wav
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def corrupt_old_bmp_headers(target_drive):
    """
    遍历目标驱动器下的所有 .bmp 文件，将其开头的 'BM' 魔数修改为 'XX'。
    这样 FPGA 的物理扇区扫描就不会把这些“被标记为删除但物理数据还在”的文件识别为有效图片。
    """
    print(f"正在扫描 {target_drive} 驱动器，准备清除旧图片的 BMP 识别头...")
    
    # 查找根目录下的所有 bmp 文件（你也可以用 os.walk 查找所有子目录）
    search_pattern = os.path.join(target_drive, "*.bmp")
    bmp_files = glob.glob(search_pattern)
    
    corrupt_count = 0
    for bmp_file in bmp_files:
        try:
            # 以读写模式打开文件（不截断）
            with open(bmp_file, 'r+b') as f:
                header = f.read(2)
                if header == b'BM':
                    # 将指针移回开头
                    f.seek(0)
                    # 写入随意字符破坏魔数，例如 'XX'
                    f.write(b'XX')
                    corrupt_count += 1
                    print(f"  [清除有效] 已破坏头文件: {os.path.basename(bmp_file)}")
        except Exception as e:
            print(f"  [警告] 无法处理文件 {bmp_file}: {e}")
            
    print(f"头文件清除完毕，共处理了 {corrupt_count} 个旧图片文件。")

def clean_drive(target_drive):
    """
    删除目标驱动器下的所有文件，模拟格式化/清空操作（不删除隐藏系统文件夹）。
    """
    print(f"\n正在清空 {target_drive} 驱动器中的旧文件...")
    for item in os.listdir(target_drive):
        item_path = os.path.join(target_drive, item)
        # 跳过系统隐藏文件夹如 System Volume Information
        if item.startswith('.'):
            continue
            
        try:
            if os.path.isfile(item_path):
                os.remove(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
        except Exception as e:
            print(f"  [警告] 无法删除 {item_path}: {e}")
    print("驱动器清空完毕。")

def sync_new_images(source_dir, target_drive, max_count=4):
    """
    从源文件夹中挑选最多 max_count 张 BMP 图片，复制到目标驱动器。
    """
    if not os.path.exists(source_dir):
        print(f"\n[错误] 源文件夹不存在: {source_dir}")
        return

    bmp_files = glob.glob(os.path.join(source_dir, "*.bmp"))
    if not bmp_files:
        print(f"\n[错误] 源文件夹 {source_dir} 中没有找到任何 BMP 图片。")
        return

    # 按名称排序，保证顺序一致性
    bmp_files.sort()
    
    # 限制复制的数量
    files_to_copy = bmp_files[:max_count]
    
    print(f"\n准备将 {len(files_to_copy)} 张图片同步到 {target_drive}...")
    
    success_count = 0
    for i, bmp_file in enumerate(files_to_copy):
        try:
            filename = os.path.basename(bmp_file)
            # 为了让 FPGA 物理扇区尽量连续，我们在文件名前加上序号
            target_name = f"{i:02d}_{filename}"
            target_path = os.path.join(target_drive, target_name)
            
            shutil.copy2(bmp_file, target_path)
            print(f"  [同步成功] {filename} -> {target_name}")
            success_count += 1
        except Exception as e:
            print(f"  [同步失败] {filename}: {e}")
            
    print(f"\n同步完成！成功写入 {success_count} 张新图片。")
    print("你现在可以安全弹出 SD 卡，插入 FPGA 开发板了。")

def sync_audio(audio_path, target_drive):
    """
    把音频（如 MP3）离线转成标准 WAV，写到目标盘根目录 MUSIC.WAV。

    必须在写 BMP 之前调用，原因有两条，缺一不可：
      1) 目录顺序：FAT32 根目录项按创建先后排列。bmp_read 的扫描找满 4 张 BMP
         就提前 scan_done 停止，所以 WAV 的目录项必须排在那 4 张 BMP 之前才会被
         扫到。先写 MUSIC.WAV -> 它占用第一个空闲目录槽。
      2) 物理连续：读卡不跟 FAT32 簇链（地址线性 +1），假设文件物理连续。空卡
         上先写这个大文件，FAT 基本为空 -> 一次性连续分配，规避碎片。
    为最大化第 2 点的可靠性，建议先对卡做一次 FAT32 格式化再运行本工具。
    """
    import convert_audio_to_wav as cw

    if not os.path.isfile(audio_path):
        print(f"\n[错误] 找不到音频文件: {audio_path}")
        return None

    out = os.path.join(target_drive, "MUSIC.WAV")
    print(f"\n正在把音频转为标准 WAV 并【先于 BMP】写入...")
    print(f"  输入: {audio_path}")
    print(f"  目标: {out}")
    try:
        pcm_len, total_len, seconds = cw.convert_mp3_to_wav(audio_path, out)
    except Exception as e:
        print(f"  [音频失败] {e}")
        return None
    mm, ss = divmod(int(seconds), 60)
    print(f"  [音频成功] 时长 {mm:02d}:{ss:02d}  PCM {pcm_len} 字节  文件 {total_len} 字节")
    print(f"  FPGA 校验: PCM 长度 = 文件字节-44 = {total_len - cw.WAV_HEADER_LEN}")
    return out

def write_existing_wav(wav_path, target_drive):
    """
    把一个【已经做好的】标准 WAV 复制到目标盘根目录 MUSIC.WAV，同样保证【先于 BMP】写入。

    与 sync_audio 的区别：sync_audio 需要源 MP3 现场转码；本函数直接复用现成 WAV，
    用于卡上已有可用 MUSIC.WAV、但手头没有源 MP3 的情况。clean_drive 会先删掉卡上的
    WAV，所以必须先把旧 WAV 备份到卡外，再用本函数写回。先写 WAV 的两条理由与
    sync_audio 完全相同：目录项要排在 4 张 BMP 之前才会被 bmp_read 扫到；空卡先写
    这个大文件才能物理连续。
    """
    import struct

    if not os.path.isfile(wav_path):
        print(f"\n[错误] 找不到 WAV 文件: {wav_path}")
        return None

    with open(wav_path, 'rb') as f:
        head = f.read(44)
    if head[0:4] != b'RIFF' or head[8:12] != b'WAVE':
        print(f"\n[错误] {wav_path} 不是合法的 RIFF/WAVE 文件")
        return None

    channels   = struct.unpack('<H', head[22:24])[0]
    samplerate = struct.unpack('<I', head[24:28])[0]
    bits       = struct.unpack('<H', head[34:36])[0]
    pcm_len    = os.path.getsize(wav_path) - 44
    if (channels, samplerate, bits) != (2, 48000, 16):
        print(f"  [警告] 非标准格式 {samplerate}Hz/{channels}ch/{bits}bit；"
              f"FPGA 音频链按 48k/2/16 设计，可能播放异常")

    out = os.path.join(target_drive, "MUSIC.WAV")
    print(f"\n正在复用现成 WAV 并【先于 BMP】写入...")
    print(f"  输入: {wav_path}")
    print(f"  目标: {out}")
    shutil.copy2(wav_path, out)
    seconds = pcm_len / (samplerate * channels * (bits // 8))
    mm, ss = divmod(int(seconds), 60)
    print(f"  [音频成功] {samplerate}Hz/{channels}ch/{bits}bit  时长 {mm:02d}:{ss:02d}  PCM {pcm_len} 字节")
    return out

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FPGA SD卡 图片同步工具 (防物理残留死锁版)")
    parser.add_argument('drive', help='SD 卡所在的盘符 (例如: E: 或 E:\\)')
    parser.add_argument('-s', '--source', help='存放转换好 BMP 图片的源文件夹 (默认: 当前目录的 output_bmp)', default=None)
    parser.add_argument('-n', '--num', type=int, default=4, help='最多同步的图片数量 (默认: 4)')
    parser.add_argument('-a', '--audio', help='要播放的音频文件 (如 MP3)。提供则离线转成 MUSIC.WAV 并【先于 BMP】写入卡根目录', default=None)
    parser.add_argument('-w', '--wav', help='复用【现成的】标准 WAV (48k/2/16)，直接作为 MUSIC.WAV 并【先于 BMP】写入。与 -a 互斥且优先；用于卡上已有可用 WAV 但无源 MP3 的情况', default=None)
    
    args = parser.parse_args()

    # 处理盘符格式，确保以路径分隔符结尾
    target_drive = args.drive
    if not target_drive.endswith(':\\') and not target_drive.endswith(':/'):
        if target_drive.endswith(':'):
            target_drive += '\\'
        else:
            target_drive += ':\\'

    if not os.path.exists(target_drive):
        print(f"[致命错误] 找不到指定的驱动器: {target_drive}")
        print("请检查 SD 卡是否已正确插入电脑并分配了盘符。")
        sys.exit(1)

    # 确定源文件夹
    current_dir = os.path.dirname(os.path.abspath(__name__))
    source_dir = args.source if args.source else os.path.join(current_dir, "output_bmp")

    print("="*50)
    print(f"目标 SD 卡盘符: {target_drive}")
    print(f"图片源文件夹  : {source_dir}")
    print("="*50)
    
    # 为了防止误操作清空 C 盘，做个简单拦截
    if target_drive.upper().startswith('C:'):
        confirm = input("警告: 你指定了系统盘(C:)！这可能会导致系统崩溃。你确定要继续吗？(y/N): ")
        if confirm.lower() != 'y':
            print("操作已取消。")
            sys.exit(0)

    # 步骤 1: 破坏旧 BMP 的物理文件头
    corrupt_old_bmp_headers(target_drive)
    
    # 步骤 2: 清空 SD 卡（从文件系统层面）
    clean_drive(target_drive)
    
    # 步骤 3: 若指定了音频，先写 MUSIC.WAV（必须早于 BMP：目录项排序 + 物理连续）
    if args.wav:
        write_existing_wav(args.wav, target_drive)
    elif args.audio:
        sync_audio(args.audio, target_drive)

    # 步骤 4: 复制新图片
    sync_new_images(source_dir, target_drive, max_count=args.num)
