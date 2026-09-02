# 2026 FPGA Anlu 赛题一：基于 EG4S20 的 HDMI 多媒体播放系统

本项目面向 2026 安路赛道 FPGA 赛题一，目标是在 HX4S20C 开发板上，基于安路 EG4S20 FPGA 实现一个 HDMI 多媒体播放与展示系统。当前工程已经完成 TF 卡 BMP 图片读取、SDRAM 帧缓存、HDMI 1.4b 视频显示、基础 HDMI 音频输出、按键交互、亮度调节、OSD 状态叠加、图片渐入转场、音频波形可视化与 SD 卡同步辅助工具，后续将在此基础上继续扩展图层叠加、复杂转场和更丰富的音视频联动展示能力。

## 硬件平台

- 开发板：HX4S20C
- FPGA：Anlogic EG4S20
- 显示输出：HDMI_B
- 存储介质：TF / Micro SD 卡，建议 FAT32
- 推荐显示设备：支持 HDMI 音视频输入的电视或带扬声器显示器；也可使用 HDMI 显示器加外接音箱

## 当前已实现内容

### 1. TF 卡 BMP 图片读取与扫描

- 通过 SPI 方式读取 TF 卡内容。
- 在 FAT32 卷内自动扫描 BMP 图片资源。
- 支持最多 4 张合法 BMP 图片自动发现与加载。
- 支持 640 x 480、24-bit RGB、非压缩 BMP 图片。
- 对异常图片、截断图片或非预期数据加入超时保护，避免底层状态机长时间卡死。

### 2. SDRAM 帧缓存与多缓冲显示

- 使用片上工程中的 SDRAM IP 作为帧缓存。
- 当前按 640 x 480 图像帧组织缓存空间。
- 使用多个帧缓冲区地址保存不同图片帧。
- 写入缓冲区与显示缓冲区分离，降低切图过程中的显示中断风险。
- 首图未加载完成前输出黑屏，图片提交后显示当前有效缓存。

### 3. HDMI 1.4b 视频输出

- 基于 APUG092 HDMI 1.4b Transmitter IP 输出视频。
- 当前输出分辨率为 640 x 480 @ 60 Hz。
- RGB 视频数据经 `video_rgb_to_axis_640x480` 转换为 AXI-Stream 后输入 HDMI 发射核。
- 使用 `hdmi_phy_wrapper` 将 TMDS 数据串行化并输出到 HDMI_B 差分接口。
- 上电后自动触发 EDID 读取。

### 4. HDMI 基础音频输出

- 使用 `PLL_HDMI_AUDIO` 产生 12.288 MHz 音频主时钟。
- 在 FPGA 内部生成 I2S 测试音。
- 使用 `I2S_receiver` 解串为左右声道 24-bit PCM 数据。
- 使用 `audio_arc_calculate` 生成 HDMI ACR 参数。
- 将音频样本与视频一起送入 HDMI 1.4b 发射核，实现 HDMI_B 音视频同步输出。

### 5. 按键交互与状态显示

- `key1`：手动切换到下一张已扫描到的 BMP 图片。
- `key2`：开启或关闭自动轮播。
- `key3`：循环调节显示亮度档位，范围为 `B0` 到 `B4`，默认 `B2`。
- 数码管显示 SD 卡状态码，便于调试 SD 初始化、扫描和读取流程。

### 6. 亮度调节与展示型 HDMI OSD

- 新增 `video_brightness.v`，对 RGB 三通道进行饱和加减，实现低成本亮度调节。
- 新增 `osd_overlay.v`，在 HDMI 画面左上角叠加比赛展示型信息面板。
- OSD 面板包含 `ANLOGIC MEDIA 26` 标题、图片编号、自动/手动模式、亮度档位、状态码和 HDMI AUDIO 标识。
- 面板加入边框、顶栏、亮度进度条和闪烁运行状态点，比单行调试字符更适合现场展示。
- OSD 叠加位于视频 RGB 转 AXI-Stream 之前，不影响 TF 卡读取、SDRAM 帧缓存和 HDMI 发射核结构。

### 7. 图片切换渐入转场

- 新增 `video_fade.v`，在首图提交和图片缓冲区切换时触发渐入效果。
- 渐入按视频帧推进，默认约 8 帧完成，避免按像素计数造成额外控制复杂度。
- 当前采用移位加法近似比例缩放，不引入乘法器或额外帧缓存。
- 转场位于亮度调节之后、OSD 叠加之前，图片内容渐入时 OSD 状态仍保持清晰可读。

### 8. 音频波形与频谱可视化

- 新增 `audio_visualizer.v`，采样当前 HDMI 音频链路中的左右声道 PCM 数据。
- 在画面底部绘制展示舞台，包含滚动波形、网格背景、频谱柱、峰值线和高能量闪烁点。
- 根据音频符号翻转间隔估算当前音阶频率区间，使内部音阶测试音也能形成有层次的柱状变化。
- 可视化层位于渐入转场之后、OSD 叠加之前，不改变 HDMI 音频发射路径。
- 当前测试音为 FPGA 内部生成的音阶方波，因此波形和频谱柱主要体现音阶频率变化和实时音视频联动。

### 9. 图片转换与 SD 卡同步工具

`doc/convert` 中提供了辅助脚本：

- `convert_images_to_bmp.py`：将 JPG、PNG、WebP、GIF、BMP 等常见格式转换为工程要求的 640 x 480、24-bit、非压缩 BMP。
- `sync_to_sd.py`：将转换后的 BMP 安全同步到 SD 卡，并处理旧 BMP 物理扇区残留导致 FPGA 误读的问题。

## 目录结构

```text
.
├── README.md
├── 代码说明.md
├── 设计参考例程文档.md
├── 设计参考例程文档.docx
├── doc
│   ├── APUG092_HDMI1.4b_Transmitter_V1.0.docx
│   ├── TF卡图片
│   └── convert
└── src
    ├── td_project
    └── user_source
        ├── constraints_source
        ├── hdl_source
        └── ip_source
```

主要 HDL 代码位于：

```text
src/user_source/hdl_source
```

Anlogic TD 工程文件位于：

```text
src/td_project/HDMI1.4b_Transmitter_v1.0.al
```

约束文件位于：

```text
src/user_source/constraints_source
```

## 最简复现步骤

1. 使用 FAT32 格式化 TF 卡。
2. 将 `doc/TF卡图片` 中的 BMP 示例图片复制到 TF 卡根目录，或使用 `doc/convert` 中的脚本生成并同步图片。
3. 将 TF 卡插入开发板。
4. 将 HDMI 线连接到开发板的 HDMI_B 接口，并连接显示器或电视。
5. 使用 Anlogic TD 打开 `src/td_project/HDMI1.4b_Transmitter_v1.0.al`。
6. 综合、布局布线并下载 bit 流到 FPGA。
7. 首图加载完成后，显示器应显示图片；若显示设备支持 HDMI 音频，应能听到测试音。
8. 使用 `key1` 手动切换图片，使用 `key2` 开启或关闭自动轮播，使用 `key3` 调节亮度档位；首图显示和图片切换时应能看到短暂渐入效果。

## TF 卡图片要求

```text
格式：BMP
分辨率：640 x 480
颜色：24-bit RGB
压缩：非压缩
建议文件系统：FAT32
```

注意：

- 不能直接把 PNG 或 JPG 改后缀为 `.bmp`。
- 建议使用 `doc/convert/convert_images_to_bmp.py` 统一转换图片。
- 若 FPGA 端读到旧图片或异常图片，建议使用 `doc/convert/sync_to_sd.py` 重新同步 SD 卡。

## 后续实现方向

### 1. 图层叠加与字幕增强

- 在当前展示型 OSD 基础上扩展更丰富的文字层。
- 显示时间戳、作品标语、当前图片编号、自动播放状态、参数提示等信息。
- 支持更大的字模、简单图标或中文点阵字库。

### 2. 图片轮播转场

- 在当前渐入转场基础上继续扩展淡出、交叉淡入淡出、滑动等过渡效果。
- 尽量保证切换过程画面连续、无明显撕裂或黑屏。
- 结合多缓冲机制优化转场期间的读写调度。

### 3. 图片缩放与自适应显示

- 支持读取不同分辨率 BMP 图片。
- 在 FPGA 内完成缩放或居中显示。
- 后续可比较最近邻、线性插值等不同缩放策略的资源占用与显示质量。

### 4. 实时参数调节

- 继续在当前 `key3` 亮度调节基础上扩展对比度、播放速度等参数。
- 在画面上实时叠加参数变化提示。
- 将交互控制与显示链路做成更完整的演示系统。

### 5. 音频可视化

- 在当前底部波形和频谱柱基础上继续扩展节奏提示、音量峰值保持和颜色主题切换。
- 后续可从音频样本中提取更稳定的包络或频谱特征。
- 形成音视频联动展示效果，提升赛题展示区分度。

### 6. 工程规范化

- 继续整理模块接口文档和时钟域说明。
- 增加关键模块仿真用例。
- 固化 TF 卡图片制作流程和现场演示检查清单。
- 对异常卡、异常图片、HDMI 兼容性和复位时序进行更系统的鲁棒性测试。

## 当前核心模块参考

- `top_tf_hdmi_audio.v`：系统顶层，连接 TF 卡读取、SDRAM、视频时序、HDMI 发射和音频链路。
- `SD/sd_card_bmp.v`：BMP 扫描、图片加载和切换控制。
- `SD/bmp_read.v`：BMP 扇区读取与像素解析。
- `SD/frame_read_write.v`：SDRAM 帧缓存读写控制。
- `SD/video_timing_data.v`：640 x 480 视频时序生成。
- `video_rgb_to_axis_640x480.v`：RGB/DE 视频转换为 AXI-Stream。
- `video_brightness.v`：RGB 三通道亮度档位调节。
- `video_fade.v`：图片首显和切换时的渐入转场。
- `audio_visualizer.v`：HDMI 音频样本的底部波形、频谱柱和峰值显示叠加。
- `osd_overlay.v`：HDMI 画面展示型状态面板叠加。
- `hdmi_audio_tone_i2s_64fs.v`：内部 I2S 测试音生成。
- `I2S_receiver.v`：I2S 音频解串。
- `audio_arc_calculate.v`：HDMI ACR 参数计算。

## 备注

本项目遵循赛题要求：算法、控制逻辑和数据处理流程尽量在 FPGA 内自主实现。后续如需使用外围模块，应避免引入额外处理器参与控制或算法预处理。
