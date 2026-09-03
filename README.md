# 2026 FPGA Anlu 赛题一：基于 EG4S20 的 HDMI 多媒体播放系统

本项目面向 2026 安路赛道 FPGA 赛题一，在 HX4S20C 开发板上基于安路 EG4S20 FPGA 实现一个 HDMI 多媒体播放与展示系统。

当前工程已完成 TF 卡 BMP 图片读取、**任意分辨率图片的片内最近邻缩放**、SDRAM 四缓冲帧存、HDMI 1.4b 音视频输出、按键交互、亮度调节、OSD 状态叠加、**淡入淡出与垂直擦除两种交替转场**、音频波形与频谱可视化，以及一整套无头构建脚本和周期精确验证模型。图片加载链路经过三层重试与看门狗加固，四张图片可在上电后一次性全部载入。

## 硬件平台

- 开发板：HX4S20C
- FPGA：Anlogic EG4S20BG256
- 显示输出：HDMI_B，640 x 480 @ 60 Hz
- 存储介质：TF / Micro SD 卡，FAT32
- 推荐显示设备：支持 HDMI 音视频输入的电视或带扬声器显示器；也可使用 HDMI 显示器加外接音箱

## 支持的图片格式

```text
容器：BMP（24-bit RGB，非压缩，BI_RGB）
分辨率：宽 64 ~ 1920，高 64 ~ 1080
数量：TF 卡根目录下最多 4 张，按目录项顺序取前 4 张
文件系统：FAT32（自动识别 MBR 与无 MBR 两种布局）
```

不再要求图片本身就是 640 x 480。片内缩放器会把任意受支持分辨率的图等比缩放到 640 x 480 并居中，不足处填黑边；放大倍率上限为 4 倍。

注意：

- 不能直接把 PNG 或 JPG 改后缀为 `.bmp`，必须是真正的 BMP 编码。
- 建议使用 `doc/convert/convert_images_to_bmp.py` 统一转换。
- 若卡上残留旧图片的物理扇区导致 FPGA 误读，用 `doc/convert/sync_to_sd.py` 重新同步。
- 8.3 短文件名与长文件名（LFN）目录项都能正确识别，LFN 槽位（attr 0x0F）、已删除项、卷标和子目录会被排除。

## 当前已实现内容

### 1. TF 卡 BMP 扫描与加载

- SPI 方式读取 TF 卡，高速阶段 SCK = 25 MHz（`SPI_HIGH_SPEED_DIV = 0`，即 sys_clk / 4）。
- 解析 FAT32 BPB 定位数据区与根目录，扫描根目录前 128 个扇区寻找 BMP。
- 命中后记录起始簇并换算为绝对 LBA，四张图的起始扇区存入查找表。
- 上电后一次性把四张图全部载入四个 SDRAM 帧缓冲，此后不再写入 SDRAM，因此切换图片只是索引变化、没有加载延迟。
- 四张图约 7 秒完成加载（每张 1801 个扇区）。

### 2. 片内最近邻缩放（`SD/scaler_nn.v`）

- 位于 SD 卡**写入侧**，在 `bmp_read` 像素流与帧写 FIFO 之间，SDRAM 帧缓冲保持固定 640 x 480。
- 因此下游全部不变：显示视频链路、`frame_fifo_read` 对加密 SDRAM PHY 的 `rd_delay` 假设、以及转场效果都基于固定几何尺寸工作。
- Bresenham 累加器做行列映射，1024 条目行缓冲，4096 条目弹性缓冲（skid）用于停靠源像素。
- 源流不可暂停（每 96 个 sd_card_clk 推一个像素），而目标游标是主设备；垂直重复行和黑边行都不取源像素，弹性缓冲就是为这些窗口准备的。4096 的深度覆盖了所有源几何尺寸的最坏边界。
- 升级为双线性插值只需复现同一端口列表：行缓冲变成两个源行缓冲，Bresenham 累加器变成插值权重。

### 3. 两种交替转场（`video_transition.v`）

- **淡出淡入**：按视频帧递减到全黑（默认 8 帧），在黑屏那一帧交接缓冲区索引，再递增 8 帧。只有一帧全黑，看不到硬切。
- **垂直擦除**：上半屏选择器先指向目标图、下半屏保持原图，`frame_fifo_read` 每帧把分界线向下推进 8 个两行组，从面板顶部逐次揭示新图；240 / 8 = 30 帧走完，选择器保持分离 40 帧后再合并。
- 两种效果每次换图交替使用，所以四张图轮播一遍就能看到两种转场。`mode_wipe` 复位为 0，第一次转场是淡入淡出。
- 擦除期间同一帧要读两个缓冲区，这之所以安全，是因为四张图早已全部载入、此后没有任何写入者。

### 4. SDRAM 帧缓存与多缓冲

- 使用 SDRAM 硬核 `EG_PHY_SDRAM_2M_32`（2M x 32-bit = 8 MB）作为帧缓存。
- 四个帧缓冲各 307200 字，基址 0 / 307200 / 614400 / 921600，合计 1228800 字。
- 写入缓冲与显示缓冲分离；`frame_fifo_write` 做行翻转（`WRITE_V_FLIP`）以匹配 BMP 自底向上的行序。
- 首图提交前输出黑屏。

### 5. HDMI 1.4b 音视频输出

- 基于 APUG092 HDMI 1.4b Transmitter IP，输出 640 x 480 @ 60 Hz（VIC 1）。
- RGB 经 `video_rgb_to_axis_640x480` 转为 AXI-Stream 后送入发射核，`hdmi_phy_warpper` 串行化输出到 HDMI_B 差分接口。
- `PLL_HDMI_AUDIO` 产生 12.288 MHz 音频主时钟，片内生成 I2S 测试音，`I2S_receiver` 解串为左右声道 24-bit PCM，`audio_arc_calculate` 生成 ACR 参数，音视频一并送入发射核。
- 上电后自动触发 EDID 读取。

### 6. 按键交互与 OSD

- `key1`：手动切换到下一张已载入的图片。
- `key2`：开启 / 关闭自动轮播（间隔 1 秒，需已载入 2 张以上）。
- `key3`：循环调节亮度档位 `B0` ~ `B4`，默认 `B2`。
- `osd_overlay.v` 在画面左上角叠加展示型面板：`ANLOGIC MEDIA 26` 标题、`IMG:n` 图片编号、自动 / 手动模式、亮度进度条、SD 状态码和 HDMI AUDIO 标识，并带边框、顶栏和闪烁运行点。内置 8x8 点阵字模，不占用外部 ROM。
- 数码管同步显示 SD 卡状态码，便于调试初始化、扫描和读取流程。
- OSD 叠加位于视频转 AXI-Stream 之前，不影响 TF 卡读取、帧缓存和发射核结构。

### 7. 亮度、音频可视化与视频链路顺序

- `video_brightness.v` 对 RGB 三通道做饱和加减，移位加法实现，不引入乘法器。
- `audio_visualizer.v` 采样 HDMI 音频链路的左右声道 PCM，在画面底部绘制滚动波形、网格背景、频谱柱、峰值线和高能量闪烁点；按符号翻转间隔估算音阶频率区间。
- 视频链路顺序：帧缓存读出 → 转场 → 亮度 → 音频可视化 → OSD → RGB 转 AXI-Stream → HDMI 发射核。因此图片渐入时 OSD 状态始终清晰可读。

### 8. 加载链路鲁棒性

三层独立的重试与看门狗，覆盖从单个扇区到整张图的不同粒度：

| 层级 | 位置 | 机制 |
|---|---|---|
| 扇区级 | `sd_card_sec_read_write.v` | 起始令牌丢失时重试，`RD_RETRY_MAX = 2`（共 3 次），最坏静默约 300 ms |
| 缩放器 | `scaler_nn.v` | `fill_wait` 饱和看门狗，`FILL_WAIT_AW = 27`，约 671 ms |
| 图片级 | `sd_card_bmp.v` | `LOAD_MAX_RETRY = 3`（共 4 次尝试），外加 1 秒无进度看门狗 |

图片级重试是关键：`next_load_idx` 在加载**开始**时自增，而 `img_loaded_count` 只在**成功**时自增。若一次加载失败而不回退，该索引就被永久消耗、那张图再也不会出现。现在 `load_failed` 与停顿看门狗统一为 `load_gave_up`，把 `next_load_idx` 回退一格，用同一缓冲重读同一文件；四张图各自最多尝试 4 次，仍失败才放弃。

三个时间常数必须保持递增关系：扇区级 300 ms < 缩放器 671 ms < 图片级 1000 ms。缩放器看门狗若短于扇区级最坏静默窗口，会在重试仍在进行时截断目标行，表现为画面花屏。

## 目录结构

```text
.
├── README.md
├── .gitignore
├── doc
│   ├── APUG092_HDMI1.4b_Transmitter_V1.0.docx
│   ├── TF卡图片
│   └── convert                 图片转换脚本、测试图组与期望上屏效果
├── tools                       构建脚本、周期精确验证模型与诊断工具
└── src
    ├── td_project
    │   ├── HDMI1.4b_Transmitter_v1.0.al      TD 工程文件
    │   └── HDMI1.4b_Transmitter_v1.0_Runs
    │       └── best_result                   烧录用比特流
    └── user_source
        ├── constraints_source  timing.sdc / pin.adc
        ├── hdl_source          全部 RTL
        └── ip_source
```

TD 的 `syn_1` / `phy_1` 运行目录和会话日志不入库，它们可由 `.al` 工程经 `tools/td_build.ps1` 完整重新生成；只保留 `best_result/`，因为那才是真正烧进板子的比特流。

## 构建与烧录

### 无头构建

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/td_build.ps1 -Stage all
```

`-Stage` 可取 `all` / `syn` / `phy`。脚本驱动 `td_commands_prompt.exe` 与 TD 自带的 `DefaultFlow.tcl`，经 `tools/td_flow_exit.tcl` 退出，并做三件容易被忽略的事：判定真实的成败（PowerShell 成功流会被捕获成 `Object[]`，`if ($ok)` 恒真）、检查产物是否本次新产出、失败时抑制上一轮的 QoR 摘要。

### 烧录路径取决于构建方式

- **TD GUI 构建**：GUI 会把物理层比特流提升到 `best_result/`，两处都可以烧。
- **无头 `td_build.ps1` 构建**：**不会**提升，`best_result/` 保持陈旧。此时必须烧 `phy_1/HDMI1.4b_Transmitter_v1.0.bit`。

烧错路径的症状是"改完没有任何变化"，很容易把排查引向错误方向。

### 时序与资源（最近一次构建实测）

Slow / Fast 两个 corner 全部收敛，违例端点 0，全局 Setup WNS +0.712 ns、Hold WNS +0.003 ns。

| 时钟 | 约束 | 实测 fmax | SWNS |
|---|---|---|---|
| `sd_card_clk` | 100 MHz | 108.225 MHz | +0.760 ns |
| `ext_mem_clk` | 125 MHz | 137.212 MHz | +0.712 ns |
| `video_clk` | 25 MHz | 32.670 MHz | +3.792 ns |
| `clk` | 50 MHz | 81.739 MHz | +3.883 ns |
| `hdmi_5x_clk` | 125 MHz | 307.220 MHz | +4.745 ns |

余量最紧的是 `sd_card_clk`（8.2%），图片级重试逻辑正好在这个域。资源占用 5059 slices（51.62%）、34 个 RAM、0 个 DSP。

`timing.sdc` 中对 SDRAM 硬核 DQ 边界写了 `set_max_delay -datapath_only` 例外：这些路径全在加密 IP 内部、fabric 与 PHY 之间没有用户逻辑，安路也未随该 IP 附带 `.tcl` 约束，不做例外时它们贡献总 TNS 的 87% 伪违例。

## 验证工具

本机没有 iverilog / verilator，因此控制流逻辑用 `tools/` 下的周期精确 Python 模型验证，再上板确认。

| 脚本 | 用途 |
|---|---|
| `sim_load_retry.py` | 图片加载调度器 + 最小 `bmp_read`。含负对照：同一次瞬时失败跑改动前的调度器，复现"卡里四张、只显示三张"的原始现象 |
| `sim_scaler_nn.py` | 缩放器时序与弹性缓冲峰值占用 |
| `sim_dir_scan.py` | 按字节重放根目录扫描，直读 TF 卡，证明扫描器确实记录到全部 BMP |
| `sim_sd_retry.py` / `sim_transition.py` | 扇区级重试、转场状态机 |
| `cmp_bit.py` | 比较两个比特流的配置体。ASCII 头带分钟级 `# Date:`，所以未改动设计的重编也不是逐字节相同，整文件哈希比较必然误报 |
| `td_build.ps1` / `td_flow_exit.tcl` | 无头构建 |
| `gen_test_bmp.py` / `check_sd_card.py` / `probe_retry_trace.py` / `render_defect_preview.py` | 测试图生成与卡上诊断 |

模型的可信度取决于是否忠实镜像 RTL 语义，两个踩过的坑：右值必须只读周期开始前的快照（否则模型能做硬件做不到的事，比如在同一周期既判失败又发起新加载）；默认值要按 RTL 的位置写（RTL 在 `else` 分支开头默认清零、由后面的赋值覆盖，模型若在别处默认就把要验证的语义当成前提了）。

`doc/convert/` 下有两组测试图与对应的期望上屏效果：`setA_fill/` 覆盖 320x240、640x480、800x600、1920x1080；`setB_border/` 覆盖 64x64、100x100、159x119、1920x64 这类极端宽高比，用于验收缩放与黑边居中。

## 最简复现步骤

1. 用 FAT32 格式化 TF 卡。
2. 把 `doc/TF卡图片` 中的示例 BMP 复制到卡根目录，或用 `doc/convert` 的脚本生成并同步自己的图片。
3. 插入 TF 卡，HDMI 线接到开发板 HDMI_B。
4. 用 Anlogic TD 打开 `src/td_project/HDMI1.4b_Transmitter_v1.0.al`，综合、布局布线并下载；仓库里 `best_result/` 的比特流是最近一次 GUI 构建的产物，未改 RTL 时可以直接烧。若用无头脚本重新构建过，按「构建与烧录」一节的路径规则选择比特流。
5. 首图加载完成后显示器应出现图片，数码管状态码停止变化；约 7 秒后四张图全部载入。
6. `key1` 手动切换，`key2` 开关自动轮播，`key3` 调节亮度。切换时应交替看到淡入淡出和自上而下的擦除效果。

## 后续实现方向

### 1. 缩放质量

- 当前是最近邻。升级为双线性插值：行缓冲改为两个源行缓冲，Bresenham 累加器改为插值权重，端口列表可以完全不变。
- 比较不同缩放策略的资源占用与显示质量。

### 2. 转场效果扩展

- 已有淡入淡出与垂直擦除。可继续扩展交叉淡入淡出、水平滑动、百叶窗等。
- 多缓冲已就位，擦除期间读两个缓冲区是安全的，新效果可以复用这一前提。

### 3. 图层叠加与字幕增强

- 在现有 OSD 基础上扩展更丰富的文字层：时间戳、作品标语、参数提示。
- 支持更大地字号、简单图标或中文点阵字库。

### 4. 实时参数调节

- 在亮度之外扩展对比度、轮播速度等参数，并在画面上实时叠加提示。

### 5. 音频可视化

- 在底部波形和频谱柱基础上扩展节奏提示、音量峰值保持和颜色主题切换。
- 从音频样本中提取更稳定的包络或频谱特征。

### 6. 工程规范化

- 补齐模块接口文档和时钟域说明。
- 把关键模块的 Python 模型沉淀为更完整的回归用例。
- 对异常卡、异常图片、HDMI 兼容性和复位时序做更系统的鲁棒性测试。

## 当前核心模块参考

- `top_tf_hdmi_audio.v`：系统顶层，连接 TF 卡读取、SDRAM、视频时序、HDMI 发射和音频链路。
- `SD/sd_card_bmp.v`：BMP 扫描调度、图片加载与重试、切换控制。
- `SD/bmp_read.v`：FAT32 解析、根目录扫描、BMP 头校验与像素流输出。
- `SD/scaler_nn.v`：最近邻缩放，把任意受支持分辨率缩放到 640 x 480 并居中。
- `SD/sd_card_sec_read_write.v`：扇区级 SPI 读，含起始令牌重试。
- `SD/frame_fifo_write.v` / `SD/frame_fifo_read.v` / `SD/frame_read_write.v`：SDRAM 帧缓存读写与擦除分界线推进。
- `SD/video_timing_data.v`：640 x 480 视频时序生成。
- `video_transition.v`：淡入淡出与垂直擦除两种交替转场，决定面板何时看到缓冲区切换。
- `video_brightness.v`：RGB 三通道亮度档位调节。
- `video_fade.v`：按转场给出的电平做比例缩放。
- `audio_visualizer.v`：底部波形、频谱柱和峰值显示叠加。
- `osd_overlay.v`：展示型状态面板叠加，内置 8x8 字模。
- `video_rgb_to_axis_640x480.v`：RGB/DE 转 AXI-Stream。
- `hdmi_audio_tone_i2s_64fs.v` / `I2S_receiver.v` / `audio_arc_calculate.v`：I2S 测试音、解串与 ACR 参数。

## 备注

本项目遵循赛题要求：算法、控制逻辑和数据处理流程均在 FPGA 内自主实现，未引入额外处理器参与控制或算法预处理。`doc/convert` 与 `tools/` 下的 Python 脚本只用于离线制作测试素材、驱动构建和验证逻辑，不参与板上的实时数据通路。
