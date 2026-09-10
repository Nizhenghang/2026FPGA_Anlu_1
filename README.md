# 2026 FPGA Anlu 赛题一：基于 EG4S20 的 HDMI 多媒体播放系统

本项目面向 2026 安路赛道 FPGA 赛题一，在 HX4S20C 开发板上基于安路 EG4S20 FPGA 实现一个 HDMI 多媒体播放与展示系统。

当前工程已完成 TF 卡 BMP 图片读取、**任意分辨率图片的片内最近邻缩放**、SDRAM 四缓冲帧存、HDMI 1.4b 音视频输出、**TF 卡 WAV 真实音乐流式播放（单曲循环）**、按键交互、亮度调节、OSD 状态叠加、**画面正中的中文滚动字幕（竞赛十周年标语，`SW4` 可屏蔽）**、**淡入淡出与一族横向带状转场（擦除 / 百叶窗 / 中心展开 / 随机条 / 梳状，可自动轮播）**、音频波形与频谱可视化、**串口屏（淘晶驰 TJC/Nextion）经板载 Type-C UART 控制（替代全部按键与拨码，实体控制保留为兜底）**，以及一整套无头构建脚本和周期精确验证模型。图片加载链路经过三层重试与看门狗加固，四张图片可在上电后一次性全部载入；图片载入完成后自动接管 SD 总线流式播放音乐，底部可视化随真实音乐跳动。

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

不再要求图片本身就是 640 x 480。片内缩放器把任意受支持分辨率的图映射到 640 x 480 画布并居中，不足处填黑边；放大倍率上限为 4 倍。

缩放**按横纵两轴各自独立**计算，不是严格等比：目标尺寸取 `tw = min(源宽 × 4, 640)`、`th = min(源高 × 4, 480)`（见 `SD/scaler_nn.v`），两轴之间没有共享的缩放系数。是否变形只取决于源图：

- 源是 4:3（320x240、640x480、800x600 等）→ 两轴同比，**不变形**，铺满画面；
- 源很小、两轴都没到 4 倍上限（源宽 < 160 且源高 < 120，如 64x64、100x100、159x119）→ 两轴统一 ×4，**保持宽高比**，四周留黑边；
- 其余（宽高比非 4:3 且至少一轴被钳到上限，如 1920x1080、1920x64）→ **会被拉伸 / 压扁**。

要确保不变形，请用 4:3 源图，或用两轴都小于 160x120 的小图。

注意：

- 不能直接把 PNG 或 JPG 改后缀为 `.bmp`，必须是真正的 BMP 编码。
- 建议使用 `doc/convert/convert_images_to_bmp.py` 统一转换。
- 若卡上残留旧图片的物理扇区导致 FPGA 误读，用 `doc/convert/sync_to_sd.py` 重新同步。
- 8.3 短文件名与长文件名（LFN）目录项都能正确识别，LFN 槽位（attr 0x0F）、已删除项、卷标和子目录会被排除。

## 支持的音频格式

```text
容器：WAV（标准 44 字节 canonical 头，RIFF/WAVE/fmt (PCM)/data）
采样：48000 Hz、立体声、16-bit 小端 PCM，帧交错 L_lo,L_hi,R_lo,R_hi
文件：TF 卡根目录 MUSIC.WAV（8.3 短名，扩展名固定 WAV）
```

FPGA 不在片内解码 MP3——与图片管线一致（图片离线转无压缩 BMP、FPGA 只读原始字节），音频也离线用 ffmpeg 解码成无压缩 PCM，FPGA 只负责扇区流读、跨时钟域、48 kHz 采样节拍与单曲循环。

注意：

- 用 `doc/convert/convert_audio_to_wav.py INPUT.mp3 [输出路径]` 把任意音频离线转成符合上述契约的 `MUSIC.WAV`（手工拼 44 字节头，不用 ffmpeg 的容器写出，避免插入 LIST/INFO 块导致 data 偏移错位）。
- 写卡用 `doc/convert/sync_to_sd.py F: --audio INPUT.mp3`：流程为「破坏旧 BMP 头 → 清空盘 → **先写 MUSIC.WAV** → 再按序号写 4 张 BMP」。WAV 先写有两个作用：① FAT32 根目录项按创建顺序排列，WAV 目录项排在 BMP 之前，`bmp_read` 在找满 4 张 BMP 提前停止前已扫到 WAV；② 空卡先写大文件更容易物理连续。
- 卡上已有可用 `MUSIC.WAV` 但手头没有源 MP3 时，用 `doc/convert/sync_to_sd.py F: --wav 现成.wav` 直接复用：流程与 `--audio` 完全相同（清空盘 → **先写 MUSIC.WAV** → 再写 4 张 BMP），只是跳过转码、原样拷贝。`--wav` 与 `--audio` 互斥且优先。清空盘会删掉卡上原有 WAV，**务必先把旧 WAV 备份到卡外**再用 `--wav` 写回。
- **强烈建议先把卡 FAT32 格式化再同步**：读卡逻辑不跟随 FAT32 簇链、假设文件物理连续（地址线性 +1），约 40 MB 的 WAV 一旦碎片化就会读到错乱数据。清空后的空卡先写 WAV 通常连续，但格式化能彻底保证。
- 卡上找不到有效 WAV（或 RIFF/WAVE 校验失败）时输出静音，图片照常轮播，不回退测试音。

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
- 几何映射是**非等比**的：`tw = min(源宽 × 4, 640)`、`th = min(源高 × 4, 480)`、`off = ((640 − tw)/2, (480 − th)/2)`，横纵两轴各自独立钳位、不共享系数。只有「源为 4:3」或「源宽 < 160 且源高 < 120（两轴统一 ×4）」时不变形，其余宽高比会被拉伸/压扁——这是当前实现的已知取舍，规则详见「支持的图片格式」一节。
- Bresenham 累加器做行列映射，1024 条目行缓冲，4096 条目弹性缓冲（skid）用于停靠源像素。
- 源流不可暂停（每 96 个 sd_card_clk 推一个像素），而目标游标是主设备；垂直重复行和黑边行都不取源像素，弹性缓冲就是为这些窗口准备的。4096 的深度覆盖了所有源几何尺寸的最坏边界。
- 升级为双线性插值只需复现同一端口列表：行缓冲变成两个源行缓冲，Bresenham 累加器变成插值权重。

### 3. 转场特效：淡入淡出 + 横向带状家族（`video_transition.v` / `SD/frame_fifo_read.v`）

转场分两类。`video_transition.v`（video_clk 域）决定面板**何时、以哪种效果**看到缓冲区切换，输出两个缓冲选择器、淡入淡出电平和 3 位带状特效码 `O_effect`；`frame_fifo_read.v`（ext_mem_clk 域）把特效码变成逐组的缓冲选择。

- **淡入淡出（非带状）**：按视频帧递减到全黑（默认 8 帧），在黑屏那一帧交接缓冲区索引，再递增 8 帧。只有一帧全黑，看不到硬切。此时两个选择器相等、`O_effect=0`，`frame_fifo_read` 的带状引擎完全失活。
- **横向带状家族**：上半屏选择器指向目标图、下半屏保持原图，`O_effect` 携带 1~6 的特效应码。`frame_fifo_read` 见两个选择器分离即冻结特效码，并每帧把 ramp 推进 8 个「两行组」；其 `select_top(g, progress, effect)` 函数把 ramp 位置 + 特效码翻译成**每个组**读哪个缓冲，于是新图以擦除 / 百叶窗 / 中心展开 / 随机条 / 梳状的形态沿垂直方向扫入。六种效果共用同一套 30 帧 ramp 和同一个「组边界重定向」机制——它们就是把旧的单次擦除从「一帧只跨一次」推广到「每个组边界按几何决定是否跨」。选择器分离保持 36 帧（ramp 30 帧走完再留 6 帧满屏新图）后合并，合并这一相等条件同时复位引擎，为下一次转场做准备。

  | 码 | 特效 | `select_top(g, progress)` 几何 |
  |---|---|---|
  | 1 | 擦除（向下） | `g < progress`，新图自顶向下扫入（即旧的单次擦除） |
  | 2 | 擦除（向上） | `g >= 240 - progress`，新图自底向上扫入 |
  | 3 | 百叶窗 | 15 条叶片（每 16 组一条）同时填充 |
  | 4 | 中心展开 | 自中线（组 120）向上下两边对称展开 |
  | 5 | 随机条 | 组索引位反转后乱序填充 |
  | 6 | 梳状 | 偶数组向下擦、奇数组向上擦，交错进行 |

- **关键不变量**：一个组 = 1280 字 = 5 个 256 字突发 = 2 行；一帧 240 组。重定向**只在组边界**触发，因此绝不会把一个突发劈到两个缓冲区，帧内字偏移始终连续（组 g 第 w 字地址 = `base_sel(g) + g*1280 + w`）。每次缓冲翻转地址跳变 `±wipe_delta`，`neg_wipe_delta` 预算好，使重定向是 2:1 mux 而非减法器；`select_top` 的组合结果先寄存成 `next_sel_r` 再喂地址 mux，保证它不进入 ext_mem_clk 地址寄存器的关键路径。所有几何只用比较 / 加减 / 常数移位 / 位掩码 / 位反转，**无除法器、无 DSP**。
- **3 位模式 `I_mode`**：`000` 自动轮播（`effect_cnt` 每次换图轮换 淡入淡出→擦除↓→擦除↑→百叶窗→中心展开→随机条→梳状，复位为 7 使第一次是淡入淡出）；`001`~`110` 固定一种带状特效；`111` 固定淡入淡出。特效在每次转场**开始时采样一次**，飞行中拨动拨码不会撕裂当前转场。详见第 6 节。
- 带状特效期间同一帧要读两个缓冲区，这之所以安全，是因为四张图早已全部载入、此后没有任何写入者。

### 4. SDRAM 帧缓存与多缓冲

- 使用 SDRAM 硬核 `EG_PHY_SDRAM_2M_32`（2M x 32-bit = 8 MB）作为帧缓存。
- 四个帧缓冲各 307200 字，基址 0 / 307200 / 614400 / 921600，合计 1228800 字。
- 写入缓冲与显示缓冲分离；`frame_fifo_write` 做行翻转（`WRITE_V_FLIP`）以匹配 BMP 自底向上的行序。
- 首图提交前输出黑屏。

### 5. HDMI 1.4b 音视频输出

- 基于 APUG092 HDMI 1.4b Transmitter IP，输出 640 x 480 @ 60 Hz（VIC 1）。
- RGB 经 `video_rgb_to_axis_640x480` 转为 AXI-Stream 后送入发射核，`hdmi_phy_warpper` 串行化输出到 HDMI_B 差分接口。
- 音频不再是片内合成测试音，而是 TF 卡真实音乐：四张图载入完成后 `sd_card_bmp` 把 SD 扇区读总线交给 `sd_audio_stream`，流式读取 `MUSIC.WAV`（跳过 44 字节头、按 4 字节组帧 {R,L}、放完回卷单曲循环），经异步 FIFO（`wfifo_32_32_512`）跨到 video_clk，`audio_pcm_player` 用小数分频产生 48 kHz 节拍把 16-bit PCM 左对齐成 24-bit 送入 `audio_arc_calculate`（ACR）、`audio_visualizer` 与发射核。欠载或无 WAV 时仍持续打 `audio_valid`（数据填 0）以保证 ACR 不断、HDMI 音频锁定。
- `PLL_HDMI_AUDIO` 仍保留（复位树依赖其 lock），但 12.288 MHz 音频主时钟与 `hdmi_audio_tone_i2s_64fs.v` / `I2S_receiver.v` 已不再例化，文件保留在仓库便于调试回挂。
- 上电后自动触发 EDID 读取。

### 6. 按键、拨码交互、OSD 与滚动字幕

- `key1`：手动切换到下一张已载入的图片。
- `key2`：开启 / 关闭自动轮播（间隔 1 秒，需已载入 2 张以上）。
- `key3`：循环调节亮度档位 `B0` ~ `B4`，默认 `B2`。
- 拨码开关 `sw[3:0]`（`SW1`~`SW4` = `C8`/`C7`/`C6`/`C5`，`PULLUP` 输入）：`SW1`/`SW2`/`SW3` 选择转场特效，`SW4` 屏蔽滚动字幕。拨码为低有效（ON 接地 = 0），RTL 同步后取反成 ON=1 的直观逻辑，即物理项 `~sw_v1[2:0]`；模式在每次转场**开始时**采样，拨动后从**下一张图**生效，不会打断进行中的转场。全 OFF（上电默认）即 `000` 自动轮播 + 字幕显示，开箱就能依次演示整族特效。**自第 7 节起，`trans_mode` / `marquee_en` 是「屏幕覆盖 mux」的物理兜底分支**——`trans_mode = mode_ovr_en_v1 ? mode_ovr_val_v1 : ~sw_v1`、`marquee_en = marq_ovr_en_v1 ? marq_ovr_val_v1 : sw4_v1`；屏幕发过 `MODE`/`MARQ` 命令时由覆盖值驱动、拨码一变动即夺回，从未发命令时 `ovr_en=0`，下表逐位不变。

  | SW3 | SW2 | SW1 | `trans_mode` | 转场特效 |
  |---|---|---|---|---|
  | OFF | OFF | OFF | `000` | 自动轮播（默认）：淡入淡出 → 擦除↓ → 擦除↑ → 百叶窗 → 中心展开 → 随机条 → 梳状，每次换图轮换一种 |
  | OFF | OFF | ON  | `001` | 擦除（向下，新图自顶向下） |
  | OFF | ON  | OFF | `010` | 擦除（向上，新图自底向上） |
  | OFF | ON  | ON  | `011` | 百叶窗（15 条叶片同时填充） |
  | ON  | OFF | OFF | `100` | 中心展开（自中线向上下两边） |
  | ON  | OFF | ON  | `101` | 随机条（乱序填充） |
  | ON  | ON  | OFF | `110` | 梳状（奇偶组反向交错） |
  | ON  | ON  | ON  | `111` | 淡入淡出过黑 |

- `SW4`（`sw[3]`）是**滚动字幕屏蔽开关**，极性与 `SW1`~`SW3` **刻意相反**：`OFF`（上电默认，`PULLUP` 读 1）= 字幕显示，`ON` = 字幕隐藏。反着来的理由是它是屏蔽键（类似静音）而不是模式选择位——这样「全 OFF 开箱即演示」不被破坏，评委面前不需要先记得拨一下；拨 `ON` 也是一条一行就能退回无字幕已验证画面的开关。RTL 上它走**独立**的 `sw4_v0`/`sw4_v1` 两级同步器，物理项 `sw_v0 <= sw[2:0]` 与 `~sw_v1` / `sw4_v1` 一位未动；第 7 节的屏幕覆盖只是叠加在它们之后的并联 mux，`ovr_en=0` 时转场特效与字幕路径与改前完全等价。
- `marquee_overlay.v` 在画面正中（y 224..255，共 32 行）叠加一条横向滚动字幕，内容是「FPGA创新设计竞赛国赛十周年」。15 个 24x24 字模单元排在 32 px 节距上（左右各 4 px 字间距），文字总宽 480 px，行程 `TRAVEL = 640 + 480 = 1120` px：自右侧屏外进入、左侧屏外退出后从头循环；`SCROLL_FRAME_DIV = 2` 帧走 1 px，在实测 59.52 Hz 帧率下即 29.76 px/s、单圈 37.6 s，24 px 高的字读起来舒适。横带上下各 1 px 青色描边（`24'h60D8FF`，与 OSD 边框同色），带内把底图压暗 `>>2` 再写亮字（`24'hFFE878`），最亮与最暗背景下都读得清。
- 字幕的全部逻辑都落在 `video_clk` 域，不碰 `sd_card_clk` / `ext_mem_clk` 那两条紧路径。两个刻意的设计选择：① 字模表是 360 项 `case` 的**纯组合 LUT 逻辑**而非同步 BRAM，后者会逼整个叠加层多打 1 像素流水；② 寻址不用除法器——`u = x_pos + marq_pos - 640` 在 11 bit 线上借位回绕，一条 `u < 480` 无符号比较就同时覆盖「尚未进场」与「已完全离场」两端，`cell_idx = u[9:5]`、`col = u[4:0]` 直接是 32 px 节距的商与余数。中文字模由 `tools/gen_marquee_font.py`（PIL + 黑体 @26 px，墨迹 bbox 22x22 原生渲染、不经缩放重采样）生成 `marquee_font.vh`，经 `` `include `` 进入模块体，**不注册进 `.al`**（裸 `function` 无法独立编译，注册会直接语法报错）。
- `osd_overlay.v` 在画面左上角叠加展示型面板：`ANLOGIC MEDIA 26` 标题、`IMG:n` 图片编号、自动 / 手动模式、亮度进度条、SD 状态码和 HDMI AUDIO 标识，并带边框、顶栏和闪烁运行点。内置 8x8 点阵字模，不占用外部 ROM。
- 数码管 6 位全部启用，从左到右依次是：**已载入图片张数** `img_loaded_count`、**加载失败原因** `fail`、**扫描找到的 BMP 张数** `img_found_count`、**下一个加载序号** `next_load_idx`、**音频链路位 `chain`**、**SD 状态码**（最右一位就是原先单独显示状态码的那一位）。

  后五位是为定位「音乐不出声」而加的诊断口：卡布局、目录扫描、流读数据通路、异步 FIFO IP、时钟频率、综合网表全部离线验证通过后，断点只能靠上板读数定位。`chain` = `{wav_found, audio_phase, ever_we, fault}`：

  | `chain` | 含义 | 下一步查哪里 |
  |---|---|---|
  | `0` | 目录扫描根本没找到 WAV | 卡上 `MUSIC.WAV` 的目录项是否排在 4 张 BMP 之前，`tools/sim_dir_scan.py F:` 复现 |
  | `8` | 找到 WAV，但 `audio_phase` 始终没置起 | 先看最左张数：不到 `4` 说明图没载完，交棒条件里的 `img_loaded_count >= SCAN_TARGET_COUNT` 就是卡住的那一项，按下面一行继续分；张数为 `4` 才查 `bmp_ready` / `load_busy` |
  | `C` | 已交棒，流读器却一次都没写 FIFO | SD 扇区读总线仲裁、`sd_sec_read` 握手 |
  | `D` | 流读器进入 `S_FAULT` | RIFF/WAVE 魔数校验失败或文件太小，`tools/check_wav_on_card.py F:` 复现 |
  | `E` | 流读器正常写 FIFO | 断点在下游：`audio_pcm_player` / ACR / 发射核 |

  张数不足 4 时用第 2~4 位分流：`found < 4` → 扫描漏了目录项（查 LFN / 属性 / 簇号判定）；`found == 4` 且 `next == 4` → 第四张加载尝试 4 次后被放弃，`fail` 的 bit3 = 1 秒无进度看门狗、bit2 = 头扇区被 `header_match_r` 拒绝、bit1:0 是读到的重试计数；`next < 4` 且 `fail == 0` → 加载根本没被再次武装，查武装条件。
- OSD 与滚动字幕两个叠加层都位于视频转 AXI-Stream 之前，不影响 TF 卡读取、帧缓存和发射核结构。

### 7. 串口屏控制（淘晶驰 TJC/Nextion，替代按键与拨码，保留兜底）

板载 Type-C 串口屏经 UART 成为主控制面，替代 `key1`/`key2`/`key3` 与 `SW1`~`SW4` 的全部功能，并扩展出直接选图 / 直接设亮度 / 直接选特效。**实体按键与拨码完整保留为兜底**：屏幕没接、没上电或故障时，板上交互与改动前逐位一致。合并策略是 last-writer-wins，两端互为退路。

RTL 全部在 `uart_screen_ctrl.v`（clk 域，UART RX + TJC 命令解析）与 `top_tf_hdmi_audio.v`（跨时钟域注入 + 与物理控制合并）里；`sd_card_bmp.v` 仍是单时钟模块，命令脉冲在 top 里 CDC 之后才接进去。

#### 接线

- **首选：板载 Type-C / CH340**。`uart_rx = F12`（FPGA 输入，`PULLUP`）、`uart_tx = D12`（FPGA 输出），LVCMOS33，经板载 CH340 USB-UART 接到 Type-C 口，用一根 C-to-C 线连屏幕与板子。
  - **电气前提**：板子这端 CH340 是 USB **从设备**，要通信，串口屏那端必须是 USB **Host** 才能枚举它。多数淘晶驰屏的 Type-C 是给 PC 编程用的从口——若上板发现 C-to-C 直连枚举不通，走下面的退路。
- **退路：2×40 GPIO 排针 TTL 飞线**。把屏幕的 TTL 串口线（TX/RX/GND）直接接到排针上两个空闲 IO，绕开 CH340，**必须共地**。此路 **RTL 完全不变**，只改 `pin.adc` 里 `uart_rx`/`uart_tx` 两行的 `LOCATION`（一行切换）。

#### 协议

- **波特率 9600**（= 淘晶驰出厂默认，开箱即通、无需改屏幕工程），8N1，LSB first。FPGA 侧 `BAUD` 是参数；如需提速，改 FPGA 参数 + 改屏幕工程波特率两端对齐即可（50 MHz / 9600 = 5208，误差 0.006%，远小于 UART 容限）。
- **帧格式**：4 字符关键字 +（可选）`空格 + 1 位数字参数`，以淘晶驰惯例的**连续 3 个 `0xFF`** 结尾。负载字节永远是 ASCII（不会是 `0xFF`），所以 `0xFF` 唯一地表示终止符；FPGA 收到第 3 个 `0xFF` 即解析缓冲区。

| 命令 | 参数 | 作用 | 等价实体控制 |
|---|---|---|---|
| `NEXT` | 无 | 下一张已载入图片 | `key1` |
| `AUTO` | 无 | 开 / 关自动轮播 | `key2` |
| `BRUP` | 无 | 亮度循环 +1（到顶回 0） | `key3` |
| `BRGT` | n = 0..4 | 直接设亮度档 | 增强 |
| `MODE` | n = 0..7 | 直接设转场模式（0 自动轮播 / 1-6 带状特效 / 7 淡入淡出） | `SW1`~`SW3`，增强为直接选 |
| `MARQ` | n = 0..1 | 字幕 1 = 显示 / 0 = 隐藏 | `SW4` |
| `IMGX` | n = 1..4 | 直接选第 n 张（仅在已载入范围内生效） | 增强 |

参数越界（如 `MODE 8`、`BRGT 5`、`IMGX 0`）或多位参数（如 `MODE 33`）会被长度 / 数字范围守卫静默丢弃，不会误触发。

#### 屏幕端按钮事件代码（淘晶驰 USART HMI）

每个按钮在 **Touch Release Event** 里写两行——`print` 发 ASCII 命令，`printh ff ff ff` 发终止符（下表 `⏎` 表示两行分开写）：

| 按钮 | 事件代码 |
|---|---|
| 下一张 | `print "NEXT"` ⏎ `printh ff ff ff` |
| 自动轮播 | `print "AUTO"` ⏎ `printh ff ff ff` |
| 亮度 +1 | `print "BRUP"` ⏎ `printh ff ff ff` |
| 亮度档 n（0..4，各一个按钮） | `print "BRGT 3"` ⏎ `printh ff ff ff` |
| 转场模式 n（0..7，各一个按钮） | `print "MODE 3"` ⏎ `printh ff ff ff` |
| 字幕显示 / 隐藏 | `print "MARQ 1"` / `print "MARQ 0"` ⏎ `printh ff ff ff` |
| 直接选图 n（1..4，各一个按钮） | `print "IMGX 2"` ⏎ `printh ff ff ff` |

屏幕工程波特率保持 9600 与 FPGA 对齐。

#### 兜底语义（last-writer-wins，两端互为退路）

- **脉冲类**（`NEXT` / `AUTO` / `BRUP`）：实体按键与屏幕命令 **OR 合并**，任一都能触发。亮度直接设值 `BRGT` 覆盖当前档，实体 `key3` 仍可继续循环。
- **电平类**（`MODE` / `MARQ`）：屏幕命令置覆盖使能 `ovr_en` 并锁存值；**实体拨码一旦变动**（clk 域同步后检测到 `sw` 电平变化）就清 `ovr_en`，物理路径立即重新接管。于是「动一下拨码 = 夺回控制权，发一条屏幕命令 = 屏幕夺回」，谁都不会被永久锁死。

#### 跨时钟域与覆盖 mux（核心正确性点）

- **亮度**：UART 与 `key3` / `brightness_level` 同在 clk 域，直接合并，无 CDC。
- **`NEXT` / `AUTO` / `IMGX`（→ sd_card_clk 域）**：命令脉冲用 **toggle-CDC** 过域——clk 域每来一条命令翻转一个 reg，sd_card_clk 域 2FF 同步 + `s1^s2` 边沿检测还原成单周期脉冲，再 OR 进 `sd_card_bmp` 现有的 key 条件。`IMGX` 的 2-bit 目标值用 **data + toggle** 同步（数据准静态、人类速率，toggle 边沿到达时数据已稳定 ≥2 拍）。所有 CDC 都在 top 里做，`sd_card_bmp` 保持单时钟。
- **`MODE` / `MARQ`（→ video_clk 域）**：保留已验证的物理路径**一位不动**——`sw_v0 <= sw[2:0]`、`trans_mode` 的物理项 `~sw_v1`、`sw4_v0 <= sw[3]`、`marquee` 的物理项 `sw4_v1` 全部原样。屏幕覆盖作为**并联 mux 叠加在其后**：clk 域维护 `mode_ovr_en/val`、`marq_ovr_en/val`，2FF 同步进 video_clk，

  ```verilog
  assign trans_mode = mode_ovr_en_v1 ? mode_ovr_val_v1 : ~sw_v1;
  assign marquee_en = marq_ovr_en_v1 ? marq_ovr_val_v1 : sw4_v1;
  ```

  **退路天然成立**：屏幕从未发命令时 `ovr_en = 0`，`trans_mode` / `marquee_en` 与改前逐位相同。

#### 退路开关与验证

- **一行退路**：不接屏幕 → 所有 `ovr_en = 0`、无命令脉冲注入，板上行为与改动前完全一致，无需改代码。
- **接线退路**：Type-C 枚举不通 → 改 `pin.adc` 两行 `LOCATION` 到 GPIO 排针 TTL 飞线，RTL 不动。
- **离线验证**：`tools/sim_uart_ctrl.py` 是这条链路的周期精确模型（三时钟域按真实相位偏移跑在同一时间轴上）。Pass A 真实分频 5208 逐字节解码，B 七条命令效果 + 越界守卫，C toggle-CDC 每条命令恰好一个 sd 脉冲且 `IMGX` 值正确，D 覆盖 mux + 拨码夺回 + 亮度合并，E 负对照（只给 2 个 `0xFF`、未知关键字、`NEXTX` 超长、半帧后接有效帧、**完全不发命令时输出与基线逐位一致**）。全部 34 项通过。

#### 竞赛规则说明

串口屏自带 MCU，但它是**人类输入外设**（等同遥控器 / 键盘）：只发命令，不参与任何媒体算法、控制逻辑或数据预处理。显示 / 转场 / 缩放 / 音频仍 100% 在 FPGA 内自主实现，符合赛题「算法、控制逻辑和数据处理流程均在 FPGA 内自主实现，未引入额外处理器参与控制或算法预处理」的要求。

#### 尚未做（Stage 2）

`uart_tx` 目前恒为空闲高（Stage 1 只做 RX 控制）。状态回传（把 `IMG:n` / 亮度 / 自动开关 / 模式实时发回屏幕文本控件）留到 Stage 1 上板验证通过后再接，用 `ENABLE_READBACK` 参数一键开关，关掉即退回 Stage 1 行为。

### 8. 亮度、音频可视化与视频链路顺序

- `video_brightness.v` 对 RGB 三通道做饱和加减，移位加法实现，不引入乘法器。
- `audio_visualizer.v` 采样 HDMI 音频链路的左右声道 PCM，在画面底部绘制滚动波形、网格背景、频谱柱、峰值线和高能量闪烁点；按符号翻转间隔估算音阶频率区间。
- 视频链路顺序：帧缓存读出 → 转场 → 亮度 → 音频可视化 → OSD → **滚动字幕** → RGB 转 AXI-Stream → HDMI 发射核。因此图片渐入时 OSD 状态与字幕始终清晰可读；字幕排在最末端，压暗的是它下面的成品画面。
- 三个叠加区在垂直方向互不重叠：OSD 面板 y ≤ 117、字幕横带 y 224..255（画面正中）、音频可视化 y ≥ 352。`tools/sim_marquee.py` 的 Pass C 对这条不重叠性和横带的 1+3+24+3+1 行构成都有断言。

### 9. 加载链路鲁棒性

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

Slow / Fast 两个 corner 全部收敛，Setup / Hold 违例端点均为 0，全局 Setup WNS +0.569 ns、Hold WNS +0.011 ns（串口屏 Stage 1 合入后的最近一次无头构建）。

| 时钟 | 约束 | 实测 fmax（域内） | SWNS（含跨域） |
|---|---|---|---|
| `sd_card_clk` | 100 MHz | 106.033 MHz | +0.569 ns |
| `ext_mem_clk` | 125 MHz | 157.134 MHz | +0.714 ns |
| `video_clk` | 25 MHz | 35.312 MHz | +11.681 ns |
| `clk` | 50 MHz | 72.558 MHz | +3.109 ns |
| `hdmi_5x_clk` | 125 MHz | 308.642 MHz | +4.760 ns |

各时钟 SWNS 已同时统计 intra- 与 inter-clock 路径（见 `final_timing.rpt` 注），全部为正、违例端点 0。资源占用：布局布线后 6436 slices（65.67%）、36 RAM、2 DSP；综合阶段 7185 LUT、6778 寄存器。串口屏 UART RX + 命令解析 + 三域 CDC 较上一版增加约 96 slices（6340 → 6436），**RAM / DSP 不变**——解析器是纯组合 LUT 逻辑，没有推断出 BRAM，与字幕字模表同理。

滚动字幕给 `video_clk` 域加了约 600 个 LUT，**BRAM 保持 36 不变**——字模表是纯组合 LUT 逻辑，没有被推断成 RAM；该域对 25 MHz 约束仍有 +11.681 ns 余量（fmax 35.312 MHz）。

历次构建间 `clk` / `ext_mem_clk` 等域的小幅漂移（如 `clk` 域 fmax 77.328 → 72.558 MHz、`ext_mem_clk` +0.872 → +0.714 ns）都落在字幕与串口屏不碰的路径上，是布局布线在不同构建间的抖动而非回归，各域违例端点始终为 0、余量为正。`clk` 域最紧路径仍是 `seg_scan` 位选多路选择器到 `seg_data` 输出寄存器（4 级逻辑、以线延迟为主）；串口屏命令解析是慢速组合匹配，没有成为该域新瓶颈（仍有 +3.109 ns 余量）。

余量最紧的是 `sd_card_clk`（+0.569 ns，fmax 106.033 MHz vs 100 MHz 约束，约 6.0%）：音频流式读取（`sd_audio_stream`）与图片级重试逻辑都在这个域，**横向带状转场引擎与滚动字幕都完全不碰它**。串口屏在该域只新增一条 CDC 同步链、key 条件的一个 OR 输入和受 `img_loaded_count` 门控的 `img_idx` 载入，构建后违例端点仍为 0、余量仍为正，与计划预期一致。

带状转场引擎落在 `ext_mem_clk`（SDRAM 读）域，是带状转场那次唯一为时序改过结构的地方。首版把 `select_top` 的组合深度暴露在 `g → next_sel_r` 路径上：前导 `g+1` 增量器叠加 split 分支 `|gi-half|` 的串行绝对值链（减法器 → 选择器 → 比较器）凑成 9 级逻辑，1 个违例端点、SWNS −0.184 ns。两处**等价**改写把这条路径拆短：① 把 `g+1` 寄存成 `g_plus1`，`next_sel_comb` 读触发器而非加法器（滞后 1 拍无影响，`next_sel_r` 只在约 1280 拍后的组边界被消费）；② 把 split 的 `|gi-half| < K` 改写成两个**并行**比较 `(gi > half-K) && (gi < half+K)`，界由 `prog` 推出，`gi` 不再串过减法器。修复后 `ext_mem_clk` 域 SWNS 回到正余量（当时 +1.307 ns，本次构建 +0.714 ns），违例端点归零。改动的正确性在 `tools/sim_transition.py` 的 Pass G 里各有断言兜底：G0 穷举证明 split 两式恒等，G2 在带 `g_plus1` 滞后的 FSM 下逐字校验六种特效的缓冲归属与组边界重定向不变。**后续若再改 `select_top`，不要把 `gi` 重新接回减法器/绝对值串行链，否则这条 8 ns 路径会再次违例。**

`timing.sdc` 中对 SDRAM 硬核 DQ 边界写了 `set_max_delay -datapath_only` 例外：这些路径全在加密 IP 内部、fabric 与 PHY 之间没有用户逻辑，安路也未随该 IP 附带 `.tcl` 约束，不做例外时它们贡献总 TNS 的 87% 伪违例。

## 验证工具

本机没有 iverilog / verilator，因此控制流逻辑用 `tools/` 下的周期精确 Python 模型验证，再上板确认。

| 脚本 | 用途 |
|---|---|
| `sim_load_retry.py` | 图片加载调度器 + 最小 `bmp_read`。含负对照：同一次瞬时失败跑改动前的调度器，复现"卡里四张、只显示三张"的原始现象 |
| `sim_scaler_nn.py` | 缩放器时序与弹性缓冲峰值占用 |
| `sim_dir_scan.py` | 按字节重放根目录扫描，直读 TF 卡，证明扫描器确实记录到全部 BMP |
| `sim_sd_retry.py` / `sim_transition.py` | 扇区级重试、转场状态机与带状特效引擎（含 3 位 `I_mode` 模式选择、`select_top` 逐组缓冲归属、组边界重定向、Pass G 六特效扫描几何与 ASCII 条带，以及模式交换 / 去掉饱和兜底两个负对照） |
| `sim_uart_ctrl.py` | 串口屏控制链路（第 7 节）的三时钟域周期精确模型，clk/sd_card_clk/video_clk 按真实相位偏移跑在同一时间轴上。Pass A 真实分频 5208 逐字节解码、B 七条命令效果 + 越界守卫、C toggle-CDC 每条命令恰好一个 sd 脉冲且 `IMGX` 值正确、D 覆盖 mux + 拨码夺回 + 亮度合并、E 五个负对照（含「零命令时输出与基线逐位一致」的退路证明），共 34 项 |
| `gen_marquee_font.py` | PIL 渲染标语中文字模 → `marquee_font.vh` 与预览图。自带墨迹自检（非空、不越 24x24、居中、`赛` 两处逐位一致、无字符冲突）与负对照（故意偏移 1 px 居中、故意留空一个字模、`PITCH=30` 都必须被抓到） |
| `sim_marquee.py` | 字幕周期精确模型。Pass A 逐周期镜像 raster tracker 两条分支、B 穷举 640x1120 = 716800 组证明借位回绕寻址恒等于无界判据且 `cell_idx`/`col` 就是节距的商与余数、C 横带几何与邻区不重叠、D 整帧渲染逐像素对比生成器的黄金模型并校验滚动节奏与 SW4 屏蔽、E 五个负对照 |
| `check_marquee_transcription.py` | RTL ↔ 生成器 ↔ top ↔ `.al` 一致性：`.vh` 与重新渲染逐字节相同，每个 localparam / 位切片 / 寄存器位宽 / 色值都由生成器常量**重算**而非二次手写，链路接线与 SW4 极性，`.al` 注册与 `` `include `` 位置；外加一条 Verilog-2001 保留字静态检查——`wire [4:0] cell;` 曾让整轮综合直接 HDL-8007 语法失败，而当时 95 项行为检查与 72 项一致性检查全绿。九个内存内变异必须全部被前面某项抓到 |
| `cmp_bit.py` | 比较两个比特流的配置体。ASCII 头带分钟级 `# Date:`，所以未改动设计的重编也不是逐字节相同，整文件哈希比较必然误报 |
| `td_build.ps1` / `td_flow_exit.tcl` | 无头构建 |
| `gen_test_bmp.py` / `check_sd_card.py` / `probe_retry_trace.py` / `render_defect_preview.py` | 测试图生成与卡上诊断 |

模型的可信度取决于是否忠实镜像 RTL 语义，两个踩过的坑：右值必须只读周期开始前的快照（否则模型能做硬件做不到的事，比如在同一周期既判失败又发起新加载）；默认值要按 RTL 的位置写（RTL 在 `else` 分支开头默认清零、由后面的赋值覆盖，模型若在别处默认就把要验证的语义当成前提了）。

`doc/convert/` 下有两组测试图与对应的期望上屏效果：`setA_fill/` 覆盖 320x240、640x480、800x600、1920x1080；`setB_border/` 覆盖 64x64、100x100、159x119、1920x64 这类极端宽高比，用于验收缩放与黑边居中。

**「自适应缩放」演示卡**（向评委展示任意分辨率图片时用）用四张**内容互不相同**的图：青苔森林（`doc/convert/3.png`）、玫瑰心形（`doc/convert/output_bmp/4.bmp`）、西瓜（`doc/TF卡图片/西瓜_640x480_24bit_显示正常_工程适配版.bmp`）、雪山湖面倒影（`doc/convert/1.png`）。森林与西瓜源本身就是 4:3 的 640x480，另两张是 1536x1024（3:2）风景照——因为缩放器按横纵两轴独立计算、非等比，直接缩 3:2 源会被压扁，所以这两张先**中心裁剪**成精确 4:3（不拉伸）。每张源都不小于其目标尺寸，生成阶段只缩不放，放大留给 FPGA 做。随后统一缩到不同的 4:3 分辨率，并在画面**右上角叠加一个小号半透明标注**写明该图自己的源分辨率——上屏读到的数字就是缩放器收到的源尺寸。

四张按 box 由小到大排序写卡：玫瑰 320x240（→640x480，×2 铺满）、西瓜 480x360（→640x480，非整数倍缩小）、森林 640x480（1:1 原生）、雪山 800x600（→640x480，从大于面板缩回）。上屏 box 单调增长、源分辨率跨 320→800，覆盖「放大铺满」「非整数缩小」「原生」「整数缩小」四条路径，且因源与面板同为 4:3 而全部不变形。`doc/convert/demo_labeled_preview.png` 是「左=带标注源图、右=模拟 640x480 上屏」的对照预览。`1920x1080`、`1920x64` 因非 4:3 会被压扁，**刻意排除**在演示卡外。

> **已知硬件缺陷（2026-09-05，未修）**：带黑边的几何（`off_x/off_y != 0`，即源高 <120 触发 4 倍上限放大留边的那条路径）在真机上**加载必然失败**——128x96 的西瓜、苹果两张不同内容、放在两个不同目录位置都复现，读数为 `3C4485`（第 4 张载入放弃、stall 看门狗与 header 拒绝都触发过）；而所有 `off_y == 0` 的几何（320x240/480x360/640x480/800x600）都稳定加载。离线 `tools/sim_scaler_nn.py` 对 128x96 全过，说明缺陷在 sim 未覆盖的耦合处，尚未定位。演示卡因此**不再使用 128x96 黑边槽**；`doc/convert/setB_border/` 里的极端宽高比图只用于离线/仿真验收，**不要**写进给评委的演示卡。

复现：`python doc/convert/gen_labeled_demo.py` 生成 `doc/convert/demo_labeled_stage/`（改图集后务必确认该目录里没有上一版残留文件，`sync_to_sd.py` 是按文件名字典序取前 4 个）；把卡上 `MUSIC.WAV` 先备份到卡外，再 `sync_to_sd.py F: -s doc/convert/demo_labeled_stage -n 4 --wav <备份WAV>`；最后三项验证：`tools/check_sd_card.py F:`（4 张 BMP 头合法且物理连续）、`tools/sim_dir_scan.py F:`（扫描器看到全部 4 张 + WAV 目录项排在 BMP 之前）、`tools/check_wav_on_card.py F:`（按 RTL 逐字节复现流读器，校验 RIFF/WAVE 魔数、每扇区边界的 4 字节对齐、写 FIFO 帧数与 PCM 非静音）。

## 最简复现步骤

1. 用 FAT32 格式化 TF 卡。
2. 把 `doc/TF卡图片` 中的示例 BMP 复制到卡根目录，或用 `doc/convert` 的脚本生成并同步自己的图片。
3. 插入 TF 卡，HDMI 线接到开发板 HDMI_B。
4. 用 Anlogic TD 打开 `src/td_project/HDMI1.4b_Transmitter_v1.0.al`，综合、布局布线并下载；仓库里 `best_result/` 的比特流是最近一次 GUI 构建的产物，未改 RTL 时可以直接烧。若用无头脚本重新构建过，按「构建与烧录」一节的路径规则选择比特流。
5. 首图加载完成后显示器应出现图片，数码管状态码停止变化；约 7 秒后四张图全部载入。
6. `key1` 手动切换，`key2` 开关自动轮播，`key3` 调节亮度。默认（`SW1`/`SW2`/`SW3` 都 OFF）切换时应依次轮播淡入淡出、擦除↓、擦除↑、百叶窗、中心展开、随机条、梳状；按第 6 节的表拨对应拨码可固定为某一种特效（下一张图生效）。画面正中（`SW4` 保持 OFF）应有一条自右向左滚动的十周年标语字幕，约 37.6 s 一圈。
7. 字幕的四项上板验收：① 横带在画面正中、文字水平滚动且不撕裂；② 左上 OSD 面板与底部频谱完全没被影响；③ `SW4` 拨 ON 字幕立即消失、拨回 OFF 立即恢复；④ `SW1`~`SW3` 的转场特效行为与改前逐项一致——这是独立的 `sw4_v0`/`sw4_v1` 同步器没有污染 `trans_mode` 的真机证据。

## 后续实现方向

### 1. 缩放质量

- 当前是最近邻。升级为双线性插值：行缓冲改为两个源行缓冲，Bresenham 累加器改为插值权重，端口列表可以完全不变。
- 比较不同缩放策略的资源占用与显示质量。

### 2. 转场效果扩展

- 已有淡入淡出 + 一族横向带状特效（擦除↓/↑、百叶窗、中心展开、随机条、梳状），全部沿垂直方向扫动、共用同一套组边界重定向机制。新增一种带状特效只需在 `select_top` 里加一条几何分支。
- 尚未做的是**列 / 逐像素**类特效（推入、水平滑动、棋盘格、溶解、圆形、轮盘、真·交叉淡化）：它们要在同一帧按列或按像素同时读两个缓冲并混合，必须重构最紧的 `ext_mem_clk` 读路径与承重的突发对齐不变量，风险显著更高，是下一步要权衡的方向。
- 多缓冲已就位，带状特效期间读两个缓冲区是安全的，这一前提可被后续效果复用。

### 3. 图层叠加与字幕增强

- ~~作品标语~~、~~中文点阵字库~~ 已随 `marquee_overlay.v` 落地：24x24 黑体点阵、15 个字模、半透明横带跑马灯。**尚未做**：时间戳、参数提示、简单图标。
- 换标语要改两处并保持一致：`gen_marquee_font.py` 的 `SLOGAN`，和 `marquee_overlay.v` 里硬写的 `localparam N_CELLS`（`TEXT_W` / `TRAVEL` 由它推出，会自动跟着走）。两边不一致时 `check_marquee_transcription.py` 直接报错。超过 16 个字模还要放宽 `marquee_glyph` 的 `cell_idx[3:0]` 实参位宽，同一条检查也会拦住。更多字号或图标可复用同一条 PIL → `.vh` → `` `include `` 管线。

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

- `top_tf_hdmi_audio.v`：系统顶层，连接 TF 卡读取、SDRAM、视频时序、HDMI 发射和音频链路；并承接串口屏命令的跨时钟域注入与「实体按键 / 拨码 ↔ 屏幕覆盖」的 last-writer-wins 合并。
- `uart_screen_ctrl.v`：clk 域串口屏桥（第 7 节）。UART 8N1 RX（2FF 同步 + 中点采样）+ 淘晶驰 TJC 命令解析（4 字符关键字 + 可选 ` 数字`，连续 3 个 `0xFF` 结尾），产出 `NEXT`/`AUTO`/`BRUP` 单周期脉冲与 `BRGT`/`MODE`/`MARQ`/`IMGX` 的「值 + set 选通」。不含合并策略；Stage 1 `uart_tx` 恒为空闲高。
- `SD/sd_card_bmp.v`：BMP 扫描调度、图片加载与重试、切换控制。仍是单时钟（sd_card_clk）模块；屏幕命令脉冲在 top 里 CDC 之后经 `cmd_next_pulse`/`cmd_auto_pulse` OR 进现有 key 条件，`cmd_img_sel`(+`_pulse`) 受 `img_loaded_count` 门控直接选图。
- `SD/bmp_read.v`：FAT32 解析、根目录扫描、BMP 头校验与像素流输出。
- `SD/scaler_nn.v`：最近邻缩放，把任意受支持分辨率映射到 640 x 480 画布并居中、填黑边；横纵两轴各自独立取 `min(源 × 4, 640/480)`，**非严格等比**——4:3 源或两轴都未达 4 倍上限的小图不变形，其余宽高比会被拉伸/压扁。
- `SD/sd_card_sec_read_write.v`：扇区级 SPI 读，含起始令牌重试。
- `SD/frame_fifo_write.v` / `SD/frame_fifo_read.v` / `SD/frame_read_write.v`：SDRAM 帧缓存读写。`frame_fifo_read` 内含带状特效引擎——`select_top(g, progress, effect)` 逐组选缓冲、组边界重定向地址、`effect` 经 2-flop 同步进 ext_mem_clk 域、`next_sel_r` 寄存使 `select_top` 不进入地址关键路径；`frame_read_write` 转发 `read_effect` 特效码。
- `SD/video_timing_data.v`：640 x 480 视频时序生成。
- `video_transition.v`：转场控制器，决定面板何时、以哪种效果看到缓冲区切换；输出两个缓冲选择器、淡入淡出电平与 3 位带状特效码 `O_effect`。3 位 `I_mode`（拨码）：`000` 自动轮播淡入淡出与六种带状特效，`001`~`110` 固定一种，`111` 固定淡入淡出。
- `video_brightness.v`：RGB 三通道亮度档位调节。
- `video_fade.v`：按转场给出的电平做比例缩放。
- `audio_visualizer.v`：底部波形、频谱柱和峰值显示叠加。
- `osd_overlay.v`：展示型状态面板叠加，内置 8x8 字模。
- `marquee_overlay.v`：画面正中滚动字幕叠加，内置 24x24 中文点阵字模——`marquee_font.vh` 由 `tools/gen_marquee_font.py` 生成，经 `` `include `` 引入模块体，**不入 `.al`**。`I_en` 由 `SW4` 驱动，OFF = 显示。
- `video_rgb_to_axis_640x480.v`：RGB/DE 转 AXI-Stream。
- `SD/sd_audio_stream.v`：sd_card_clk 域音乐流读器，扫描定位 `MUSIC.WAV` 后跳头、组帧 {R,L}、背压、放完回卷循环，写异步 FIFO。
- `audio_pcm_player.v`：video_clk 域 48 kHz 节拍器，小数分频取 FIFO 前瞻数据、16→24-bit 左对齐输出，欠载时持续打 valid 填 0。
- `audio_arc_calculate.v`：每 48 个 `audio_valid` 生成一次 ACR（CTS）参数，要求 valid 为稳定 48 kHz 脉冲流。
- `hdmi_audio_tone_i2s_64fs.v` / `I2S_receiver.v`：原 I2S 测试音与解串，已不再例化，保留在仓库便于调试回挂。

## 备注

本项目遵循赛题要求：算法、控制逻辑和数据处理流程均在 FPGA 内自主实现，未引入额外处理器参与控制或算法预处理。`doc/convert` 与 `tools/` 下的 Python 脚本只用于离线制作测试素材、驱动构建和验证逻辑，不参与板上的实时数据通路。第 7 节的串口屏自带 MCU，但它是**人类输入外设**（等同遥控器 / 键盘）：只经 UART 发命令，不参与任何媒体算法、控制逻辑或数据预处理，显示 / 转场 / 缩放 / 音频仍 100% 在 FPGA 内自主实现。
