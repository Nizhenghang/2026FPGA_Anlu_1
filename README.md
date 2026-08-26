# 2026FPGA_Anlu_1

基于安路科技 **EG4S20BG256**（康芯 HX4S20C 开发板）的 **HDMI 1.4b 多媒体播放系统**——2026 全国大学生嵌入式芯片与系统设计竞赛·安路赛道参赛项目（基础版本）。

本设计在安路官方设计参考例程（例程 5）基础上改编，实现一条完整的音视频播放链路：
**TF 卡读取 BMP → SDRAM 双缓冲帧缓存 → HDMI 1.4b 视频显示（640×480@60, RGB）+ HDMI 音频输出（48 kHz）**。

---

## 一、已实现功能（对照比赛基础要求）

| 基础要求 | 状态 | 说明 |
|---|---|---|
| 核心功能完整性 | ✅ | TF(SPI/FAT32) 读取 BMP（最多 4 张）、SDRAM 双缓冲、HDMI 1.4b 视频、按键交互（key1 切换 / key2 自动轮播） |
| 显示稳定性与鲁棒性 | ✅(设计) | 双缓冲无缝切换（切图不黑屏）、上电 POR（PLL 全锁后启动）、SD 卡 1 s 超时防死锁 |
| 基础音频输出 | ✅ | I2S 八音阶测试音（48 kHz）→ HDMI 音频数据岛包（ACR N=6144） |
| 系统工程规范性 | ✅ | 模块化分层、引脚(pin.adc)与时序(timing.sdc)约束完整、复用官方 HDMI 发射 IP 核 |

> 注：建立时序在 125 MHz 域（SDRAM 接口 / HDMI 串行）存在负 slack（SWNS≈−6.7 ns），TD 默认不阻塞 bit 流生成；实板表现需上板实测，必要时开启 `timing.sdc` 跨时钟域异步分组收时序。

---

## 二、本仓库已自包含（含安路第三方 IP，重要）

为便于团队协作与**直接综合**，本仓库已一并纳入安路科技（Anlogic）专有 IP 与官方资料。
这些组件的版权仍归安路科技所有，仅随本仓库分发供**团队内部学习与 2026 安路赛道竞赛开发**使用；
对外公开分发前请自行确认安路授权条款与赛事规则（详见 `NOTICE`）。

- **加密网表核（`*.enc.v`）**
  - HDMI 1.4b 发射核 APUG092：`src/user_source/hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v`
  - SDRAM 控制器：`src/user_source/hdl_source/include/sdr_as_ram.enc.v`、`sdr_init_ref.enc.v`、`sdr_wrrd.enc.v`
- **TD 生成 IP（`*.vhd`）**：`src/td_project/al_ip/`、`src/user_source/hdl_source/IP/`、`src/user_source/ip_source/` 下的 PLL / SDRAM / AFIFO / 音频 ROM 等
- **安路官方参考文档**：`doc/` 目录、`设计参考例程文档.md` / `.docx`

> 说明：仓库**不包含**综合产物（`*_Runs/`、生成 bit 流 `*.bit`、构建日志 `*.log/.logw`），
> 同学 clone 后需用 TD 在本机重新综合。本地记忆 `.workbuddy/` 也不入库（含个人隐私）。

---

## 三、目录结构（纳入仓库的部分）

```
.
├── .gitignore
├── LICENSE                 # 用户自写代码以 MIT 发布
├── NOTICE                  # 第三方 IP 归属声明
├── README.md
├── 代码说明.md             # 工程说明（自写）
└── src/
    ├── td_project/
    │   └── HDMI1.4b_Transmitter_v1.0.al   # TD 工程文件
    └── user_source/
        ├── constraints_source/
        │   ├── pin.adc        # 引脚约束
        │   └── timing.sdc     # 时序约束
        ├── hdl_source/
        │   ├── top_tf_hdmi_audio.v         # 顶层
        │   ├── SD/                         # TF 卡 / BMP / 双缓冲 / 数码管
        │   ├── IP/                         # 用户封装的 SDRAM/AFIFO/PLL _wrapper(.v)
        │   ├── include/global_def.v
        │   ├── rom/
        │   ├── test/
        │   └── *.v                         # 音频/视频/PHY 等自写模块
        └── ip_source/pll.v
```

---

## 四、构建与运行

1. 安装 **Anlogic TD**（建议与例程同源版本）。本仓库已含 EG4S20 IP 包所需的 `*.enc.v` 加密核与 `*.vhd` 生成 IP，**无需另行获取**。
2. 用 TD 打开 `src/td_project/HDMI1.4b_Transmitter_v1.0.al`。
3. 综合 → 实现 → 生成 bit 流，下载至 HX4S20C。
4. HDMI 线接开发板 **HDMI_B** 接口连接显示器；TF 卡放入符合要求的 24-bit 非压缩 BMP（640×480，可直接用 `doc/TF卡图片` 中的测试图，或用 `doc/convert/` 脚本转换自己的图片）。

> 注：综合产物与日志未纳入版本管理，每次在本机重新综合即可。

---

## 五、许可证

- 本仓库中的**用户自写代码**（全部 `.v`、约束文件、说明文档）以 **MIT 许可证**发布，见 `LICENSE`。
- 安路科技第三方 IP 与官方文档版权归安路科技所有，须遵守其相应授权条款，不在本仓库许可范围内。

---

© 2026 Nizhenghang
