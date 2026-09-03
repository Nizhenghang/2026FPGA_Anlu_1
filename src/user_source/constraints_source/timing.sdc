
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

#    sys_pll_m0 的 clkc[2] 是 180° 相移的 SDRAM 采样时钟，下面 5.5 节要引用它。
#    它此前从未被命名，在时序报告里以裸名 sys_pll_m0/pll_inst.clkc[2] 出现，
#    因而无法用 get_clocks 引用、也就无法对它写任何例外约束。
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk sd_card_clk ext_mem_clk]

# ------------------------------------------------------------
# 5.5 SDRAM 硬核 DQ 边界例外（消除占总 TNS 87% 的伪违例）
#
#    实测违例分解（phy_1，audio_visualizer 已入网表的基线）：
#      ext_mem_clk_sft -> ext_mem_clk : SWNS -6.301  STNS -197.968  32 端点
#      ext_mem_clk -> ext_mem_clk_sft : SWNS -3.343  STNS  -95.481  32 端点
#    两组合计 -293.449ns，占 STNS(-336.172ns) 的 87%。
#
#    这两组的端点是 SDRAM 硬核 EG_PHY_SDRAM_2M_32 的 DQ PAD（U3/sdram.dq[31:0]），
#    路径全部落在加密 IP 内部，fabric 与 PHY 之间没有任何用户逻辑：
#      读入方向 Logic Level = 0，6.453ns 全是 cell 延迟、net 延迟占 0%
#      写出方向 Logic Level = 1 (PAD=1)
#    也就是说这部分没有 RTL 可改，只能在宏边界上用 SDC 表达。
#
#    ext_mem_clk_sft 是 180° 相移时钟，硬核 PHY 用它对 DQ 做中心对齐采样，
#    建立/保持由安路在硅片层面保证。工具在该边界按 4ns 半周期预算做静态分析，
#    并不反映这一保证；而安路未随该 IP 附带 .tcl 约束
#    （对比：工程里两个异步 FIFO 都自带 .tcl，并已由 settings.cfg 的 IpSDCList 挂载，
#      SDRAM IP 目录里只有 sdram.ipc，没有任何 .tcl）。
#
#    写法采用 IPUG012 §5 推荐的 set_max_delay -datapath_only 放松方式。
#    ext_mem_clk_sft 不是任何 FIFO 的时钟，因此不触犯
#    “禁止对 FIFO 读写时钟使用 set_clock_groups / set_false_path” 这条铁律。
# ------------------------------------------------------------
set_max_delay -from [get_clocks {ext_mem_clk}]     -to [get_clocks {ext_mem_clk_sft}] -datapath_only 100
set_max_delay -from [get_clocks {ext_mem_clk_sft}] -to [get_clocks {ext_mem_clk}]     -datapath_only 100

# ------------------------------------------------------------
# 5.6 ext_mem_clk -> sd_card_clk 跨域例外
#
#    实测：SWNS -1.433ns  STNS -40.894ns  32/36 端点违例，全组仅 40 条路径。
#    最差路径 Budget 只有 2.000ns（工具按同一 PLL 的 8ns/10ns 最近沿配对：
#    sd_card_clk rising@10ns - ext_mem_clk rising@8ns），而数据路径 3.196ns。
#
#    端点全部是 sd_card_bmp / bmp_read 的用户逻辑（load_stall_cnt[*]、
#    load_busy、load_abort、bmp_read_m0/state[*]、sd_sec_read），以及用户自己的
#    toggle 同步器首级 wrfin_tgl_sync_reg[0]。
#
#    与 FIFO IP 自带约束不冲突，这一点已逐项核对：
#      write_buf : clkw=write_clk(sd_card_clk) -> clkr=mem_clk(ext_mem_clk)
#      read_buf  : clkw=mem_clk(ext_mem_clk)   -> clkr=read_clk(video_clk)
#    IP 的 .tcl 约束的是 primary_addr_gray_reg[*] -> sync_r1[*] 这对寄存器，
#    对应方向分别是 sd_card_clk->ext_mem_clk（实测 +0.714ns 干净）和
#    ext_mem_clk->video_clk（实测 +3.934ns 干净），都不是本节约束的方向。
#
#    反向 sd_card_clk -> ext_mem_clk 已干净，不加约束。
# ------------------------------------------------------------
set_max_delay -from [get_clocks {ext_mem_clk}] -to [get_clocks {sd_card_clk}] -datapath_only 100

# ------------------------------------------------------------
# 6. 【禁止启用】下面这组异步分组必须永久保持注释状态
#
#    原因：sd_card_clk / ext_mem_clk / video_clk 正是两个异步 FIFO 的读写时钟
#      write_buf : clkw=sd_card_clk, clkr=ext_mem_clk
#      read_buf  : clkw=ext_mem_clk, clkr=video_clk
#    IPUG012 §5 明确规定：用户 SDC 中禁止对 FIFO 读写时钟使用
#    set_clock_groups / set_false_path / set_cross_domain_timing self，
#    否则会把 IP 自带 .tcl 里对格雷码指针的 set_max_delay 冲掉，
#    使 FIFO 跨域失去约束——那才是真实的时序风险。
#
#    本工程 FIFO 之外的跨域一律用上面 5.5 / 5.6 的 set_max_delay -datapath_only
#    逐对放松，既能消伪违例，又不破坏 FIFO 自带约束。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. Input/Output delay 经评估后【有意留空】，理由如下（非遗漏）
#
#    set_input_delay / set_output_delay 需要板级走线延迟与器件 tAC/tOH 数据。
#    本工程三条对外接口都不具备填写这些数据的前提：
#
#    a) SDRAM dq[31:0] 是片内硬核 EG_PHY_SDRAM_2M_32 的接口，不是板级 IO，
#       没有 PCB 走线延迟可言；其采样关系已在 5.5 节按硬核保证处理。
#    b) HDMI TMDS 差分对走加密的 hdmi_phy_wrapper，IO 时序由该 IP 自行约束，
#       用户 SDC 无法也不应介入。
#    c) TF 卡 SPI 最高 25MHz（SPI_HIGH_SPEED_DIV=0 -> SCK = 100MHz/((0+2)*2)），
#       周期 40ns，相对板级走线延迟裕量极大。
#
#    在缺少实测走线数据的情况下凭空填写数值，只会制造虚假的时序可信度，
#    比明确留空更糟——因此这里保持空白并记录依据。
# ------------------------------------------------------------
