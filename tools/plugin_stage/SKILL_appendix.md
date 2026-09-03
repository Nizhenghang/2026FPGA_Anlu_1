
---

## 本项目适配说明（2026Anlu1 · 基于 FPGA 的 HDMI 多媒体播放系统）

本节由 2026 安路赛道赛题一的实施过程沉淀，仅在处理 `D:/Nizhenghang/Project/2026Anlu1` 时适用。

### 证据优先级（重要）

**官方 `lab_ex4_tf` 例程不是权威。** `references/lab_ex4_tf_constraints.md` 记录的是官方例程的取值，而本工程是用户在该例程基础上**自行修改且已上板跑通**的版本。两者冲突时，一律以**本工程的实测结果**为准，例程文档只作旁证。同理，「官方例程就是这么做的」不能单独作为改动本工程的理由。

### 已实测确认的工具链事实

以下均为在该机器上跑通验证过的结论，不是文档推演：

1. **TD 可无人值守驱动**。安装于 `D:/Download/TD`（V6.2.178840），批处理入口是 `bin/td_commands_prompt.exe <flow.tcl>`，而非打开 GUI。工程内 `tools/td_build.ps1 -Stage syn|phy|all` 可一键跑完综合与布局布线并打印 WNS/TNS。
2. **`DefaultFlow.tcl` 结尾没有 `exit`**。跑完后进程退回交互式 prompt 永久阻塞在 stdin，症状是构建**静默地只完成一半、且日志里没有任何错误**。必须用 wrapper tcl 以 `catch {source ...}` 包住再显式 `exit`（`catch` 是必需的，因为 flow 用 `return -code error` 报告步骤失败），调用侧再加带超时的看门狗兜底。原厂 `run.bat` 末尾的 `pause` 其实也永远到不了。
3. **改 `.sdc` 必须重跑综合**。phy 阶段的命令序列只有 `import_device / open_project -noanalyze / import_db / place / route / bitgen`，**没有 `read_sdc`**；时序约束是从 `../syn_1/*_gate.db` 里 "Import timing constraints" 带进来的。只跑 phy 会让 SDC 改动完全不生效，且时序数字与改前逐位相同，极易误判为「约束写了但无效」。
4. **无人值守流程读的是 run 目录下的 `.prj`（`.al` 的快照），不是 GUI 工程 `.al`**。在 GUI 里新增源文件后若未重新生成 `.prj`，新文件不参与综合并报 `HDL-8007 black box`；其悬空输出还会让综合把无关逻辑合并，制造出**物理上不可能的时序路径**（曾出现 `osd_overlay → audio_arc_calculate → I2S_receiver` 这种穿加密核的假路径）。工程内 `tools/sync_prj_from_al.ps1` 负责比对与同步。
5. **`EG_PHY_SDRAM_2M_32` 硬核没有随附 `.tcl` 约束**（两个异步 FIFO 都有，且已被 `settings.cfg` 的 `IpSDCList` 挂载）。因此 fabric↔PHY 的 DQ 边界是约束真空，工具会按 4ns 半周期预算去卡硬核内部的 DQS 对齐路径，凭空产生占 TNS 九成的伪违例。解法是在用户 SDC 里先给相移时钟 `rename_clock` 命名，再用 `set_max_delay ... -datapath_only` 放松该时钟对。
6. **禁止对 FIFO 的读写时钟用 `set_clock_groups` / `set_false_path`**（IPUG012 §5 铁律），否则会冲掉 IP 内部格雷码指针自带的 `set_max_delay`。本工程 `timing.sdc` 里那段被注释掉的 `set_clock_groups -asynchronous` **必须保持注释状态**。
7. **大量伪违例会破坏布局布线器收敛真实路径的能力**。把上述硬核伪违例用例外约束消除后，原本在零点附近摆动的真实边际路径常被 P&R 自行修好，无需改 RTL。
8. **校验约束是否被工具接受**看 `*_exception.timing` 的 MCP/SMD Summary：`Total / Dominated / Shadowed / Ignored` 四列可确认 IP 自带的寄存器级约束没有被用户写的时钟级约束冲掉。

### 本工程 RTL 的两条通用教训

- **准静态派生值不要每拍重算**。FAT32 几何量（`bpb_*`、`file_sector_count`、簇→扇区偏移）在解析出引导扇区后全程不变，把它们寄存一拍即可整条移出关键路径，比通用流水更彻底，且功能严格等价。
- **寄存前必须核对消费点的状态机时序**。同一个 `bpb_root_dir_calc` 里，簇偏移量早于消费点约 460 拍就绪（可安全寄存），而 `data_start_sector` 是在消费点的**前一拍**才载入（整体寄存会采到旧值，是真实功能 bug）。同一表达式内不同操作数的可寄存性不同，必须逐项判定。
