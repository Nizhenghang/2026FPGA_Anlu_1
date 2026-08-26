


// ============================================================================
// 文件：include/global_def.v
// 功能：SDRAM 控制器全局宏定义（被 SDRAM 相关模块 `include 引用）
// 说明：这些是安路官方 SDRAM 控制器 IP 配套的参数定义，不要随意改动。
//       位宽/时序必须与 IP 核配置一致，否则读写会出错。
// ============================================================================

// ---- 数据/地址/掩码位宽（对应 SDRAM 控制器 App 接口）----
`define   DATA_WIDTH                        32  // App 总线数据位宽：32bit(一次突发传 4 字节)
`define   ADDR_WIDTH                        21  // App 地址位宽：21bit -> 2M 地址空间
`define   DM_WIDTH                          4   // 数据掩码位宽：4bit(每 byte 1bit)

// ---- SDRAM 行列/ Bank 地址位宽（由具体 SDRAM 型号决定）----
`define   ROW_WIDTH                        11  // 行地址 11bit
`define   BA_WIDTH                        2   // Bank 地址 2bit(共 4 个 Bank)

// ---- 时序参数（单位：ns，基于控制器 125MHz 时钟）----
`define	  SDR_CLK_PERIOD				1000000000/125000000  // SDRAM 时钟周期 = 8ns(125MHz)
`define   SELF_REFRESH_INTERVAL			64000000/`SDR_CLK_PERIOD/2**(`ROW_WIDTH) // 自刷新间隔计数

