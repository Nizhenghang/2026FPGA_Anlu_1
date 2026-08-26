

// ============================================================================
// 文件：hdmi_phy_warpper.v  (注：文件名/例化名沿用安路官方 "warpper" 拼写)
// 功能：HDMI TMDS 物理层封装 —— 把 4 路 10bit TMDS 并行数据串行化后输出 LVDS 差分
// 接口：输入 4 路 I_tmds_channel_{0,1,2,clk}(各 10bit，来自 HDMI 发射核)；
//       输出 O_tmds_ch{0,1,2}_p + O_tmds_clk_p(板载 HDMI_B 的 TMDS 差分对)
// 结构：例化 4 个 lane_lvds_10_1(3 数据通道 + 1 像素时钟通道)
// 时序：I_pixel_clk=25MHz(像素), I_serial_clk=125MHz(=5x，DDR 串行)
// 说明：前面 S_tmds_data_ch* 的逐位重组是等价占位(输入=输出)，可忽略
// ============================================================================
module hdmi_phy_wrapper#(
    parameter DEVICE = "EG"
    )(
    input wire      I_pixel_clk,
    input wire      I_serial_clk,
    input wire      I_rst,

    input wire[9:0] I_tmds_channel_0,
    input wire[9:0] I_tmds_channel_1,
    input wire[9:0] I_tmds_channel_2,
    input wire[9:0] I_tmds_channel_clk,
    
    output wire     O_tmds_ch0_p,
    output wire     O_tmds_ch1_p,
    output wire     O_tmds_ch2_p,
    output wire     O_tmds_clk_p
);
    
    wire[9:0] S_tmds_data_ch0;
    wire[9:0] S_tmds_data_ch1;
    wire[9:0] S_tmds_data_ch2;
    wire[9:0] S_tmds_data_clk;

    assign S_tmds_data_ch0 = {I_tmds_channel_0[0],
                              I_tmds_channel_0[1],
                              I_tmds_channel_0[2],
                              I_tmds_channel_0[3],
                              I_tmds_channel_0[4],
                              I_tmds_channel_0[5],
                              I_tmds_channel_0[6],
                              I_tmds_channel_0[7],
                              I_tmds_channel_0[8],
                              I_tmds_channel_0[9]};

    assign S_tmds_data_ch1 = {I_tmds_channel_1[0],
                              I_tmds_channel_1[1],
                              I_tmds_channel_1[2],
                              I_tmds_channel_1[3],
                              I_tmds_channel_1[4],
                              I_tmds_channel_1[5],
                              I_tmds_channel_1[6],
                              I_tmds_channel_1[7],
                              I_tmds_channel_1[8],
                              I_tmds_channel_1[9]};

    assign S_tmds_data_ch2 = {I_tmds_channel_2[0],
                              I_tmds_channel_2[1],
                              I_tmds_channel_2[2],
                              I_tmds_channel_2[3],
                              I_tmds_channel_2[4],
                              I_tmds_channel_2[5],
                              I_tmds_channel_2[6],
                              I_tmds_channel_2[7],
                              I_tmds_channel_2[8],
                              I_tmds_channel_2[9]};

    assign S_tmds_data_clk = {I_tmds_channel_clk[0],
                              I_tmds_channel_clk[1],
                              I_tmds_channel_clk[2],
                              I_tmds_channel_clk[3],
                              I_tmds_channel_clk[4],
                              I_tmds_channel_clk[5],
                              I_tmds_channel_clk[6],
                              I_tmds_channel_clk[7],
                              I_tmds_channel_clk[8],
                              I_tmds_channel_clk[9]};


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u0_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),

        .I_data_in    ( S_tmds_data_ch0 ),
        .O_serial_out ( O_tmds_ch0_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u1_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_ch1 ),
        .O_serial_out ( O_tmds_ch1_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u2_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_ch2 ),
        .O_serial_out ( O_tmds_ch2_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u3_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_clk ),
        .O_serial_out ( O_tmds_clk_p    )
    );


endmodule





