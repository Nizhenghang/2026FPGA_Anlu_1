
// ============================================================================
// 文件：video_rgb_to_axis_640x480.v
// 功能：把并行 RGB + 时序(hs/vs/de) 打包成 AXI4-Stream 视频格式(供 HDMI 发射核)
// 接口：输入 I_rgb(24bit) + I_de/I_vs；输出 O_video_user(SOF 帧首) / O_video_valid(=de)
//       / O_video_last(帧末) / O_video_data(24bit RGB)
// 关键逻辑：
//   - O_video_valid 直接等于 I_de(有效像素期间为高)
//   - SOF(user)：检测到一次 VS 边沿后，把"下一行首像素"标记成帧首(S_frame_arm 机制)
//   - last：每行第 640 个像素(S_x_cnt==639)拉高，标记一帧最后一个有效像素
//   协议细节见安路 HDMI 发射核 AXI-Stream 视频接口要求
// ============================================================================
module video_rgb_to_axis_640x480(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    output reg         O_video_user,
    output reg         O_video_valid,
    output reg         O_video_last,
    output reg  [23:0] O_video_data
);

reg        S_vs_1d;
reg        S_de_1d;
reg        S_frame_arm;
reg [10:0] S_x_cnt;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_vs_1d       <= 1'b0;
        S_de_1d       <= 1'b0;
        S_frame_arm   <= 1'b1;
        S_x_cnt       <= 11'd0;
        O_video_user  <= 1'b0;
        O_video_valid <= 1'b0;
        O_video_last  <= 1'b0;
        O_video_data  <= 24'd0;
    end
    else begin
        S_vs_1d <= I_vs;
        S_de_1d <= I_de;

        O_video_user  <= 1'b0;
        O_video_valid <= I_de;
        O_video_last  <= 1'b0;
        O_video_data  <= I_rgb;

        // 任意一次 VS 跳变后，把“下一次 active video 的首像素”当成 SOF
        if (S_vs_1d != I_vs)
            S_frame_arm <= 1'b1;

        if (I_de) begin
            if (!S_de_1d) begin
                // 一行首像素
                S_x_cnt <= 11'd0;
                if (S_frame_arm) begin
                    O_video_user <= 1'b1;
                    S_frame_arm  <= 1'b0;
                end
            end
            else begin
                S_x_cnt <= S_x_cnt + 11'd1;
            end

            // 640x480 最后一个有效像素
            if (S_x_cnt == 11'd639)
                O_video_last <= 1'b1;
        end
        else begin
            S_x_cnt <= 11'd0;
        end
    end
end

endmodule
