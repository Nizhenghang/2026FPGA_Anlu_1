

////////////////////////////////////////////////////////////////////////////////
// 模块: I2S_receiver
// 功能: HDMI 音频测试音的 I2S 接收端(比特串行 -> 并行 24bit L/R PCM)
//       把发送端 hdmi_audio_tone_i2s_64fs 的 BCLK/LRCK/DOUT 恢复为
//       并行的 O_audio_left_data / O_audio_right_data(各 24bit) 及 O_audio_valid.
// 时钟域: I_clk = 12.288MHz(与发送端同主时钟, 异步采样用 3 级同步器打拍)
// 接口:
//   I_i2s_BCLK/LRCK/DOUT : I2S 串行输入
//   O_audio_valid        : 一个音频帧(左右声道)就绪脉冲
//   O_audio_left_data    : 左声道 24bit PCM
//   O_audio_right_data   : 右声道 24bit PCM
// 关键算法:
//   S_i2s_*_1d/2d/3d  : 对 BCLK/LRCK/DOUT 做 3 级同步器, 消除跨时钟亚稳态
//   S_i2s_bclk_p_edge : BCLK 上升沿检测(用于逐位采样)
//   每个 LRCK 半周期把 64bit 串入移位寄存器, 取高 24bit([63:40])作为 PCM
////////////////////////////////////////////////////////////////////////////////
module I2S_receiver (
    input wire       I_clk,
    input wire       I_rst,
  
    input wire       I_i2s_BCLK,
    input wire       I_i2s_LRCK,
    input wire       I_i2s_DOUT,

    output reg       O_audio_valid,
    output reg[23:0] O_audio_left_data,
    output reg[23:0] O_audio_right_data
);


    reg       S_i2s_bclk_1d;
    reg       S_i2s_bclk_2d;
    reg       S_i2s_bclk_3d;
  
    wire      S_i2s_bclk_p_edge;  
  
    reg       S_i2s_lrck_1d;
    reg       S_i2s_lrck_2d;
    reg       S_i2s_lrck_3d;
    reg       S_i2s_lrck_sync;
    wire      S_i2s_lrck_p_edge;
    wire      S_i2s_lrck_n_edge;
  
    reg       S_i2s_dout_1d;
    reg       S_i2s_dout_2d;
    reg       S_i2s_dout_3d;

    reg[63:0] S_left_shift_data;
    reg[63:0] S_right_shift_data;

    reg       S_left_data_valid;
    reg       S_right_data_valid;

    reg[63:0] S_left_data_lock;

    // 3 级同步器: 把异步 I2S 信号(B/LR/DOUT)打 3 拍, 避免跨时钟域亚稳态
    always @(posedge I_clk) begin
        S_i2s_bclk_1d <= I_i2s_BCLK;
        S_i2s_bclk_2d <= S_i2s_bclk_1d;
        S_i2s_bclk_3d <= S_i2s_bclk_2d;

        S_i2s_lrck_1d <= I_i2s_LRCK;
        S_i2s_lrck_2d <= S_i2s_lrck_1d;
        S_i2s_lrck_3d <= S_i2s_lrck_2d;

        S_i2s_dout_1d <= I_i2s_DOUT;
        S_i2s_dout_2d <= S_i2s_dout_1d;
        S_i2s_dout_3d <= S_i2s_dout_2d;
    end

    assign S_i2s_bclk_p_edge = ~S_i2s_bclk_3d & S_i2s_bclk_2d; // BCLK 上升沿(用 3d/2d 打拍检测)


    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            S_i2s_lrck_sync <= S_i2s_lrck_3d;
        else
            S_i2s_lrck_sync <= S_i2s_lrck_sync;
    end

    assign S_i2s_lrck_p_edge = ~S_i2s_lrck_sync & S_i2s_lrck_3d;

    assign S_i2s_lrck_n_edge = ~S_i2s_lrck_3d & S_i2s_lrck_sync;

    // 在 BCLK 上升沿逐位把 DOUT 串入移位寄存器; LRCK 决定当前是左/右声道
    // 左声道时清右寄存器, 右声道时清左寄存器(两声道各占一个 64bit 帧)
    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            if(S_i2s_lrck_sync)
                begin
                    S_right_shift_data <= {S_right_shift_data[62:0],S_i2s_dout_3d};
                    S_left_shift_data  <= 'd0;
                end
            else
                begin
                    S_right_shift_data <= 'd0;
                    S_left_shift_data  <= {S_left_shift_data[62:0],S_i2s_dout_3d};
                end
        else
            begin
                S_right_shift_data <= S_right_shift_data;
                S_left_shift_data  <= S_left_shift_data;
            end
    end


    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_p_edge)
            S_left_data_valid <= 1'b1;
        else
            S_left_data_valid <= 1'b0;
    end

    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_n_edge)
            S_right_data_valid <= 1'b1;
        else
            S_right_data_valid <= 1'b0;
    end

    always @(posedge I_clk) begin
        if(S_left_data_valid)
            S_left_data_lock <= S_left_shift_data;
        else
            S_left_data_lock <= S_left_data_lock;
    end

    always @(posedge I_clk) begin
        if(S_right_data_valid)
            begin
                O_audio_valid      <= 1'b1;
                // 取 64bit 移位寄存器的高 24bit([63:40])作为该声道 24bit PCM
                O_audio_left_data  <= S_left_data_lock[63:40];
                O_audio_right_data <= S_right_shift_data[63:40];
            end
        else
            begin
                O_audio_valid      <= 1'b0;
                O_audio_left_data  <= 'd0;
                O_audio_right_data <= 'd0;
            end
    end

    // always @(posedge I_clk) begin
    //         begin
    //             if(S_i2s_lrck_3d)
    //                 begin
    //                     S_right_channel_cnt <= S_right_channel_cnt + 1'b1;
    //                     S_left_channel_cnt  <= 'd0;
    //                 end
    //             else    
    //                 begin
    //                     S_right_channel_cnt <= 'd0;
    //                     S_left_channel_cnt  <= S_left_channel_cnt + 1'b1;
    //                 end
    //         end
    //     else
    //         begin
    //             S_right_channel_cnt <= S_right_channel_cnt;
    //             S_left_channel_cnt  <= S_left_channel_cnt;
    //         end
    // end


    

endmodule