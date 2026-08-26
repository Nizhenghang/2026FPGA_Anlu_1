


////////////////////////////////////////////////////////////////////////////////
// 模块: audio_arc_calculate
// 功能: 计算 HDMI 音频时钟再生(ACR)包所需的 CTS / N 值
//       HDMI 接收端用 CTS/N 从 TMDS 时钟恢复出音频采样时钟(fs=48kHz).
//       本模块在每 48 个音频采样时刻之间对 I_clk 计数, 计数值即 CTS; N 固定 6144.
// 时钟域: I_clk (音频相关时钟, 此处与 I2S 同主时钟域)
// 接口:
//   I_audio_valid : 一个音频样本有效脉冲(来自 I2S_receiver)
//   O_acr_valid   : 一次 ACR 更新有效
//   O_acr_cts     : CTS 值(音频周期内 TMDS 时钟数)
//   O_acr_n       : N 值(固定 6144, 48kHz 标准)
// 关键算法:
//   AUDIO_DIV = ACR_N>>7 = 48 : 每累计 48 个 audio_valid 更新一次 ACR
//   S_acr_cts_cnt 在两次更新之间对 I_clk 计数 -> 该值即 CTS
////////////////////////////////////////////////////////////////////////////////
module audio_arc_calculate #(
    parameter ACR_N = 6144
    )(
    input wire       I_clk,
    input wire       I_rst,

    input wire       I_audio_valid,

    output reg       O_acr_valid,
    output reg[19:0] O_acr_cts,
    output reg[19:0] O_acr_n
);


    localparam AUDIO_DIV = ACR_N >> 7;  // = 48 : 每累计 48 个 audio_valid 输出一次 ACR(每 48 采样更新 CTS)

    reg[7:0]  S_audio_div_cnt;
    wire      S_audio_valid_div;
    reg[19:0] S_acr_cts_cnt;


    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_audio_div_cnt <= 'd0;
        else
            if(I_audio_valid)
                begin
                    if(S_audio_div_cnt >= AUDIO_DIV-1)
                        S_audio_div_cnt <= 'd0;
                    else
                        S_audio_div_cnt <= S_audio_div_cnt + 'd1;
                end
            else
                S_audio_div_cnt <= S_audio_div_cnt;
    end


    assign S_audio_valid_div = I_audio_valid && (S_audio_div_cnt == AUDIO_DIV-1) ? 1'b1 : 1'b0;

    // 在两次 ACR 更新之间(每 48 采样)对 I_clk 计数 -> 该值即 CTS
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_acr_cts_cnt <= 'd0;
        else
            if(S_audio_valid_div)
                S_acr_cts_cnt <= 'd0;
            else
                S_acr_cts_cnt <= S_acr_cts_cnt + 'd1;
    end


    always @(posedge I_clk) begin
        if(S_audio_valid_div)
            begin
                O_acr_valid <= 1'b1;
                O_acr_cts   <= S_acr_cts_cnt + 'd1;
                O_acr_n     <= ACR_N;
            end
        else    
            begin
                O_acr_valid <= 1'b0;
                O_acr_cts   <= 'd0;
                O_acr_n     <= 'd0;
            end
    end

    
endmodule