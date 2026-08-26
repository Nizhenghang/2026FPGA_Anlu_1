
////////////////////////////////////////////////////////////////////////////////
// 模块: hdmi_audio_tone_pcm_scale
// 功能: 并行 24bit PCM 音频测试音发生器(八音阶方波, do-re-mi-...-do)
//       直接输出并行的 O_audio_left_data / O_audio_right_data, 而非 I2S 串行.
// 说明: 本项目顶层实际使用的是 hdmi_audio_tone_i2s_64fs(I2S 串行发送端);
//       本文件为备用/对比实现(并行 PCM 版), 供不需要 I2S 链路时直接使用.
// 时钟域: I_clk (默认 25MHz, 由参数 CLK_FREQ_HZ 指定)
// 关键算法:
//   分数分频: S_sample_acc 每周期累加 SAMPLE_RATE_HZ, 溢出(>=CLK_FREQ_HZ)即一个
//             48kHz 采样点 -> 产生 O_audio_valid.
//   1bit DDS: S_phase_acc 累加 note_inc_lut(频率=inc*fs/2^32), 最高位判决 ±AMP 方波.
//   每个音持续 NOTE_HOLD_SAMPLES(默认 24000=0.5s@48k)后切换到下一音阶.
////////////////////////////////////////////////////////////////////////////////
module hdmi_audio_tone_pcm_scale #(
    parameter integer CLK_FREQ_HZ       = 25_000_000,
    parameter integer SAMPLE_RATE_HZ    = 48_000,
    parameter signed [23:0] AMP         = 24'sd2000000,
    parameter integer NOTE_HOLD_SAMPLES = 24_000
)(
    input  wire        I_clk,
    input  wire        I_rst,
    output reg         O_audio_valid,
    output reg  [23:0] O_audio_left_data,
    output reg  [23:0] O_audio_right_data
);

// do/re/mi/fa/so/la/si/do  （C4 D4 E4 F4 G4 A4 B4 C5）
// 相位增量按 48kHz 采样率计算
function [31:0] note_inc_lut;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: note_inc_lut = 32'd23409862; // do  C4 261.63Hz
            3'd1: note_inc_lut = 32'd26276681; // re  D4 293.66Hz
            3'd2: note_inc_lut = 32'd29494578; // mi  E4 329.63Hz
            3'd3: note_inc_lut = 32'd31248410; // fa  F4 349.23Hz
            3'd4: note_inc_lut = 32'd35075155; // so  G4 392.00Hz
            3'd5: note_inc_lut = 32'd39370534; // la  A4 440.00Hz
            3'd6: note_inc_lut = 32'd44191930; // si  B4 493.88Hz
            default: note_inc_lut = 32'd46819716; // do  C5 523.25Hz
        endcase
    end
endfunction

reg [31:0] S_sample_acc;
reg [31:0] S_phase_acc;
reg [2:0]  S_note_idx;
reg [31:0] S_note_sample_cnt;
reg signed [23:0] S_pcm_sample;
reg [31:0] S_acc_add;

wire [31:0] W_note_inc = note_inc_lut(S_note_idx);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        O_audio_valid      <= 1'b0;
        O_audio_left_data  <= 24'd0;
        O_audio_right_data <= 24'd0;
        S_sample_acc       <= 32'd0;
        S_phase_acc        <= 32'd0;
        S_note_idx         <= 3'd0;
        S_note_sample_cnt  <= 32'd0;
        S_pcm_sample       <= 24'sd0;
        S_acc_add          <= 32'd0;
    end else begin
        O_audio_valid <= 1'b0;
        S_acc_add = S_sample_acc + SAMPLE_RATE_HZ;

        // 分数分频产生 48kHz 采样使能
        if (S_acc_add >= CLK_FREQ_HZ) begin
            S_sample_acc  <= S_acc_add - CLK_FREQ_HZ;
            O_audio_valid <= 1'b1;

            // 下一个采样点: 1bit DDS 相位累加, 最高位判决方波极性
            S_phase_acc <= S_phase_acc + W_note_inc;  // 频率 = W_note_inc * fs / 2^32
            if (S_phase_acc[31])
                S_pcm_sample <= AMP;                  // 最高位=1 -> 正半周
            else
                S_pcm_sample <= -AMP;                 // 最高位=0 -> 负半周

            O_audio_left_data  <= S_pcm_sample;
            O_audio_right_data <= S_pcm_sample;

            // do/re/mi/fa/so/la/si/do 循环
            if (S_note_sample_cnt == NOTE_HOLD_SAMPLES - 1) begin
                S_note_sample_cnt <= 32'd0;
                if (S_note_idx == 3'd7)
                    S_note_idx <= 3'd0;
                else
                    S_note_idx <= S_note_idx + 3'd1;
            end else begin
                S_note_sample_cnt <= S_note_sample_cnt + 32'd1;
            end
        end else begin
            S_sample_acc <= S_acc_add;
        end
    end
end

endmodule
