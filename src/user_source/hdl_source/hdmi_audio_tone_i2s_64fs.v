
module hdmi_audio_tone_i2s_64fs #(
    parameter [31:0] PHASE_INC = 32'd39370534,   // 兼容旧顶层，实际本版内部不用它
    // Square-wave amplitude before the envelope. The analyser's input scaling
    // was calibrated for a *steady* 2000000 (-12.5 dBFS), but the envelope's
    // sustain sits ENV_SUSTAIN=2 steps below the peak, so AMP itself has to be
    // 4x that. 8000000 is 24-bit legal (< 8388607), and the shift-0 attack
    // peak deliberately overdrives the analyser's sat16 mixer into clipping so
    // a note onset flashes the bar to full height.
    parameter signed [23:0] AMP = 24'sd8000000,
    parameter integer NOTE_HOLD_FRAMES = 24000   // 每个音持续 0.5s @ 48kHz
)(
    input  wire I_mclk,      // 12.288MHz
    input  wire I_rst,
    output reg  O_i2s_BCLK,
    output reg  O_i2s_LRCK,
    output reg  O_i2s_DOUT
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

// ---------------------------------------------------------------
// Segment table, 6.18 s loop (296448 samples):
//   0..7  eight notes, NOTE_HOLD_FRAMES each, with an ADSR envelope
//   8     up-sweep   100 Hz -> 7003 Hz over 69632 samples (1.451 s)
//   9     down-sweep 7003 Hz ->  100 Hz over 34816 samples (0.725 s)
//
// The sweep pair fades in at its start and out at its end over ENV_FADE_LEN, so
// all nine segment boundaries sit at the -48 dB floor. Nothing here ever resets
// the phase accumulators, so every frequency change -- note to note, note to
// sweep, sweep turnaround -- is phase-continuous. A phase jump or an amplitude
// jump would be a click, and a click is broadband: it lights all 16 analyser
// bands at once, which is exactly the imprecision the analyser was built to
// avoid.
// ---------------------------------------------------------------
localparam [3:0]  SEG_LAST      = 4'd9;
localparam [3:0]  SEG_SWEEP_UP  = 4'd8;
localparam [3:0]  SEG_LAST_NOTE = 4'd7;

// The sweep is GEOMETRIC, not linear: inc += inc >> K. Time spent inside band k
// is proportional to f_k, so a linear 100 -> 7000 Hz ramp dwells 6.4 ms in the
// lowest band -- less than one 10.7 ms envelope window, so that bar never rises
// -- and 445 ms in the highest, a 70x spread. A geometric ramp spends
// ln(1.3274)/ln(1+2^-K) samples in every band: 96.7 ms going up and 48.3 ms
// coming down, 9.1 and 4.5 envelope windows. One shift and one add, no
// multiplier, which is the house style (video_fade.v, video_brightness.v).
//
// Lengths are the smallest multiples of 32 whose integer iteration lands on
// frequency; tools/sim_tone_gen.py re-runs the same integer recurrence and
// asserts the endpoints rather than trusting these comments.
localparam [31:0] SWEEP_INC_LO    = 32'd8947848;   //  100.0 Hz
localparam [4:0]  SWEEP_SHIFT_UP   = 5'd14;
localparam [4:0]  SWEEP_SHIFT_DOWN = 5'd13;
localparam [16:0] SWEEP_UP_LEN    = 17'd69632;     // -> 7002.87 Hz, +0.04%
localparam [16:0] SWEEP_DOWN_LEN  = 17'd34816;     // ->   99.93 Hz, -0.07%

// ADSR boundaries in samples from the start of a note segment. The step lengths
// are powers of two -- 8 attack steps of 256, 2 decay steps of 1024, 6 release
// steps of 512 -- so the shift amount is a bit-select. The plan's 40/60/80 ms
// would need divisions by 240/1440/640; 42.7/42.7/64 ms buys a comparator-free
// ramp, and this is a test tone, not an instrument.
//
// The release finishes 448 samples before the segment ends and holds at the
// -48 dB floor for the remainder, so the note is already silent when the next
// one starts.
localparam [16:0] ENV_ATTACK_END  = 17'd2048;
localparam [16:0] ENV_DECAY_END   = 17'd4096;
localparam [16:0] ENV_RELEASE_BEG = NOTE_HOLD_FRAMES - 3520;   // 20480
localparam [16:0] ENV_TAIL_BEG    = NOTE_HOLD_FRAMES - 448;    // 23552
localparam [3:0]  ENV_SUSTAIN     = 4'd2;      // -12 dB
localparam [3:0]  ENV_FLOOR       = 4'd8;      // -48 dB

// Sweep fade length. Seven steps of 256 samples is exactly ENV_FLOOR down to
// ENV_SUSTAIN inclusive, so a fade arrives on the plateau level on the very
// sample the plateau starts instead of one 6 dB step above it. There is one
// fade-in (segment 8) and one fade-out (segment 9); the fade-out is the same
// expression read against W_cnt_rem, which is what makes it land on the floor at
// the segment's last sample.
localparam [16:0] ENV_FADE_LEN    = 17'd1792;  // 37.3 ms

reg  [5:0]  S_bit_cnt;
reg  [63:0] S_shift_reg;
reg  [31:0] S_phase_acc;
reg  [31:0] S_phase_acc2;
reg  [31:0] S_sweep_inc;
reg signed [23:0] S_sample_word;
reg signed [23:0] S_sample_next;
reg  [3:0]  S_seg_idx;
reg  [16:0] S_seg_frame_cnt;

wire        W_is_sweep_up   = (S_seg_idx == SEG_SWEEP_UP);
wire        W_is_sweep_down = (S_seg_idx == SEG_LAST);
wire        W_is_sweep      = W_is_sweep_up | W_is_sweep_down;
// NOTE_LEN is a separate 17-bit localparam rather than a term of the select
// below spelled out as NOTE_HOLD_FRAMES, which is a 32-bit integer parameter.
localparam [16:0] NOTE_LEN = NOTE_HOLD_FRAMES;

// Three-way, and it has to be: a two-way `is_sweep_up ? UP : DOWN` silently
// gives the eight note segments the down-sweep's length. That was harmless
// while SWEEP_DOWN_LEN was defined as NOTE_HOLD_FRAMES and became wrong the
// moment the two sweeps got different lengths.
wire [16:0] W_seg_len       = W_is_sweep_up   ? SWEEP_UP_LEN :
                              W_is_sweep_down ? SWEEP_DOWN_LEN : NOTE_LEN;
// Samples left after this one. Never negative: S_seg_frame_cnt < W_seg_len is
// the loop invariant.
wire [16:0] W_cnt_rem       = W_seg_len - 17'd1 - S_seg_frame_cnt;

wire [31:0] W_note_inc = note_inc_lut(S_seg_idx[2:0]);
wire [31:0] W_inc      = W_is_sweep ? S_sweep_inc : W_note_inc;

// Second voice detuned by +1/128 = +13.3 cents, which beats at 2..4 Hz across
// the note range and gives the bars something to breathe on even while a note
// holds a constant frequency.
wire [31:0] W_inc2 = W_inc + (W_inc >> 7);

wire signed [23:0] W_sq1 = S_phase_acc[31]  ?  AMP : -AMP;
wire signed [23:0] W_sq2 = S_phase_acc2[31] ?  AMP : -AMP;
// Halve before adding so the peak is still AMP, not 2*AMP.
wire signed [23:0] W_mix = (W_sq1 >>> 1) + (W_sq2 >>> 1);

// 0..8, one step = -6 dB. >>> on a signed operand is an arithmetic shift, so
// the sign survives without video_fade.v's per-channel scaling ladder. The
// right operand of a shift is always treated as unsigned, so W_env_shift being
// an unsigned 4-bit wire does not turn >>> into a logical shift.
//
// The sweep branch was a flat shift of 1 with no fade, which put a ±AMP/2 step
// at both of its boundaries. tools/sim_tone_gen.py measured it: worst sample
// adjacent to a boundary 1000000 against a floor of 15624, a click, and a click
// splashes all 16 bars at once.
//
// Only the OUTER ends of the sweep pair fade. Up-to-down needs nothing: the RTL
// leaves S_sweep_inc alone across that boundary, so frequency is continuous by
// construction, and both sides sit at ENV_SUSTAIN, so level is too. Fading all
// four ends instead cost the top band 1792 of its 2328 samples, leaving it 1.0
// envelope window at full level against the interior bands' 9.1 -- band 15 then
// never became the brightest bar, which the model reports.
wire [3:0] W_env_shift =
    W_is_sweep_up ? (
        (S_seg_frame_cnt < ENV_FADE_LEN)  ? (ENV_FLOOR - S_seg_frame_cnt[10:8]) :
        ENV_SUSTAIN) :
    W_is_sweep_down ? (
        (W_cnt_rem       < ENV_FADE_LEN)  ? (ENV_FLOOR - W_cnt_rem[10:8]) :
        ENV_SUSTAIN) :
    (S_seg_frame_cnt < ENV_ATTACK_END)  ? (ENV_FLOOR - S_seg_frame_cnt[10:8]) :
    (S_seg_frame_cnt < ENV_DECAY_END)   ? S_seg_frame_cnt[10] :
    (S_seg_frame_cnt < ENV_RELEASE_BEG) ? ENV_SUSTAIN :
    (S_seg_frame_cnt < ENV_TAIL_BEG)    ?
        (ENV_SUSTAIN + ((S_seg_frame_cnt - ENV_RELEASE_BEG) >> 9)) :
        ENV_FLOOR;

wire signed [23:0] W_sample = W_mix >>> W_env_shift;

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK      <= 1'b0;
        O_i2s_LRCK      <= 1'b0;
        O_i2s_DOUT      <= 1'b0;
        S_bit_cnt       <= 6'd0;
        S_shift_reg     <= 64'd0;
        S_phase_acc     <= 32'd0;
        S_phase_acc2    <= 32'd0;
        S_sweep_inc     <= SWEEP_INC_LO;
        S_sample_word   <= 24'sd0;
        // Was AMP. That primed the first frame at full scale, which was harmless
        // while the waveform had no envelope but is a lone impulse now that the
        // ADSR starts at the -48 dB floor -- a click at power-on, and a splash
        // across all 16 analyser bands in the first window.
        S_sample_next   <= 24'sd0;
        S_seg_idx       <= 4'd0;
        S_seg_frame_cnt <= 17'd0;
    end
    else begin
        // 保持和你现有接收端兼容的 64fs 发送时序
        O_i2s_BCLK <= ~O_i2s_BCLK;

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin
                    // 左声道发完，右声道复用同一个样本
                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {S_sample_word[23:0], 40'd0};
                end
                else begin
                    // 右声道发完，进入下一帧样本
                    O_i2s_LRCK  <= 1'b0;

                    // Sample value only. W_sample reads the phase accumulators
                    // as they stood at the start of this cycle, so the waveform
                    // trails the accumulator by one frame -- as it did before.
                    S_phase_acc   <= S_phase_acc  + W_inc;
                    S_phase_acc2  <= S_phase_acc2 + W_inc2;
                    S_sample_next <= W_sample;

                    S_sample_word <= S_sample_next;
                    S_shift_reg   <= {S_sample_next[23:0], 40'd0};

                    if (S_seg_frame_cnt == W_seg_len - 17'd1) begin
                        S_seg_frame_cnt <= 17'd0;
                        if (S_seg_idx == SEG_LAST)
                            S_seg_idx <= 4'd0;
                        else
                            S_seg_idx <= S_seg_idx + 4'd1;
                        // Preload on the way INTO segment 8 rather than out of
                        // it, so the up-sweep always starts at 100 Hz no matter
                        // how the down-sweep's last step rounded off.
                        if (S_seg_idx == SEG_LAST_NOTE)
                            S_sweep_inc <= SWEEP_INC_LO;
                    end
                    else begin
                        S_seg_frame_cnt <= S_seg_frame_cnt + 17'd1;
                        if (W_is_sweep_up)
                            S_sweep_inc <= S_sweep_inc + (S_sweep_inc >> SWEEP_SHIFT_UP);
                        else if (W_is_sweep_down)
                            S_sweep_inc <= S_sweep_inc - (S_sweep_inc >> SWEEP_SHIFT_DOWN);
                    end
                end
            end
            else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
