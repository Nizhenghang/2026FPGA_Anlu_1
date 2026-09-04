// Consume stereo PCM frames from the async FIFO (written by sd_audio_stream in
// the sd_card_clk domain) and drive the HDMI audio interface in the video_clk
// domain at a continuous 48 kHz.
//
// A fractional accumulator turns the 25 MHz video_clk into a 48 kHz sample tick
// (same scheme as hdmi_audio_tone_pcm_scale). Every tick asserts O_audio_valid
// so the downstream ACR block (audio_arc_calculate, one CTS every 48 valids) and
// the HDMI audio clock regeneration never lose their reference -- INCLUDING when
// the FIFO is empty. On underrun (startup, missing WAV, or a stream fault) the
// samples are forced to zero but the valid stream keeps running: silence, not a
// stalled clock.
//
// FIFO word layout from sd_audio_stream is {R[15:0], L[15:0]}. Each 16-bit
// sample is left-justified into the 24-bit HDMI audio word ({s,8'b0}), which
// preserves two's-complement sign and matches the amplitude scale the existing
// visualizer was tuned for.
module audio_pcm_player #(
    parameter integer CLK_FREQ_HZ    = 25_000_000,
    parameter integer SAMPLE_RATE_HZ = 48_000
)(
    input  wire        I_clk,            // video_clk (25 MHz)
    input  wire        I_rst,

    // FIFO read side (show-ahead): dout is the head word when rdusedw != 0.
    output reg         fifo_re,          // single-cycle pop pulse
    input  wire [31:0] fifo_dout,        // {R[15:0], L[15:0]}
    input  wire [8:0]  fifo_rdusedw,     // words available to read

    output reg         O_audio_valid,
    output reg  [23:0] O_audio_left_data,
    output reg  [23:0] O_audio_right_data
);

reg  [31:0] S_sample_acc;
wire [31:0] W_acc_next = S_sample_acc + SAMPLE_RATE_HZ;
wire        W_tick     = (W_acc_next >= CLK_FREQ_HZ);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_sample_acc       <= 32'd0;
        fifo_re            <= 1'b0;
        O_audio_valid      <= 1'b0;
        O_audio_left_data  <= 24'd0;
        O_audio_right_data <= 24'd0;
    end else begin
        // fifo_re and O_audio_valid are single-cycle by default; the tick below
        // re-arms them once every ~520 clocks.
        fifo_re       <= 1'b0;
        O_audio_valid <= 1'b0;

        if (W_tick) begin
            S_sample_acc <= W_acc_next - CLK_FREQ_HZ;

            // ACR/HDMI reference must never gap: valid every tick, even silent.
            O_audio_valid <= 1'b1;

            if (fifo_rdusedw != 9'd0) begin
                // Show-ahead head is live on fifo_dout; pop it for the next tick.
                fifo_re            <= 1'b1;
                O_audio_left_data  <= {fifo_dout[15:0],  8'b0};
                O_audio_right_data <= {fifo_dout[31:16], 8'b0};
            end else begin
                // Underrun: keep the clock alive with silence.
                O_audio_left_data  <= 24'd0;
                O_audio_right_data <= 24'd0;
            end
        end else begin
            S_sample_acc <= W_acc_next;
        end
    end
end

endmodule
