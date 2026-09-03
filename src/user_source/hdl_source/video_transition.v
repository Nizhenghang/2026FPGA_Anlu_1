// ---------------------------------------------------------------------------
// video_transition -- stage 4 transition effects, video_clk domain.
//
// What it owns
//   The two buffer selectors that frame_fifo_read turns into read base
//   addresses, and the fade level that video_fade scales by. sd_card_bmp still
//   decides WHICH picture is current and when it advances; this module decides
//   WHEN THE PANEL GETS TO SEE that decision, and how.
//
// Why the panel index is decoupled from disp_buf_idx
//   All four pictures are loaded into the four SDRAM frame buffers once, during
//   the initial scan, and after that nothing is ever written to SDRAM again:
//   sd_card_bmp only re-arms a load while img_loaded_count < SCAN_TARGET_COUNT,
//   and that counter saturates at 4. So a buffer switch is purely an index
//   change with no load latency, and both the outgoing and the incoming buffer
//   stay valid for the whole transition. That is what makes the wipe below safe
//   to read from two buffers in one frame -- there is no writer to race with.
//
// Effect 1, fade out / fade in
//   dim to black over FADE_MAX frames, hand the panel over to the target while
//   the screen is black, then brighten over FADE_MAX frames.
//
// Effect 2, vertical wipe
//   O_top_idx is set to the target while O_bot_idx stays on the outgoing
//   picture. frame_fifo_read advances its own boundary by WIPE_GRP_STEP
//   two-line groups on every frame read it sees the two selectors disagree, so
//   the new picture is revealed from the top of the panel downwards. The
//   selectors are made equal again once the boundary has had time to saturate,
//   and that equality is also what resets the boundary for the next wipe.
//
//   The two effects alternate, one per picture change, so a carousel of four
//   pictures shows both of them within a single pass. mode_wipe resets to 0,
//   which makes the first transition a fade.
//
// Frame alignment, and why the swap lands where it does
//   video_timing_data raises read_req on the vsync edge, and I_frame_start here
//   is that same edge delayed by video_delay's 20 tap shift register, so
//   I_frame_start fires about 20 video clocks AFTER the read request that
//   fetches this frame's pixels. An index change made on I_frame_start of frame
//   N therefore first reaches the panel on frame N+1.
//
//   The fade uses that deliberately: the level reaches 0 on frame N (black,
//   still reading the outgoing buffer) and the indices change on the same
//   I_frame_start, so frame N+1 both reads the target buffer and is the first
//   frame the fade in brightens. One black frame, no visible cut.
//
//   The wipe absorbs it as a constant one frame offset between the hold
//   counter here and the group counter in frame_fifo_read, which is why
//   WIPE_HOLD has to exceed the ramp length rather than equal it.
//
// Clock domain crossings
//   I_disp_idx arrives already two flip flop synchronised into video_clk by the
//   caller. O_bot_idx / O_top_idx cross into ext_mem_clk inside
//   frame_fifo_read, which synchronises them the same way it has always
//   synchronised read_addr_index. They only ever change on I_frame_start,
//   roughly 100 ext_mem_clk cycles before the S_ACK that samples them, so the
//   crossing is quasi static by construction rather than by luck.
//
//   Moving the selector source from sd_card_clk to video_clk is also a small
//   CDC improvement over the previous wiring, which fed disp_buf_idx straight
//   from the SD control domain into frame_read_write.
//
// Reset polarity
//   I_rst is rst_all at the top level and is ACTIVE HIGH, and the whole project
//   uses `always @(posedge clk or posedge rst) if (rst) ...` -- see
//   audio_visualizer, osd_overlay, video_rgb_to_axis and the three sync blocks
//   in top_tf_hdmi_audio. Writing `if (!I_rst)` against a `posedge I_rst`
//   sensitivity list does not merely look odd: the async reset then never
//   fires, and the else branch is evaluated on the rising edge of I_rst, so
//   synthesis infers an asynchronous SET alongside the asynchronous RESET for
//   every register here. The registers would power up to zero on the FPGA and
//   appear to work, but an audio_pll_lock dropout would leave this module
//   frozen mid transition with the two selectors held apart, i.e. a wipe
//   boundary stuck halfway down the panel. Keep the polarity as written.
// ---------------------------------------------------------------------------

module video_transition #(
    parameter [3:0] FADE_MAX    = 4'd8,     // fade steps, one per frame, matches the old video_fade default
    parameter [5:0] WIPE_HOLD   = 6'd40,    // frames the selectors are held apart
    parameter [5:0] WIPE_SETTLE = 6'd2      // frames the selectors are held equal before a new transition may start
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_frame_start,       // one pulse per frame, see the alignment note above
    input  wire        I_display_valid,     // at least one picture has been committed
    input  wire [1:0]  I_disp_idx,          // the picture sd_card_bmp wants shown, synchronised

    output reg  [1:0]  O_bot_idx,           // frame_fifo_read read_addr_index     : boundary line and below
    output reg  [1:0]  O_top_idx,           // frame_fifo_read read_addr_index_top : above the boundary
    output reg  [1:0]  O_img_idx,           // what the OSD should call the current picture
    output reg  [3:0]  O_fade_level         // video_fade scale level, 0 is black
);

localparam [2:0] ST_IDLE     = 3'd0;
localparam [2:0] ST_FADE_OUT = 3'd1;
localparam [2:0] ST_FADE_IN  = 3'd2;
localparam [2:0] ST_WIPE     = 3'd3;
localparam [2:0] ST_WIPE_END = 3'd4;

reg [2:0] state;
reg [1:0] cur_idx;          // the picture the panel is showing, i.e. what O_bot_idx will settle on
reg [1:0] tgt_idx;          // latched at transition start so a second advance mid transition cannot move the goalposts
reg [5:0] hold_cnt;
reg       mode_wipe;        // alternates, 0 is fade
reg       dv_d;

wire dv_rise = I_display_valid && !dv_d;
wire pending = (I_disp_idx != cur_idx);

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        state        <= ST_IDLE;
        cur_idx      <= 2'd0;
        tgt_idx      <= 2'd0;
        hold_cnt     <= 6'd0;
        mode_wipe    <= 1'b0;
        dv_d         <= 1'b0;
        O_bot_idx    <= 2'd0;
        O_top_idx    <= 2'd0;
        O_img_idx    <= 2'd0;
        O_fade_level <= 4'd0;
    end else begin
        dv_d <= I_display_valid;

        if (!I_display_valid) begin
            // Nothing committed, or the card was pulled: hold black and abandon
            // any half finished transition so the next commit starts clean.
            // Equalising the selectors here also stops a running wipe.
            state        <= ST_IDLE;
            hold_cnt     <= 6'd0;
            O_fade_level <= 4'd0;
            O_top_idx    <= O_bot_idx;
        end else if (dv_rise) begin
            cur_idx      <= I_disp_idx;
            tgt_idx      <= I_disp_idx;
            O_bot_idx    <= I_disp_idx;
            O_top_idx    <= I_disp_idx;
            O_img_idx    <= I_disp_idx;
            O_fade_level <= 4'd0;
            hold_cnt     <= 6'd0;
            state        <= ST_FADE_IN;
        end else if (I_frame_start) begin
            case (state)
                ST_IDLE: begin
                    if (pending) begin
                        tgt_idx   <= I_disp_idx;
                        hold_cnt  <= 6'd0;
                        mode_wipe <= ~mode_wipe;
                        if (mode_wipe) begin
                            // Only the top selector moves. frame_fifo_read
                            // takes the disagreement as "start a wipe" and
                            // ramps its boundary from the top of the panel.
                            O_top_idx <= I_disp_idx;
                            state     <= ST_WIPE;
                        end else begin
                            state     <= ST_FADE_OUT;
                        end
                    end
                end

                ST_FADE_OUT: begin
                    if (O_fade_level <= 4'd1) begin
                        // This frame is rendered black, and the index change
                        // takes effect on the next frame's read request, which
                        // is the first frame the fade in below brightens.
                        O_fade_level <= 4'd0;
                        cur_idx      <= tgt_idx;
                        O_bot_idx    <= tgt_idx;
                        O_top_idx    <= tgt_idx;
                        O_img_idx    <= tgt_idx;
                        state        <= ST_FADE_IN;
                    end else begin
                        O_fade_level <= O_fade_level - 4'd1;
                    end
                end

                ST_FADE_IN: begin
                    if (O_fade_level >= FADE_MAX) begin
                        O_fade_level <= FADE_MAX;
                        state        <= ST_IDLE;
                    end else begin
                        O_fade_level <= O_fade_level + 4'd1;
                    end
                end

                ST_WIPE: begin
                    if (hold_cnt >= (WIPE_HOLD - 6'd1)) begin
                        // The boundary has saturated, the whole panel already
                        // comes from the top buffer. Equalise: that equality is
                        // frame_fifo_read's "no wipe" condition and resets its
                        // group counter on the next frame read.
                        cur_idx   <= tgt_idx;
                        O_bot_idx <= tgt_idx;
                        O_img_idx <= tgt_idx;
                        hold_cnt  <= 6'd0;
                        state     <= ST_WIPE_END;
                    end else begin
                        hold_cnt <= hold_cnt + 6'd1;
                    end
                end

                ST_WIPE_END: begin
                    // A couple of frames of guaranteed equality before another
                    // transition is allowed to pull the selectors apart again.
                    if (hold_cnt >= (WIPE_SETTLE - 6'd1)) begin
                        hold_cnt <= 6'd0;
                        state    <= ST_IDLE;
                    end else begin
                        hold_cnt <= hold_cnt + 6'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
end

endmodule
