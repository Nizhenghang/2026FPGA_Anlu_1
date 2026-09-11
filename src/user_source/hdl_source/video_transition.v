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
// Fade, the non band effect
//   dim to black over FADE_MAX frames, hand the panel over to the target while
//   the screen is black, then brighten over FADE_MAX frames. The selectors stay
//   equal throughout, so frame_fifo_read's band engine is inert and O_effect is
//   driven to 0.
//
// Band effects, the vertical sweep family
//   O_top_idx is set to the target while O_bot_idx stays on the outgoing
//   picture, and O_effect carries a band code 1..6 or 8..11. frame_fifo_read
//   sees the two selectors disagree, freezes O_effect, and advances its own ramp
//   by WIPE_GRP_STEP two-line groups on every frame read; its select_top
//   function turns that ramp plus the code into a per-group buffer choice, so
//   the new picture sweeps in as a wipe, blinds, centre split, scrambled bars,
//   comb, pincer, two pass interlace, coarse blocks or quad interlace. All ten
//   share the same 30 frame ramp and the same group-boundary redirect mechanism,
//   so they are the old single wipe generalised bit for bit from one crossing to
//   many. The selectors are made equal again once the ramp has had time to
//   saturate, and that equality is also what resets the engine for the next
//   transition.
//
// Non band effects
//   7 (and the reserved 15) is the fade described above. Three more were added
//   for the serial screen and all reuse the same fade machinery rather than
//   adding a second dimmer: 12 is an instant cut, 13 halves the fade rate with
//   a one bit prescaler, 14 fades out, holds the panel black for BLACK_HOLD
//   frames in ST_BLACK, then fades up on the new picture.
//
//   I_mode is 4 bits. 0 is auto-cycle: effect_cnt rotates
//   7,1,2,3,4,5,6,8,9,10,11,7,... one per picture change, so a carousel shows
//   fade then every band effect in turn. effect_cnt resets to 7, which makes the
//   very first transition a fade. The chain deliberately SKIPS 12/13/14: an
//   instant cut in the middle of an unattended carousel reads as a glitch rather
//   than an effect, and the slow fade and black hold would stretch one picture
//   change past the 1 s auto play interval in sd_card_bmp. Including them is a
//   one line change to effect_cnt below. 1..6 force one of the original band
//   effects, 7 forces fade, 8..11 force one of the new band effects, 12..14
//   force the three non band modes above, 15 is reserved and behaves as fade.
//   The effect for a transition is sampled once, at the ST_IDLE/pending branch,
//   so a mid-flight DIP change cannot tear a transition in progress.
//
//   Codes 8..15 are reachable only from the serial screen: the DIP path at the
//   top level is {1'b0, ~sw[2:0]}, which can only ever produce 0..7, so the
//   behaviour of every physical switch setting is bit-for-bit what it was when
//   I_mode was 3 bits wide.
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
    parameter [5:0] WIPE_SETTLE = 6'd2,     // frames the selectors are held equal before a new transition may start
    parameter [5:0] BLACK_HOLD  = 6'd12     // frames the panel is held black by effect 14, ~0.2 s at 60 Hz
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_frame_start,       // one pulse per frame, see the alignment note above
    input  wire        I_display_valid,     // at least one picture has been committed
    input  wire [1:0]  I_disp_idx,          // the picture sd_card_bmp wants shown, synchronised
    input  wire [3:0]  I_mode,              // transition select, quasi static: 0 auto-cycle, 1..6 / 8..11 force that band effect, 7 fade, 12 instant cut, 13 slow fade, 14 black hold, 15 reserved = fade. Only 0..7 are reachable from the DIP switches.

    output reg  [1:0]  O_bot_idx,           // frame_fifo_read read_addr_index     : boundary line and below
    output reg  [1:0]  O_top_idx,           // frame_fifo_read read_addr_index_top : above the boundary
    output reg  [3:0]  O_effect,            // frame_fifo_read effect : band code 1..6 or 8..11, 0 means no band redirect (fade / idle)
    output reg  [1:0]  O_img_idx,           // what the OSD should call the current picture
    output reg  [3:0]  O_fade_level         // video_fade scale level, 0 is black
);

localparam [2:0] ST_IDLE     = 3'd0;
localparam [2:0] ST_FADE_OUT = 3'd1;
localparam [2:0] ST_FADE_IN  = 3'd2;
localparam [2:0] ST_BAND     = 3'd3;
localparam [2:0] ST_WIPE_END = 3'd4;
localparam [2:0] ST_BLACK    = 3'd5;

reg [2:0] state;
reg [1:0] cur_idx;          // the picture the panel is showing, i.e. what O_bot_idx will settle on
reg [1:0] tgt_idx;          // latched at transition start so a second advance mid transition cannot move the goalposts
reg [5:0] hold_cnt;
reg [3:0] effect_cnt;       // auto-cycle counter, rotates 7,1,2,3,4,5,6,8,9,10,11,7,... one per transition; 7 resets so the first is a fade
reg       dv_d;
// Two flags that reshape the fade without touching FADE_MAX. They exist because
// video_fade's scale_channel is 4 bits with 8 as unity gain, so levels 9..16 all
// fall into its default arm and stop attenuating: asking for a longer fade by
// raising FADE_MAX would dim for 8 frames and then sit at full brightness for
// the rest. Both are cleared at the start of every transition and are ignored
// by the band path, so with neither ever set the fade is bit-for-bit the old one.
reg       fade_slow;        // 1 = halve the fade rate using hold_cnt[0] as a prescaler
reg       fade_hold;        // 1 = insert ST_BLACK between the fade out and the index change

wire dv_rise = I_display_valid && !dv_d;
wire pending = (I_disp_idx != cur_idx);

// Effect select for the transition that is about to start. Auto-cycle (0)
// rotates the free running effect_cnt through fade and the ten band effects; 7
// forces fade; every other code forces itself. chosen_effect reads the OLD
// effect_cnt, matching non-blocking semantics, and use_band is the "drive the
// selectors apart and ramp" condition -- note it is an explicit pair of ranges,
// not "not 0 and not 7", because 12/13/14 must NOT reach the band engine and 15
// is reserved. Sampled combinationally at the ST_IDLE/pending branch below, i.e.
// once per transition, so a mid-flight DIP change cannot tear a transition
// already in progress.
wire [3:0] chosen_effect = (I_mode == 4'd0) ? effect_cnt :
                           (I_mode == 4'd7) ? 4'd7 :
                           I_mode;
wire       use_band      = ((chosen_effect >= 4'd1) && (chosen_effect <= 4'd6)) ||
                           ((chosen_effect >= 4'd8) && (chosen_effect <= 4'd11));

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        state        <= ST_IDLE;
        cur_idx      <= 2'd0;
        tgt_idx      <= 2'd0;
        hold_cnt     <= 6'd0;
        effect_cnt   <= 4'd7;
        dv_d         <= 1'b0;
        fade_slow    <= 1'b0;
        fade_hold    <= 1'b0;
        O_bot_idx    <= 2'd0;
        O_top_idx    <= 2'd0;
        O_effect     <= 4'd0;
        O_img_idx    <= 2'd0;
        O_fade_level <= 4'd0;
    end else begin
        dv_d <= I_display_valid;

        if (!I_display_valid) begin
            // Nothing committed, or the card was pulled: hold black and abandon
            // any half finished transition so the next commit starts clean.
            // Equalising the selectors here also stops a running band effect.
            state        <= ST_IDLE;
            hold_cnt     <= 6'd0;
            fade_slow    <= 1'b0;
            fade_hold    <= 1'b0;
            O_fade_level <= 4'd0;
            O_top_idx    <= O_bot_idx;
            O_effect     <= 4'd0;
        end else if (dv_rise) begin
            cur_idx      <= I_disp_idx;
            tgt_idx      <= I_disp_idx;
            O_bot_idx    <= I_disp_idx;
            O_top_idx    <= I_disp_idx;
            O_img_idx    <= I_disp_idx;
            O_fade_level <= 4'd0;
            O_effect     <= 4'd0;
            hold_cnt     <= 6'd0;
            fade_slow    <= 1'b0;
            fade_hold    <= 1'b0;
            state        <= ST_FADE_IN;
        end else if (I_frame_start) begin
            case (state)
                ST_IDLE: begin
                    if (pending) begin
                        tgt_idx   <= I_disp_idx;
                        hold_cnt  <= 6'd0;
                        // Cleared first so the case arms below win by last
                        // assignment; with neither ever set the fade is exactly
                        // the pre-existing one.
                        fade_slow <= 1'b0;
                        fade_hold <= 1'b0;
                        if (use_band) begin
                            // Only the top selector moves, and O_effect carries
                            // the band code. frame_fifo_read takes the
                            // disagreement as "start a band sweep" and ramps its
                            // boundary; select_top turns that ramp plus the code
                            // into the per-group buffer choice.
                            O_effect  <= chosen_effect;
                            O_top_idx <= I_disp_idx;
                            state     <= ST_BAND;
                        end else begin
                            O_effect  <= 4'd0;
                            case (chosen_effect)
                                4'd12: begin
                                    // Instant cut. Every index moves on this same
                                    // I_frame_start, exactly as the fade path moves
                                    // them, so the ext_mem_clk crossing stays a
                                    // quasi static 2FF. Because O_bot_idx and
                                    // O_top_idx are made equal here, the next frame
                                    // read sees wipe_sel_diff == 0, first_sel is 0,
                                    // cur_sel is 0 and base_bot is already the new
                                    // picture -- the band engine never has to be
                                    // told anything. O_fade_level is deliberately
                                    // left alone: in ST_IDLE it is necessarily
                                    // FADE_MAX, the value ST_FADE_IN saturates to.
                                    // ST_WIPE_END gives the same couple of settled
                                    // frames before another change is allowed.
                                    cur_idx   <= I_disp_idx;
                                    O_bot_idx <= I_disp_idx;
                                    O_top_idx <= I_disp_idx;
                                    O_img_idx <= I_disp_idx;
                                    state     <= ST_WIPE_END;
                                end
                                4'd13: begin
                                    fade_slow <= 1'b1;
                                    state     <= ST_FADE_OUT;
                                end
                                4'd14: begin
                                    fade_hold <= 1'b1;
                                    state     <= ST_FADE_OUT;
                                end
                                default: state <= ST_FADE_OUT;   //7 normal fade, and reserved 15
                            endcase
                        end
                        // The auto-cycle counter keeps running in every mode;
                        // forced modes ignore it, so advancing is harmless and
                        // switching back to auto resumes the rotation. The chain
                        // is fade, the six original band effects, the four new
                        // band effects, back to fade -- 12/13/14 are excluded on
                        // purpose, see the header.
                        effect_cnt <= (effect_cnt == 4'd7)  ? 4'd1 :
                                      (effect_cnt == 4'd6)  ? 4'd8 :
                                      (effect_cnt == 4'd11) ? 4'd7 :
                                      (effect_cnt + 4'd1);
                    end
                end

                ST_FADE_OUT: begin
                    // hold_cnt doubles as the fade_slow prescaler. It reads its
                    // OLD value in the condition below (non-blocking), so with
                    // fade_slow set the level moves on every other frame and the
                    // dim takes twice as many frames. With fade_slow clear the
                    // condition is unconditionally true and the level sequence is
                    // the original one.
                    hold_cnt <= hold_cnt + 6'd1;
                    if (O_fade_level <= 4'd1) begin
                        // This frame is rendered black, and the index change
                        // takes effect on the next frame's read request, which
                        // is the first frame the fade in below brightens.
                        O_fade_level <= 4'd0;
                        if (fade_hold) begin
                            // Effect 14: stay black for BLACK_HOLD frames before
                            // handing the panel over. The index change moves to
                            // the end of that hold, so the swap happens while the
                            // screen is genuinely dark rather than merely dim.
                            hold_cnt <= 6'd0;
                            state    <= ST_BLACK;
                        end else begin
                            cur_idx   <= tgt_idx;
                            O_bot_idx <= tgt_idx;
                            O_top_idx <= tgt_idx;
                            O_img_idx <= tgt_idx;
                            hold_cnt  <= 6'd0;
                            state     <= ST_FADE_IN;
                        end
                    end else if (!fade_slow || (hold_cnt[0] == 1'b1)) begin
                        O_fade_level <= O_fade_level - 4'd1;
                    end
                end

                ST_FADE_IN: begin
                    hold_cnt <= hold_cnt + 6'd1;
                    if (O_fade_level >= FADE_MAX) begin
                        O_fade_level <= FADE_MAX;
                        hold_cnt     <= 6'd0;
                        state        <= ST_IDLE;
                    end else if (!fade_slow || (hold_cnt[0] == 1'b1)) begin
                        O_fade_level <= O_fade_level + 4'd1;
                    end
                end

                ST_BLACK: begin
                    // Only reachable from effect 14. The level is already 0 and
                    // the selectors are still equal and still on the outgoing
                    // picture, so the panel is showing true black for the whole
                    // of this state.
                    if (hold_cnt >= (BLACK_HOLD - 6'd1)) begin
                        cur_idx   <= tgt_idx;
                        O_bot_idx <= tgt_idx;
                        O_top_idx <= tgt_idx;
                        O_img_idx <= tgt_idx;
                        hold_cnt  <= 6'd0;
                        state     <= ST_FADE_IN;
                    end else begin
                        hold_cnt <= hold_cnt + 6'd1;
                    end
                end

                ST_BAND: begin
                    if (hold_cnt >= (WIPE_HOLD - 6'd1)) begin
                        // The ramp has saturated, the whole panel already comes
                        // from the top buffer. Equalise the selectors and clear
                        // the effect code: that equality is frame_fifo_read's
                        // "no band" condition and resets its engine on the next
                        // frame read.
                        cur_idx   <= tgt_idx;
                        O_bot_idx <= tgt_idx;
                        O_img_idx <= tgt_idx;
                        hold_cnt  <= 6'd0;
                        O_effect  <= 4'd0;
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
