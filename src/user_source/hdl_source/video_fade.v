// ---------------------------------------------------------------------------
// video_fade -- stage 4, pure combinational brightness scaling stage.
//
// The level counter that used to live here moved to video_transition.v. This
// module used to be given a one shot start pulse and count the ramp itself,
// which cannot work any more: the sequence is now fade out, hand the buffer
// over while the screen is black, fade in, and that has to be timed against
// the frame boundary, which is only visible from the video_clk domain controller.
// scale_channel and the output mux are unchanged from the version that was
// verified on hardware, so the visible ramp is bit for bit the same; the only
// difference is that the level arrives on a port instead of being generated.
//
// Level 0 is black, FADE_MAX in video_transition (8) is unity gain. The steps
// in between are shift and add approximations of n/8, which is why no
// multiplier and no DSP is involved.
// ---------------------------------------------------------------------------

module video_fade #(
    parameter [3:0] FADE_MAX = 4'd8      // documentation only: the level is driven externally
)(
    input  wire        I_display_valid,
    input  wire [3:0]  I_level,
    input  wire [23:0] I_rgb,
    output wire [23:0] O_rgb
);

assign O_rgb = I_display_valid ? {
    scale_channel(I_rgb[23:16], I_level),
    scale_channel(I_rgb[15:8],  I_level),
    scale_channel(I_rgb[7:0],   I_level)
} : 24'd0;

function [7:0] scale_channel;
    input [7:0] ch;
    input [3:0] level;
    begin
        case (level)
            4'd0: scale_channel = 8'd0;
            4'd1: scale_channel = ch >> 3;
            4'd2: scale_channel = ch >> 2;
            4'd3: scale_channel = (ch >> 2) + (ch >> 3);
            4'd4: scale_channel = ch >> 1;
            4'd5: scale_channel = (ch >> 1) + (ch >> 3);
            4'd6: scale_channel = (ch >> 1) + (ch >> 2);
            4'd7: scale_channel = ch - (ch >> 3);
            default: scale_channel = ch;
        endcase
    end
endfunction

endmodule
