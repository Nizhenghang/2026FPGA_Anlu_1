module video_fade #(
    parameter [3:0] FADE_MAX = 4'd8
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_frame_start,
    input  wire        I_start,
    input  wire        I_display_valid,
    input  wire [23:0] I_rgb,
    output wire [23:0] O_rgb
);

reg [3:0] fade_level;
reg       fade_active;

assign O_rgb = I_display_valid ? {
    scale_channel(I_rgb[23:16], fade_level),
    scale_channel(I_rgb[15:8],  fade_level),
    scale_channel(I_rgb[7:0],   fade_level)
} : 24'd0;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        fade_level  <= FADE_MAX;
        fade_active <= 1'b0;
    end else if (!I_display_valid) begin
        fade_level  <= 4'd0;
        fade_active <= 1'b0;
    end else if (I_start) begin
        fade_level  <= 4'd0;
        fade_active <= 1'b1;
    end else if (fade_active && I_frame_start) begin
        if (fade_level >= FADE_MAX) begin
            fade_level  <= FADE_MAX;
            fade_active <= 1'b0;
        end else begin
            fade_level <= fade_level + 4'd1;
        end
    end
end

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
