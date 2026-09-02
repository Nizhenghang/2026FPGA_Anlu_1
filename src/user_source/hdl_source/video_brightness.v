module video_brightness(
    input  wire [23:0] I_rgb,
    input  wire [2:0]  I_level,
    output wire [23:0] O_rgb
);

assign O_rgb = {
    adjust_channel(I_rgb[23:16], I_level),
    adjust_channel(I_rgb[15:8],  I_level),
    adjust_channel(I_rgb[7:0],   I_level)
};

function [7:0] adjust_channel;
    input [7:0] ch;
    input [2:0] level;
    begin
        case (level)
            3'd0: adjust_channel = (ch < 8'd64)  ? 8'd0   : (ch - 8'd64);
            3'd1: adjust_channel = (ch < 8'd32)  ? 8'd0   : (ch - 8'd32);
            3'd2: adjust_channel = ch;
            3'd3: adjust_channel = (ch > 8'd223) ? 8'd255 : (ch + 8'd32);
            default: adjust_channel = (ch > 8'd191) ? 8'd255 : (ch + 8'd64);
        endcase
    end
endfunction

endmodule
