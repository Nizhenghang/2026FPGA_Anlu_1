module video_effect(
    input  wire [23:0] I_rgb,
    input  wire [3:0]  I_sel,
    output wire [23:0] O_rgb
);

wire [7:0] r;
wire [7:0] g;
wire [7:0] b;

wire [7:0]  luma;
wire [10:0] g_mul5;
wire [9:0]  g_mul3;
wire [23:0] thresh;
wire [23:0] sepia;
wire [23:0] contrast;
wire [23:0] saturate;
wire [23:0] mute;
wire [23:0] amber;
wire [23:0] cool;
wire [23:0] solar;
wire [23:0] ginv;

assign r = I_rgb[23:16];
assign g = I_rgb[15:8];
assign b = I_rgb[7:0];

assign luma    = gray_y(r, g, b);
assign thresh  = (luma >= 8'd128) ? 24'hFFFFFF : 24'h000000;
assign sepia   = sepia_rgb(r, g, b);
assign contrast = {contrast_ch(r), contrast_ch(g), contrast_ch(b)};
assign saturate = {satu_ch(r, luma), satu_ch(g, luma), satu_ch(b, luma)};
assign mute     = {mute_ch(r, luma), mute_ch(g, luma), mute_ch(b, luma)};
// AMBER keeps red and drops blue to a quarter; COOL is the opposite cast and
// drops red instead. Their green gains differ (5/8 warm, 3/8 cool) so the two
// read as distinct looks rather than as one effect mirrored.
//
// The obvious shift-add (g>>1)+(g>>3) is WRONG for 5/8: the two floors
// compound, and at g=5 it gives 2 where floor(25/8) is 3. Forming 5g and 3g
// first and then taking a bit-select is exact, is still one adder each with no
// multiplier, and the >>3 is free wiring. Every gain here is <= 1, so neither
// cast needs a clamp.
assign g_mul5 = {g, 2'b00} + {3'b000, g};    // 5g, peak 1275, fits 11 bits
assign g_mul3 = {g, 1'b0}  + {2'b00,  g};    // 3g, peak  765, fits 10 bits
assign amber  = {r,      g_mul5[10:3], b >> 2};
assign cool   = {r >> 2, g_mul3[9:3],  b};
assign solar  = {solar_ch(r), solar_ch(g), solar_ch(b)};
// Posterize is a pure bit truncate and GINV reuses the luma already computed
// for GRAY, so neither costs an adder.
assign ginv     = {~luma, ~luma, ~luma};

assign O_rgb = (I_sel == 4'd1)  ? {luma, luma, luma} :
               (I_sel == 4'd2)  ? ~I_rgb :
               (I_sel == 4'd3)  ? thresh :
               (I_sel == 4'd4)  ? sepia :
               (I_sel == 4'd5)  ? contrast :
               (I_sel == 4'd6)  ? saturate :
               (I_sel == 4'd7)  ? mute :
               (I_sel == 4'd8)  ? amber :
               (I_sel == 4'd9)  ? cool :
               (I_sel == 4'd10) ? solar :
               (I_sel == 4'd11) ? {r[7:6], 6'b0, g[7:6], 6'b0, b[7:6], 6'b0} :
               (I_sel == 4'd12) ? ginv :
                                  I_rgb;

// 77/150/29 sum to exactly 256, so the >>8 needs no clamp; peak 65280 fits 16 bits.
function [7:0] gray_y;
    input [7:0] r;
    input [7:0] g;
    input [7:0] b;
    reg  [15:0] pr;
    reg  [15:0] pg;
    reg  [15:0] pb;
    reg  [15:0] sum;
    begin
        pr = ({2'b00, r, 6'b0} + {5'b0, r, 3'b0}) + ({6'b0, r, 2'b0} + {8'b0, r});
        pg = ({1'b0, g, 7'b0} + {4'b0, g, 4'b0}) + ({6'b0, g, 2'b0} + {7'b0, g, 1'b0});
        pb = ({4'b0, b, 4'b0} + {5'b0, b, 3'b0}) + ({6'b0, b, 2'b0} + {8'b0, b});
        sum = (pr + pg) + pb;
        gray_y = sum[15:8];
    end
endfunction

// Classic sepia rows are proportional, so one weighted sum drives all three channels.
// T = 2R+4G+B factors out 12: S = 12T + R + G = 25R + 49G + 12B, peak 21930 (15 bits).
// The channel gains must come off the UNCLAMPED sw: the textbook matrix clamps each
// row on its own, so deriving them from a clamped s8 diverges by up to 66 levels on
// bright pixels. Gains 0.890625 / 0.6953125 hold the full-gamut error to 2/3/5 levels.
function [23:0] sepia_rgb;
    input [7:0] r;
    input [7:0] g;
    input [7:0] b;
    reg  [10:0] t;
    reg  [8:0]  rg;
    reg  [14:0] s;
    reg  [8:0]  sw;
    reg  [8:0]  gp;
    reg  [8:0]  bp;
    reg  [7:0]  s8;
    reg  [7:0]  g8;
    reg  [7:0]  b8;
    begin
        t  = ({2'b00, r, 1'b0} + {1'b0, g, 2'b00}) + {3'b000, b};
        rg = {1'b0, r} + {1'b0, g};
        s  = ({1'b0, t, 3'b000} + {2'b00, t, 2'b00}) + {6'b0, rg};
        sw = {1'b0, s[14:6]};
        gp = (sw - {3'b000, sw[8:3]}) + {6'b000000, sw[8:6]};
        bp = ({1'b0, sw[8:1]} + {3'b000, sw[8:3]}) + ({4'b0000, sw[8:4]} + {7'b0000000, sw[8:7]});
        s8 = (sw > 9'd255) ? 8'd255 : sw[7:0];
        g8 = (gp > 9'd255) ? 8'd255 : gp[7:0];
        b8 = bp[7:0];
        sepia_rgb = {s8, g8, b8};
    end
endfunction

// Gain 1.5 around the 128 pivot, unsigned on both sides so 128 is an exact fixed point.
function [7:0] contrast_ch;
    input [7:0] ch;
    reg   [7:0] half;
    reg   [8:0] sum;
    reg   [8:0] dif;
    begin
        if (ch >= 8'd128) begin
            half = (ch - 8'd128) >> 1;
            sum  = {1'b0, ch} + {1'b0, half};
            contrast_ch = (sum > 9'd255) ? 8'd255 : sum[7:0];
        end else begin
            half = (8'd128 - ch) >> 1;
            dif  = {1'b0, ch} - {1'b0, half};
            contrast_ch = dif[8] ? 8'd0 : dif[7:0];
        end
    end
endfunction

// Saturation boost, y + 2*(ch - y) == 2*ch - y. This is the only one of the
// seven new filters whose result can leave 0..255, so it is split on ch >= y
// and clamped on both sides instead of going through a signed path. The
// negative side needs just dn[9]: 2*ch - y lies in [-255, 255], and in 10 bit
// two's complement every value there is negative exactly when bit 9 is set,
// while every non-negative one fits in 8 bits.
function [7:0] satu_ch;
    input [7:0] ch;
    input [7:0] y;
    reg   [8:0] dup;
    reg   [8:0] ddn;
    reg   [9:0] up;
    reg   [9:0] dn;
    begin
        if (ch >= y) begin
            dup = {1'b0, ch} - {1'b0, y};
            up  = {2'b00, y} + {dup, 1'b0};
            satu_ch = (up > 10'd255) ? 8'd255 : up[7:0];
        end else begin
            ddn = {1'b0, y} - {1'b0, ch};
            dn  = {2'b00, y} - {ddn, 1'b0};
            satu_ch = dn[9] ? 8'd0 : dn[7:0];
        end
    end
endfunction

// Saturation halved, y + (ch - y)/2 floored, which is identically (ch + y) >> 1.
// Written that way it needs one 9 bit add and no clamp: ch + y <= 510.
function [7:0] mute_ch;
    input [7:0] ch;
    input [7:0] y;
    reg   [8:0] sum;
    begin
        sum = {1'b0, ch} + {1'b0, y};
        mute_ch = sum[8:1];
    end
endfunction

// Solarize: the dark half passes through and the bright half is inverted, so
// the transfer curve is a triangle peaking at 127 (128 itself folds back to
// 127). Nothing can exceed 127, hence no clamp.
function [7:0] solar_ch;
    input [7:0] ch;
    begin
        solar_ch = (ch < 8'd128) ? ch : ~ch;
    end
endfunction

endmodule
