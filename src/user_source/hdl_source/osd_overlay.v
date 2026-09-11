module osd_overlay #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_display_valid,
    input  wire [1:0]  I_image_index,
    input  wire        I_auto_play,
    input  wire [2:0]  I_brightness,
    input  wire [3:0]  I_state_code,
    input  wire [3:0]  I_filt,
    input  wire        I_font,
    input  wire        I_asrc,             // audio source select: 0 built-in test tone, 1 TF card WAV
    output wire [23:0] O_rgb
);

localparam PANEL_X    = 14;
localparam PANEL_Y    = 14;
localparam PANEL_W    = 304;
localparam PANEL_H    = 122;
localparam TEXT_X     = 24;
localparam TEXT_Y     = 30;
localparam TEXT_COLS  = 16;
localparam CHAR_SCALE = 2;
localparam CHAR_W_S   = 8 * CHAR_SCALE;
localparam CHAR_H_S   = 8 * CHAR_SCALE;
localparam TEXT_W     = TEXT_COLS * CHAR_W_S;
localparam TEXT_H     = 5 * CHAR_H_S;
localparam BAR_X      = 28;
localparam BAR_Y      = 114;
localparam BAR_W      = 120;
localparam BAR_H      = 8;

localparam [9:0] H_ACTIVE_W   = H_ACTIVE;
localparam [9:0] V_ACTIVE_W   = V_ACTIVE;
localparam [9:0] PANEL_X_W    = PANEL_X;
localparam [9:0] PANEL_Y_W    = PANEL_Y;
localparam [9:0] PANEL_X_END  = PANEL_X + PANEL_W;
localparam [9:0] PANEL_Y_END  = PANEL_Y + PANEL_H;
localparam [9:0] PANEL_X_LAST = PANEL_X + PANEL_W - 1;
localparam [9:0] PANEL_Y_LAST = PANEL_Y + PANEL_H - 1;
localparam [9:0] TEXT_X_W     = TEXT_X;
localparam [9:0] TEXT_Y_W     = TEXT_Y;
localparam [9:0] TEXT_X_END   = TEXT_X + TEXT_W;
localparam [9:0] TEXT_Y_END   = TEXT_Y + TEXT_H;
localparam [9:0] BAR_X_W      = BAR_X;
localparam [9:0] BAR_Y_W      = BAR_Y;
localparam [9:0] BAR_X_END    = BAR_X + BAR_W;
localparam [9:0] BAR_Y_END    = BAR_Y + BAR_H;

reg [9:0] x_pos;
reg [9:0] y_pos;
reg       de_d;
reg [5:0] frame_phase;

wire in_active;
wire in_panel;
wire in_text;
wire in_title_strip;
wire in_brightness_bar;
wire in_brightness_fill;
wire live_dot;
wire panel_border;
wire [9:0] text_rel_x;
wire [9:0] text_rel_y;
wire [2:0] line_idx;
wire [3:0] char_idx;
wire [2:0] font_col;
wire [2:0] font_row;
wire [7:0] char_code;
wire [7:0] font_bits;
wire text_pixel;
wire [7:0] bg_r;
wire [7:0] bg_g;
wire [7:0] bg_b;
wire [23:0] panel_rgb;
wire [23:0] status_rgb;
wire [9:0] brightness_end;

assign in_active = I_de && (x_pos < H_ACTIVE_W) && (y_pos < V_ACTIVE_W);
assign in_panel = in_active &&
                  (x_pos >= PANEL_X_W) && (x_pos < PANEL_X_END) &&
                  (y_pos >= PANEL_Y_W) && (y_pos < PANEL_Y_END);
assign in_title_strip = in_panel && (y_pos < PANEL_Y_W + 10'd8);
assign panel_border = in_panel &&
                      ((x_pos == PANEL_X_W) || (x_pos == PANEL_X_LAST) ||
                       (y_pos == PANEL_Y_W) || (y_pos == PANEL_Y_LAST));

assign in_text = in_active &&
                 (x_pos >= TEXT_X_W) && (x_pos < TEXT_X_END) &&
                 (y_pos >= TEXT_Y_W) && (y_pos < TEXT_Y_END);
assign text_rel_x = x_pos - TEXT_X_W;
assign text_rel_y = y_pos - TEXT_Y_W;
assign line_idx = text_rel_y[6:4];
assign char_idx = text_rel_x[7:4];
assign font_col = text_rel_x[3:1];
assign font_row = text_rel_y[3:1];
assign char_code = osd_char(line_idx, char_idx, I_display_valid,
                            I_image_index, I_auto_play, I_brightness,
                            I_state_code, I_filt, I_font, I_asrc);
assign font_bits = font8x8(char_code, font_row);
assign text_pixel = in_text && font_bits[3'd7 - font_col];

assign brightness_end = BAR_X_W + 10'd24 + {3'b000, I_brightness, 4'b0000};
assign in_brightness_bar = in_active &&
                           (x_pos >= BAR_X_W) && (x_pos < BAR_X_END) &&
                           (y_pos >= BAR_Y_W) && (y_pos < BAR_Y_END);
assign in_brightness_fill = in_brightness_bar && (x_pos < brightness_end);
assign live_dot = in_panel &&
                  (x_pos >= PANEL_X_W + 10'd274) && (x_pos < PANEL_X_W + 10'd288) &&
                  (y_pos >= PANEL_Y_W + 10'd18)  && (y_pos < PANEL_Y_W + 10'd32);

assign bg_r = {2'b00, I_rgb[23:18]} + {3'b000, I_rgb[23:19]};
assign bg_g = {2'b00, I_rgb[15:10]} + {3'b000, I_rgb[15:11]};
assign bg_b = {2'b00, I_rgb[7:2]}   + {3'b000, I_rgb[7:3]};
assign panel_rgb = {bg_r, bg_g, bg_b};
assign status_rgb = I_auto_play ? 24'h40FF80 : 24'h60B0FF;

assign O_rgb = text_pixel         ? 24'hFFE878 :
               live_dot           ? (frame_phase[4] ? status_rgb : 24'h204838) :
               in_brightness_fill ? 24'hFFD040 :
               in_brightness_bar  ? 24'h384858 :
               panel_border       ? 24'h60D8FF :
               in_title_strip     ? 24'h3068D0 :
               in_panel           ? panel_rgb :
                                    I_rgb;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x_pos       <= 10'd0;
        y_pos       <= 10'd0;
        de_d        <= 1'b0;
        frame_phase <= 6'd0;
    end else begin
        de_d <= I_de;

        if (I_de) begin
            if (!de_d) begin
                x_pos <= 10'd1;
            end else if (x_pos == H_ACTIVE_W - 10'd1) begin
                x_pos <= 10'd0;
            end else begin
                x_pos <= x_pos + 10'd1;
            end
        end else begin
            x_pos <= 10'd0;

            if (de_d) begin
                if (y_pos == V_ACTIVE_W - 10'd1) begin
                    y_pos <= 10'd0;
                    frame_phase <= frame_phase + 6'd1;
                end else begin
                    y_pos <= y_pos + 10'd1;
                end
            end
        end
    end
end

function [7:0] osd_char;
    input [2:0] line;
    input [3:0] idx;
    input       display_valid;
    input [1:0] image_index;
    input       auto_play;
    input [2:0] brightness;
    input [3:0] state_code;
    input [3:0] filt;
    input       font;
    input       asrc;
    reg  [39:0] name40;
    begin
        case (line)
            3'd0: begin
                case (idx)
                    4'd0: osd_char = "A";
                    4'd1: osd_char = "N";
                    4'd2: osd_char = "L";
                    4'd3: osd_char = "O";
                    4'd4: osd_char = "G";
                    4'd5: osd_char = "I";
                    4'd6: osd_char = "C";
                    4'd7: osd_char = " ";
                    4'd8: osd_char = "M";
                    4'd9: osd_char = "E";
                    4'd10: osd_char = "D";
                    4'd11: osd_char = "I";
                    4'd12: osd_char = "A";
                    4'd13: osd_char = " ";
                    4'd14: osd_char = "2";
                    4'd15: osd_char = "6";
                    default: osd_char = " ";
                endcase
            end
            3'd1: begin
                if (display_valid) begin
                    case (idx)
                        4'd0: osd_char = "I";
                        4'd1: osd_char = "M";
                        4'd2: osd_char = "G";
                        4'd3: osd_char = ":";
                        4'd4: osd_char = hex_char({2'b00, image_index} + 4'd1);
                        4'd5: osd_char = " ";
                        4'd6: osd_char = " ";
                        4'd7: osd_char = "M";
                        4'd8: osd_char = "O";
                        4'd9: osd_char = "D";
                        4'd10: osd_char = "E";
                        4'd11: osd_char = " ";
                        4'd12: osd_char = auto_play ? "A" : "M";
                        4'd13: osd_char = auto_play ? "U" : "A";
                        4'd14: osd_char = auto_play ? "T" : "N";
                        4'd15: osd_char = auto_play ? "O" : "U";
                        default: osd_char = " ";
                    endcase
                end else begin
                    case (idx)
                        4'd0: osd_char = "L";
                        4'd1: osd_char = "O";
                        4'd2: osd_char = "A";
                        4'd3: osd_char = "D";
                        4'd4: osd_char = " ";
                        4'd5: osd_char = " ";
                        4'd6: osd_char = "T";
                        4'd7: osd_char = "F";
                        4'd8: osd_char = " ";
                        4'd9: osd_char = "C";
                        4'd10: osd_char = "A";
                        4'd11: osd_char = "R";
                        4'd12: osd_char = "D";
                        default: osd_char = " ";
                    endcase
                end
            end
            3'd2: begin
                case (idx)
                    4'd0: osd_char = "B";
                    4'd1: osd_char = "R";
                    4'd2: osd_char = "I";
                    4'd3: osd_char = "G";
                    4'd4: osd_char = "H";
                    4'd5: osd_char = "T";
                    4'd6: osd_char = " ";
                    4'd7: osd_char = "B";
                    4'd8: osd_char = hex_char({1'b0, brightness});
                    4'd9: osd_char = " ";
                    4'd10: osd_char = " ";
                    4'd11: osd_char = "S";
                    4'd12: osd_char = ":";
                    4'd13: osd_char = hex_char(state_code);
                    default: osd_char = " ";
                endcase
            end
            3'd3: begin
                // "HDMI AUDIO " + the live source. Exactly 16 characters, which
                // is the whole line: char_idx is text_rel_x[7:4], so there is no
                // room for a 17th. The old trailing "FPGA" tag gives up its four
                // characters -- the board is still identified on line 0, and
                // which of the two mutually exclusive audio sources is actually
                // on the air is the far more useful thing to read off a panel
                // that has no other way to show it.
                case (idx)
                    4'd0: osd_char = "H";
                    4'd1: osd_char = "D";
                    4'd2: osd_char = "M";
                    4'd3: osd_char = "I";
                    4'd4: osd_char = " ";
                    4'd5: osd_char = "A";
                    4'd6: osd_char = "U";
                    4'd7: osd_char = "D";
                    4'd8: osd_char = "I";
                    4'd9: osd_char = "O";
                    4'd10: osd_char = " ";
                    4'd11: osd_char = asrc ? "M" : "T";
                    4'd12: osd_char = asrc ? "U" : "O";
                    4'd13: osd_char = asrc ? "S" : "N";
                    4'd14: osd_char = asrc ? "I" : "E";
                    default: osd_char = asrc ? "C" : " ";
                endcase
            end
            3'd4: begin
                // Names use only glyphs font8x8 already has. The available set is
                // ": 0-9 A B C D E F G H I L M N O P R S T U V X Y" -- there is
                // no J, K, Q, W or Z, which is why the passthrough filter is
                // called "OFF" rather than "RAW" and the half-way blend toward
                // luma is "MUTE" rather than "WASH". Codes D/E/F are reserved and
                // fall through to "OFF  ", which is what they actually do:
                // video_effect passes I_rgb straight through for anything it does
                // not recognise.
                case (filt)
                    4'd0: name40 = "OFF  ";
                    4'd1: name40 = "GRAY ";
                    4'd2: name40 = "INVT ";
                    4'd3: name40 = "THRS ";
                    4'd4: name40 = "SEPIA";
                    4'd5: name40 = "CNTR ";
                    4'd6: name40 = "SATU ";
                    4'd7: name40 = "MUTE ";
                    4'd8: name40 = "AMBER";
                    4'd9: name40 = "COOL ";
                    4'd10: name40 = "SOLAR";
                    4'd11: name40 = "POSTE";
                    4'd12: name40 = "GINV ";
                    default: name40 = "OFF  ";
                endcase
                case (idx)
                    4'd0: osd_char = "F";
                    4'd1: osd_char = "X";
                    4'd2: osd_char = ":";
                    4'd3: osd_char = name40[39:32];
                    4'd4: osd_char = name40[31:24];
                    4'd5: osd_char = name40[23:16];
                    4'd6: osd_char = name40[15:8];
                    4'd7: osd_char = name40[7:0];
                    4'd8: osd_char = " ";
                    4'd9: osd_char = "F";
                    4'd10: osd_char = "O";
                    4'd11: osd_char = "N";
                    4'd12: osd_char = "T";
                    4'd13: osd_char = ":";
                    4'd14: osd_char = font ? "3" : "2";
                    4'd15: osd_char = "D";
                endcase
            end
            default: osd_char = " ";
        endcase
    end
endfunction

function [7:0] hex_char;
    input [3:0] value;
    begin
        case (value)
            4'h0: hex_char = "0";
            4'h1: hex_char = "1";
            4'h2: hex_char = "2";
            4'h3: hex_char = "3";
            4'h4: hex_char = "4";
            4'h5: hex_char = "5";
            4'h6: hex_char = "6";
            4'h7: hex_char = "7";
            4'h8: hex_char = "8";
            4'h9: hex_char = "9";
            4'hA: hex_char = "A";
            4'hB: hex_char = "B";
            4'hC: hex_char = "C";
            4'hD: hex_char = "D";
            4'hE: hex_char = "E";
            default: hex_char = "F";
        endcase
    end
endfunction

function [7:0] font8x8;
    input [7:0] code;
    input [2:0] row;
    begin
        case (code)
            ":": case (row)
                3'd1: font8x8 = 8'b00011000;
                3'd2: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00011000;
                3'd5: font8x8 = 8'b00011000;
                default: font8x8 = 8'b00000000;
            endcase
            "0": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01101110;
                3'd3: font8x8 = 8'b01110110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "1": case (row)
                3'd0: font8x8 = 8'b00011000;
                3'd1: font8x8 = 8'b00111000;
                3'd2: font8x8 = 8'b00011000;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00011000;
                3'd5: font8x8 = 8'b00011000;
                3'd6: font8x8 = 8'b01111110;
                default: font8x8 = 8'b00000000;
            endcase
            "2": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b00000110;
                3'd3: font8x8 = 8'b00001100;
                3'd4: font8x8 = 8'b00110000;
                3'd5: font8x8 = 8'b01100000;
                3'd6: font8x8 = 8'b01111110;
                default: font8x8 = 8'b00000000;
            endcase
            "3": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b00000110;
                3'd3: font8x8 = 8'b00011100;
                3'd4: font8x8 = 8'b00000110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "4": case (row)
                3'd0: font8x8 = 8'b00001100;
                3'd1: font8x8 = 8'b00011100;
                3'd2: font8x8 = 8'b00101100;
                3'd3: font8x8 = 8'b01001100;
                3'd4: font8x8 = 8'b01111110;
                3'd5: font8x8 = 8'b00001100;
                3'd6: font8x8 = 8'b00001100;
                default: font8x8 = 8'b00000000;
            endcase
            "5": case (row)
                3'd0: font8x8 = 8'b01111110;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01111100;
                3'd3: font8x8 = 8'b00000110;
                3'd4: font8x8 = 8'b00000110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "6": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01111100;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "7": case (row)
                3'd0: font8x8 = 8'b01111110;
                3'd1: font8x8 = 8'b00000110;
                3'd2: font8x8 = 8'b00001100;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00110000;
                3'd5: font8x8 = 8'b00110000;
                3'd6: font8x8 = 8'b00110000;
                default: font8x8 = 8'b00000000;
            endcase
            "8": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b00111100;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "9": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b00111110;
                3'd4: font8x8 = 8'b00000110;
                3'd5: font8x8 = 8'b00001100;
                3'd6: font8x8 = 8'b00111000;
                default: font8x8 = 8'b00000000;
            endcase
            "A": case (row)
                3'd0: font8x8 = 8'b00011000;
                3'd1: font8x8 = 8'b00111100;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01111110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b01100110;
                default: font8x8 = 8'b00000000;
            endcase
            "B": case (row)
                3'd0: font8x8 = 8'b01111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01111100;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b01111100;
                default: font8x8 = 8'b00000000;
            endcase
            "C": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b01100000;
                3'd4: font8x8 = 8'b01100000;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "D": case (row)
                3'd0: font8x8 = 8'b01111000;
                3'd1: font8x8 = 8'b01101100;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01101100;
                3'd6: font8x8 = 8'b01111000;
                default: font8x8 = 8'b00000000;
            endcase
            "E": case (row)
                3'd0: font8x8 = 8'b01111110;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b01111100;
                3'd4: font8x8 = 8'b01100000;
                3'd5: font8x8 = 8'b01100000;
                3'd6: font8x8 = 8'b01111110;
                default: font8x8 = 8'b00000000;
            endcase
            "F": case (row)
                3'd0: font8x8 = 8'b01111110;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b01111100;
                3'd4: font8x8 = 8'b01100000;
                3'd5: font8x8 = 8'b01100000;
                3'd6: font8x8 = 8'b01100000;
                default: font8x8 = 8'b00000000;
            endcase
            "G": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b01101110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "H": case (row)
                3'd0: font8x8 = 8'b01100110;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01111110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b01100110;
                default: font8x8 = 8'b00000000;
            endcase
            "I": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b00011000;
                3'd2: font8x8 = 8'b00011000;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00011000;
                3'd5: font8x8 = 8'b00011000;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "L": case (row)
                3'd0: font8x8 = 8'b01100000;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b01100000;
                3'd4: font8x8 = 8'b01100000;
                3'd5: font8x8 = 8'b01100000;
                3'd6: font8x8 = 8'b01111110;
                default: font8x8 = 8'b00000000;
            endcase
            "M": case (row)
                3'd0: font8x8 = 8'b01100011;
                3'd1: font8x8 = 8'b01110111;
                3'd2: font8x8 = 8'b01111111;
                3'd3: font8x8 = 8'b01101011;
                3'd4: font8x8 = 8'b01100011;
                3'd5: font8x8 = 8'b01100011;
                3'd6: font8x8 = 8'b01100011;
                default: font8x8 = 8'b00000000;
            endcase
            "N": case (row)
                3'd0: font8x8 = 8'b01100011;
                3'd1: font8x8 = 8'b01110011;
                3'd2: font8x8 = 8'b01111011;
                3'd3: font8x8 = 8'b01101111;
                3'd4: font8x8 = 8'b01100111;
                3'd5: font8x8 = 8'b01100011;
                3'd6: font8x8 = 8'b01100011;
                default: font8x8 = 8'b00000000;
            endcase
            "O": case (row)
                3'd0: font8x8 = 8'b00111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "P": case (row)
                3'd0: font8x8 = 8'b01111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01111100;
                3'd4: font8x8 = 8'b01100000;
                3'd5: font8x8 = 8'b01100000;
                3'd6: font8x8 = 8'b01100000;
                default: font8x8 = 8'b00000000;
            endcase
            "R": case (row)
                3'd0: font8x8 = 8'b01111100;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01111100;
                3'd4: font8x8 = 8'b01101100;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b01100011;
                default: font8x8 = 8'b00000000;
            endcase
            "S": case (row)
                3'd0: font8x8 = 8'b00111110;
                3'd1: font8x8 = 8'b01100000;
                3'd2: font8x8 = 8'b01100000;
                3'd3: font8x8 = 8'b00111100;
                3'd4: font8x8 = 8'b00000110;
                3'd5: font8x8 = 8'b00000110;
                3'd6: font8x8 = 8'b01111100;
                default: font8x8 = 8'b00000000;
            endcase
            "T": case (row)
                3'd0: font8x8 = 8'b01111110;
                3'd1: font8x8 = 8'b00011000;
                3'd2: font8x8 = 8'b00011000;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00011000;
                3'd5: font8x8 = 8'b00011000;
                3'd6: font8x8 = 8'b00011000;
                default: font8x8 = 8'b00000000;
            endcase
            "U": case (row)
                3'd0: font8x8 = 8'b01100110;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b00111100;
                default: font8x8 = 8'b00000000;
            endcase
            "V": case (row)
                3'd0: font8x8 = 8'b01100110;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b01100110;
                3'd3: font8x8 = 8'b01100110;
                3'd4: font8x8 = 8'b01100110;
                3'd5: font8x8 = 8'b00111100;
                3'd6: font8x8 = 8'b00011000;
                default: font8x8 = 8'b00000000;
            endcase
            "X": case (row)
                3'd0: font8x8 = 8'b01100110;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b00111100;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00111100;
                3'd5: font8x8 = 8'b01100110;
                3'd6: font8x8 = 8'b01100110;
                default: font8x8 = 8'b00000000;
            endcase
            "Y": case (row)
                3'd0: font8x8 = 8'b01100110;
                3'd1: font8x8 = 8'b01100110;
                3'd2: font8x8 = 8'b00111100;
                3'd3: font8x8 = 8'b00011000;
                3'd4: font8x8 = 8'b00011000;
                3'd5: font8x8 = 8'b00011000;
                3'd6: font8x8 = 8'b00011000;
                default: font8x8 = 8'b00000000;
            endcase
            default: font8x8 = 8'b00000000;
        endcase
    end
endfunction

endmodule
