// Scrolling slogan banner across the vertical middle of the picture.
//
// Sits at the very end of the video chain, after osd_overlay, so it can dim
// whatever is underneath it uniformly. The glyph bitmaps live in the generated
// marquee_font.vh (tools/gen_marquee_font.py is the golden reference); this
// file only holds the raster tracker, the scroll counter and the addressing.
//
// Geometry: 15 glyph cells of 24 px on a 32 px pitch, so cell and column are
// bit slices of u instead of a division. u = x_pos + marq_pos - H_ACTIVE taken
// on an 11 bit wire; the single unsigned compare u < TEXT_W covers both the
// "text has not entered yet" and "text has fully left" cases because the
// borrow wraps the negative side above TEXT_W.
module marquee_overlay #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_en,
    output wire [23:0] O_rgb
);

localparam CELL             = 24;
localparam PITCH            = 32;
localparam GUTTER           = (PITCH - CELL) / 2;
localparam N_CELLS          = 15;
localparam TEXT_W           = N_CELLS * PITCH;
localparam TRAVEL           = H_ACTIVE + TEXT_W;
localparam BAND_H           = 32;
localparam BAND_Y           = (V_ACTIVE - BAND_H) / 2;
localparam BAND_Y_LAST      = BAND_Y + BAND_H - 1;
localparam BAND_TEXT_Y      = BAND_Y + 4;
localparam BAND_TEXT_Y_LAST = BAND_TEXT_Y + CELL - 1;
localparam ROW_LSB          = BAND_TEXT_Y % PITCH;
localparam SCROLL_FRAME_DIV = 2;
localparam DIM_SHIFT        = 2;

localparam [9:0]  H_ACTIVE_W         = H_ACTIVE;
localparam [9:0]  V_ACTIVE_W         = V_ACTIVE;
localparam [9:0]  BAND_Y_W           = BAND_Y;
localparam [9:0]  BAND_Y_LAST_W      = BAND_Y_LAST;
localparam [9:0]  BAND_TEXT_Y_W      = BAND_TEXT_Y;
localparam [9:0]  BAND_TEXT_Y_LAST_W = BAND_TEXT_Y_LAST;
localparam [10:0] H_ACTIVE_P         = H_ACTIVE;
localparam [10:0] TEXT_W_P           = TEXT_W;
localparam [10:0] TRAVEL_LAST_P      = TRAVEL - 1;
localparam [4:0]  GUTTER_W           = GUTTER;
localparam [4:0]  GUTTER_LAST_W      = GUTTER + CELL - 1;
localparam [4:0]  ROW_LSB_W          = ROW_LSB;
localparam [5:0]  FRAME_DIV_LAST     = SCROLL_FRAME_DIV - 1;

reg [9:0]  x_pos;
reg [9:0]  y_pos;
reg        de_d;
reg [10:0] marq_pos;
reg [5:0]  frame_div;

wire        in_band;
wire        band_edge;
wire        in_text_rows;
wire        frame_wrap;
wire [10:0] s;
wire [10:0] u;
wire        in_region;
// `cell` is a Verilog-2001 reserved word, hence the suffix.
wire [4:0]  cell_idx;
wire [4:0]  col;
wire        col_in_glyph;
wire [4:0]  gcol;
wire [4:0]  row;
wire [23:0] glyph_bits;
wire        text_on;
wire [7:0]  dim_r;
wire [7:0]  dim_g;
wire [7:0]  dim_b;

assign in_band = I_en && I_de &&
                 (y_pos >= BAND_Y_W) && (y_pos <= BAND_Y_LAST_W);
assign band_edge = (y_pos == BAND_Y_W) || (y_pos == BAND_Y_LAST_W);
assign in_text_rows = (y_pos >= BAND_TEXT_Y_W) && (y_pos <= BAND_TEXT_Y_LAST_W);
assign frame_wrap = !I_de && de_d && (y_pos == V_ACTIVE_W - 10'd1);

assign s = {1'b0, x_pos} + marq_pos;
assign u = s - H_ACTIVE_P;
assign in_region = (u < TEXT_W_P);
assign cell_idx = u[9:5];
assign col      = u[4:0];
assign col_in_glyph = (col >= GUTTER_W) && (col <= GUTTER_LAST_W);
assign gcol = col - GUTTER_W;
assign row  = y_pos[4:0] - ROW_LSB_W;

assign glyph_bits = marquee_glyph(cell_idx[3:0], row);
assign text_on = in_text_rows && in_region && col_in_glyph &&
                 glyph_bits[5'd23 - gcol];

assign dim_r = I_rgb[23:16] >> DIM_SHIFT;
assign dim_g = I_rgb[15:8]  >> DIM_SHIFT;
assign dim_b = I_rgb[7:0]   >> DIM_SHIFT;

assign O_rgb = !in_band     ? I_rgb :
               text_on      ? 24'hFFE878 :
               band_edge    ? 24'h60D8FF :
                              {dim_r, dim_g, dim_b};

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x_pos     <= 10'd0;
        y_pos     <= 10'd0;
        de_d      <= 1'b0;
        marq_pos  <= 11'd0;
        frame_div <= 6'd0;
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

                    if (frame_div == FRAME_DIV_LAST) begin
                        frame_div <= 6'd0;
                        if (marq_pos == TRAVEL_LAST_P) begin
                            marq_pos <= 11'd0;
                        end else begin
                            marq_pos <= marq_pos + 11'd1;
                        end
                    end else begin
                        frame_div <= frame_div + 6'd1;
                    end
                end else begin
                    y_pos <= y_pos + 10'd1;
                end
            end
        end
    end
end

`include "marquee_font.vh"

endmodule
