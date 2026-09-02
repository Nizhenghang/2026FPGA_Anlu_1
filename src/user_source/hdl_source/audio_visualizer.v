module audio_visualizer #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_de,
    input  wire        I_frame_start,
    input  wire [23:0] I_rgb,
    input  wire        I_audio_valid,
    input  wire [23:0] I_audio_left,
    input  wire [23:0] I_audio_right,
    output wire [23:0] O_rgb
);

localparam PANEL_X      = 28;
localparam PANEL_Y      = 352;
localparam PANEL_W      = 584;
localparam PANEL_H      = 104;
localparam WAVE_X       = 44;
localparam WAVE_Y       = 362;
localparam WAVE_W       = 256;
localparam WAVE_H       = 80;
localparam BAR_X        = 328;
localparam BAR_Y        = 370;
localparam BAR_W        = 256;
localparam BAR_H        = 64;
localparam WAVE_CENTER  = WAVE_Y + (WAVE_H / 2);
localparam WAVE_COUNT   = 64;

localparam [9:0] H_ACTIVE_W    = H_ACTIVE;
localparam [9:0] V_ACTIVE_W    = V_ACTIVE;
localparam [9:0] PANEL_X_W     = PANEL_X;
localparam [9:0] PANEL_Y_W     = PANEL_Y;
localparam [9:0] PANEL_X_END   = PANEL_X + PANEL_W;
localparam [9:0] PANEL_Y_END   = PANEL_Y + PANEL_H;
localparam [9:0] PANEL_X_LAST  = PANEL_X + PANEL_W - 1;
localparam [9:0] PANEL_Y_LAST  = PANEL_Y + PANEL_H - 1;
localparam [9:0] WAVE_X_W      = WAVE_X;
localparam [9:0] WAVE_Y_W      = WAVE_Y;
localparam [9:0] WAVE_X_END    = WAVE_X + WAVE_W;
localparam [9:0] WAVE_Y_END    = WAVE_Y + WAVE_H;
localparam [9:0] WAVE_CENTER_W = WAVE_CENTER;
localparam [9:0] BAR_X_W       = BAR_X;
localparam [9:0] BAR_Y_W       = BAR_Y;
localparam [9:0] BAR_X_END     = BAR_X + BAR_W;
localparam [9:0] BAR_Y_END     = BAR_Y + BAR_H;

reg [9:0] x_pos;
reg [9:0] y_pos;
reg       de_d;
reg [5:0] write_idx;
reg [2:0] sample_div;
reg signed [5:0] wave_mem [0:WAVE_COUNT-1];

reg signed [5:0] last_sample;
reg [11:0]       zero_distance;
reg [11:0]       period_est;
reg [5:0]        energy_hold;
reg [5:0]        peak_hold;
reg [5:0]        bin_level0;
reg [5:0]        bin_level1;
reg [5:0]        bin_level2;
reg [5:0]        bin_level3;
reg [5:0]        bin_level4;
reg [5:0]        bin_level5;
reg [5:0]        bin_level6;
reg [5:0]        bin_level7;

integer i;

wire in_active;
wire in_panel;
wire in_wave;
wire in_bars;
wire panel_border;
wire grid_line;
wire center_line;
wire [9:0] wave_rel_x;
wire [9:0] bar_rel_x;
wire [9:0] bar_rel_y;
wire [5:0] col_idx;
wire [5:0] rd_idx;
wire [2:0] bar_idx;
wire signed [5:0] wave_sample;
wire signed [6:0] left_sample;
wire signed [6:0] right_sample;
wire signed [6:0] mixed_sample;
wire signed [5:0] audio_sample;
wire [5:0] audio_abs;
wire [5:0] bin_level;
wire [5:0] peak_level;
wire [9:0] wave_y;
wire [9:0] bar_height;
wire [9:0] peak_y;
wire wave_pixel;
wire bar_pixel;
wire peak_pixel;
wire sparkle_pixel;
wire [7:0] bg_r;
wire [7:0] bg_g;
wire [7:0] bg_b;
wire [23:0] panel_rgb;
wire [23:0] bar_rgb;
wire [23:0] stage_rgb;

assign in_active = I_de && (x_pos < H_ACTIVE_W) && (y_pos < V_ACTIVE_W);
assign in_panel = in_active &&
                  (x_pos >= PANEL_X_W) && (x_pos < PANEL_X_END) &&
                  (y_pos >= PANEL_Y_W) && (y_pos < PANEL_Y_END);
assign in_wave = in_active &&
                 (x_pos >= WAVE_X_W) && (x_pos < WAVE_X_END) &&
                 (y_pos >= WAVE_Y_W) && (y_pos < WAVE_Y_END);
assign in_bars = in_active &&
                 (x_pos >= BAR_X_W) && (x_pos < BAR_X_END) &&
                 (y_pos >= BAR_Y_W) && (y_pos < BAR_Y_END);

assign panel_border = in_panel &&
                      ((x_pos == PANEL_X_W) || (x_pos == PANEL_X_LAST) ||
                       (y_pos == PANEL_Y_W) || (y_pos == PANEL_Y_LAST));
assign center_line = in_wave && (y_pos == WAVE_CENTER_W);
assign grid_line = in_panel && ((x_pos[5:0] == 6'd0) || (y_pos[4:0] == 5'd0));

assign wave_rel_x = x_pos - WAVE_X_W;
assign col_idx = wave_rel_x[8:3];
assign rd_idx = write_idx + col_idx;
assign wave_sample = wave_mem[rd_idx];
assign wave_y = sample_to_y(wave_sample);
assign wave_pixel = in_wave && (y_pos >= wave_y - 10'd1) && (y_pos <= wave_y + 10'd1);

assign bar_rel_x = x_pos - BAR_X_W;
assign bar_rel_y = BAR_Y_END - y_pos;
assign bar_idx = bar_rel_x[7:5];
assign bin_level = get_bin_level(bar_idx);
assign peak_level = get_peak_level(bar_idx);
assign bar_height = {4'b0000, bin_level};
assign peak_y = BAR_Y_END - {4'b0000, peak_level};
assign bar_pixel = in_bars &&
                   (bar_rel_x[4:0] >= 5'd3) && (bar_rel_x[4:0] <= 5'd20) &&
                   (bar_rel_y <= bar_height);
assign peak_pixel = in_bars &&
                    (bar_rel_x[4:0] >= 5'd3) && (bar_rel_x[4:0] <= 5'd20) &&
                    (y_pos >= peak_y - 10'd1) && (y_pos <= peak_y);
assign sparkle_pixel = in_panel && energy_hold[5] &&
                       ((x_pos[4:0] ^ y_pos[4:0] ^ write_idx[4:0]) == 5'd0);

assign left_sample = {I_audio_left[23], I_audio_left[23:18]};
assign right_sample = {I_audio_right[23], I_audio_right[23:18]};
assign mixed_sample = left_sample + right_sample;
assign audio_sample = mixed_sample[6:1];
assign audio_abs = audio_sample[5] ? (~audio_sample + 6'd1) : audio_sample;

assign bg_r = {2'b00, I_rgb[23:18]} + {3'b000, I_rgb[23:19]};
assign bg_g = {2'b00, I_rgb[15:10]} + {3'b000, I_rgb[15:11]};
assign bg_b = {2'b00, I_rgb[7:2]}   + {3'b000, I_rgb[7:3]};
assign panel_rgb = {bg_r, bg_g, bg_b};

assign bar_rgb = bar_rel_y[5] ? 24'hFFB84C :
                 bar_rel_y[4] ? 24'h6CFFB0 :
                                 24'h40C8FF;

assign stage_rgb = sparkle_pixel ? 24'hFFFFFF :
                   wave_pixel    ? 24'hA0FF70 :
                   peak_pixel    ? 24'hFFE060 :
                   bar_pixel     ? bar_rgb :
                   center_line   ? 24'h4C6060 :
                   panel_border  ? 24'h70D0FF :
                   grid_line     ? 24'h24343C :
                   in_panel      ? panel_rgb :
                                   I_rgb;

assign O_rgb = stage_rgb;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x_pos         <= 10'd0;
        y_pos         <= 10'd0;
        de_d          <= 1'b0;
        write_idx     <= 6'd0;
        sample_div    <= 3'd0;
        last_sample   <= 6'sd0;
        zero_distance <= 12'd0;
        period_est    <= 12'd96;
        energy_hold   <= 6'd0;
        peak_hold     <= 6'd0;
        bin_level0    <= 6'd6;
        bin_level1    <= 6'd10;
        bin_level2    <= 6'd14;
        bin_level3    <= 6'd18;
        bin_level4    <= 6'd20;
        bin_level5    <= 6'd16;
        bin_level6    <= 6'd12;
        bin_level7    <= 6'd8;
        for (i = 0; i < WAVE_COUNT; i = i + 1)
            wave_mem[i] <= 6'sd0;
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
                if (y_pos == V_ACTIVE_W - 10'd1)
                    y_pos <= 10'd0;
                else
                    y_pos <= y_pos + 10'd1;
            end
        end

        if (I_frame_start) begin
            if (energy_hold > 6'd1)
                energy_hold <= energy_hold - 6'd1;
            if (peak_hold > 6'd1)
                peak_hold <= peak_hold - 6'd1;
            decay_bins;
        end

        if (I_audio_valid) begin
            if (zero_distance != 12'hFFF)
                zero_distance <= zero_distance + 12'd1;

            if ((last_sample[5] != audio_sample[5]) && (audio_abs > 6'd2)) begin
                period_est <= zero_distance;
                zero_distance <= 12'd0;
            end

            last_sample <= audio_sample;

            if (audio_abs > energy_hold)
                energy_hold <= audio_abs;
            if (audio_abs > peak_hold)
                peak_hold <= audio_abs;

            update_bins(audio_abs, period_est);

            if (sample_div == 3'd5) begin
                sample_div <= 3'd0;
                wave_mem[write_idx] <= audio_sample;
                write_idx <= write_idx + 6'd1;
            end else begin
                sample_div <= sample_div + 3'd1;
            end
        end
    end
end

function [9:0] sample_to_y;
    input signed [5:0] sample;
    reg signed [10:0] y_signed;
    begin
        y_signed = $signed({1'b0, WAVE_CENTER_W}) - {{5{sample[5]}}, sample};
        sample_to_y = y_signed[9:0];
    end
endfunction

function [5:0] get_bin_level;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: get_bin_level = bin_level0;
            3'd1: get_bin_level = bin_level1;
            3'd2: get_bin_level = bin_level2;
            3'd3: get_bin_level = bin_level3;
            3'd4: get_bin_level = bin_level4;
            3'd5: get_bin_level = bin_level5;
            3'd6: get_bin_level = bin_level6;
            default: get_bin_level = bin_level7;
        endcase
    end
endfunction

function [5:0] get_peak_level;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: get_peak_level = bin_level0 + 6'd4;
            3'd1: get_peak_level = bin_level1 + 6'd4;
            3'd2: get_peak_level = bin_level2 + 6'd4;
            3'd3: get_peak_level = bin_level3 + 6'd4;
            3'd4: get_peak_level = bin_level4 + 6'd4;
            3'd5: get_peak_level = bin_level5 + 6'd4;
            3'd6: get_peak_level = bin_level6 + 6'd4;
            default: get_peak_level = bin_level7 + 6'd4;
        endcase
    end
endfunction

task decay_bins;
    begin
        if (bin_level0 > 6'd4) bin_level0 <= bin_level0 - 6'd1;
        if (bin_level1 > 6'd4) bin_level1 <= bin_level1 - 6'd1;
        if (bin_level2 > 6'd4) bin_level2 <= bin_level2 - 6'd1;
        if (bin_level3 > 6'd4) bin_level3 <= bin_level3 - 6'd1;
        if (bin_level4 > 6'd4) bin_level4 <= bin_level4 - 6'd1;
        if (bin_level5 > 6'd4) bin_level5 <= bin_level5 - 6'd1;
        if (bin_level6 > 6'd4) bin_level6 <= bin_level6 - 6'd1;
        if (bin_level7 > 6'd4) bin_level7 <= bin_level7 - 6'd1;
    end
endtask

task update_bins;
    input [5:0] amp;
    input [11:0] period;
    reg [5:0] target;
    begin
        target = (amp > 6'd48) ? 6'd56 : (amp + 6'd8);

        if (period > 12'd76) begin
            if (target > bin_level0) bin_level0 <= target;
            if ((target >> 1) > bin_level1) bin_level1 <= target >> 1;
        end else if (period > 12'd64) begin
            if (target > bin_level1) bin_level1 <= target;
            if ((target >> 1) > bin_level2) bin_level2 <= target >> 1;
        end else if (period > 12'd54) begin
            if (target > bin_level2) bin_level2 <= target;
            if ((target >> 1) > bin_level3) bin_level3 <= target >> 1;
        end else if (period > 12'd46) begin
            if (target > bin_level3) bin_level3 <= target;
            if ((target >> 1) > bin_level4) bin_level4 <= target >> 1;
        end else if (period > 12'd38) begin
            if (target > bin_level4) bin_level4 <= target;
            if ((target >> 1) > bin_level5) bin_level5 <= target >> 1;
        end else if (period > 12'd32) begin
            if (target > bin_level5) bin_level5 <= target;
            if ((target >> 1) > bin_level6) bin_level6 <= target >> 1;
        end else if (period > 12'd26) begin
            if (target > bin_level6) bin_level6 <= target;
            if ((target >> 1) > bin_level7) bin_level7 <= target >> 1;
        end else begin
            if (target > bin_level7) bin_level7 <= target;
            if ((target >> 1) > bin_level6) bin_level6 <= target >> 1;
        end
    end
endtask

endmodule
