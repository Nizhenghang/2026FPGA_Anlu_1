module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output                      auto_play_enabled,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire [3:0]       bmp_state_code;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;
wire             load_failed;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_raw_only;
reg              raw_fallback_started;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [2:0]        img_loaded_count;
reg [2:0]        next_load_idx;
reg [1:0]        img_idx;
reg [1:0]        load_idx;
reg [1:0]        load_buf_idx;
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              load_busy;
reg              source_done_seen;
reg              write_done_seen;
reg [31:0]       load_stall_cnt;
reg              load_abort;

reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_loaded;
wire       source_done_now;
wire       write_done_now;
wire       load_complete_now;
wire       load_progress;
wire [2:0] loaded_count_plus_one;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_loaded = next_index_limited(img_idx, img_loaded_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign source_done_now = source_done_seen | (load_busy && bmp_ready);
assign write_done_now  = write_done_seen  | (load_busy && write_finish_pulse);
assign load_complete_now = load_busy && source_done_now && write_done_now;
assign load_progress = bmp_data_wr_en || write_finish_pulse || write_req_ack;
assign loaded_count_plus_one = img_loaded_count + 3'd1;
assign auto_play_enabled = auto_play_en;
assign state_code = (!sd_init_done)                 ? 4'd0 :
                    (display_valid && auto_play_en) ? 4'd6 :
                    (display_valid)                 ? 4'd5 :
                    (scan_raw_only && !scan_done)   ? 4'd7 :
                                                       bmp_state_code;

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_raw_only         <= 1'b0;
        raw_fallback_started  <= 1'b0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_loaded_count      <= 3'd0;
        next_load_idx         <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        load_buf_idx          <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        load_stall_cnt        <= 32'd0;
        load_abort            <= 1'b0;
        display_valid         <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_loaded_count      <= 3'd0;
            next_load_idx         <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            load_buf_idx          <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            load_stall_cnt        <= 32'd0;
            load_abort            <= 1'b0;
            display_valid         <= 1'b0;
            scan_raw_only         <= 1'b0;
            raw_fallback_started  <= 1'b0;
        end else begin
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            if (load_busy && load_failed) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;
            end else if (load_complete_now) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;

                if (img_loaded_count < SCAN_TARGET_COUNT)
                    img_loaded_count <= loaded_count_plus_one;

                if (!first_image_committed && (load_buf_idx == 2'd0)) begin
                    disp_buf_idx          <= load_buf_idx;
                    img_idx               <= load_buf_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                end
            end else if (load_busy) begin
                if (load_progress) begin
                    load_stall_cnt <= 32'd0;
                end else if (load_stall_cnt >= CLK_FREQ_HZ - 1) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_stall_cnt   <= 32'd0;
                    load_abort       <= 1'b1;
                end else begin
                    load_stall_cnt <= load_stall_cnt + 32'd1;
                end
            end else begin
                load_stall_cnt <= 32'd0;
            end

            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_loaded_count      <= 3'd0;
                next_load_idx         <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                load_buf_idx          <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_stall_cnt        <= 32'd0;
                load_abort            <= 1'b0;
                scan_raw_only         <= 1'b0;
                raw_fallback_started  <= 1'b0;
            end else begin
                if (key_auto_press && first_image_committed && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                if (auto_play_en && first_image_committed && (img_loaded_count > 3'd1)) begin
                    if (auto_tick) begin
                        auto_cnt     <= 32'd0;
                        img_idx      <= next_from_loaded;
                        disp_buf_idx <= next_from_loaded;
                    end else begin
                        auto_cnt <= auto_cnt + 32'd1;
                    end
                end else begin
                    auto_cnt <= 32'd0;
                end

                if (key_next_press && first_image_committed && (img_loaded_count > 3'd1)) begin
                    img_idx      <= next_from_loaded;
                    disp_buf_idx <= next_from_loaded;
                    auto_cnt     <= 32'd0;
                end

                if (scan_done && bmp_ready && !load_busy &&
                    !scan_raw_only && !raw_fallback_started && !first_image_committed &&
                    (next_load_idx >= img_found_count)) begin
                    scan_start_pulse     <= 1'b1;
                    scan_raw_only        <= 1'b1;
                    raw_fallback_started <= 1'b1;
                    img_found_count      <= 3'd0;
                    img_loaded_count     <= 3'd0;
                    next_load_idx        <= 3'd0;
                    img_sector0          <= 32'd0;
                    img_sector1          <= 32'd0;
                    img_sector2          <= 32'd0;
                    img_sector3          <= 32'd0;
                    source_done_seen     <= 1'b0;
                    write_done_seen      <= 1'b0;
                    load_stall_cnt       <= 32'd0;
                end else if (scan_done && bmp_ready && !load_busy &&
                             (next_load_idx < img_found_count) &&
                             (img_loaded_count < SCAN_TARGET_COUNT)) begin
                    load_idx         <= next_load_idx[1:0];
                    load_buf_idx     <= img_loaded_count[1:0];
                    load_sector      <= sector_lut(next_load_idx[1:0]);
                    write_buf_idx    <= img_loaded_count[1:0];
                    next_load_idx    <= next_load_idx + 3'd1;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_stall_cnt   <= 32'd0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_raw_only          (scan_raw_only),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),
    .load_failed            (load_failed),

    .sd_init_done           (sd_init_done),
    .state_code             (bmp_state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
