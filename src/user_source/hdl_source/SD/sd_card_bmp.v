module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    // Extra attempts per picture after the first one. sd_card_sec_read_write
    // already retries a missed start token three times, but when all three
    // miss it skips the sector and keeps going. During ST_LOAD_DATA that
    // shifts every later pixel by 512 bytes; during ST_LOAD_HDR it produces a
    // headerless sector, header_match_r fails, load_failed fires, and the
    // picture would be lost for good -- next_load_idx has already advanced
    // past it and nothing ever walks it back. Retrying the whole picture from
    // its header is what turns a transient bit error into a slightly longer
    // startup instead of a permanently short carousel.
    parameter [2:0]   LOAD_MAX_RETRY    = 3'd3
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    // Serial-screen commands, already crossed into this sd_card_clk domain by
    // the top level (toggle-CDC for the pulses, data+toggle for cmd_img_sel), so
    // this module stays single-clock. The two pulses OR into the debounced key
    // conditions below; cmd_img_sel directly selects an already-loaded picture.
    input                       cmd_next_pulse,
    input                       cmd_auto_pulse,
    input       [1:0]           cmd_img_sel,
    input                       cmd_img_sel_pulse,
    output [3:0]                state_code,
    output reg                  display_valid,
    output                      auto_play_enabled,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    // Occupancy of the write-side async FIFO, routed out of frame_read_write so
    // the scaler can apply backpressure from inside this clock domain.
    input  [8:0]                write_fifo_usedw,

    // Audio stream FIFO write side. The async FIFO body lives in the top level
    // (it crosses sd_card_clk -> video_clk); sd_card_bmp stays single-clock and
    // only drives the write port and reads the write-side occupancy for the
    // streamer's sector-boundary backpressure.
    output                      aud_fifo_we,
    output [31:0]               aud_fifo_di,
    input  [8:0]                aud_fifo_wrusedw,

    // Bring-up visibility for a silent audio chain, read by the 7-segment in the
    // top level. dbg_audio_chain is {wav_found, audio_phase, ever_we, fault}:
    // the scan saw a WAV, the SD port was handed to the streamer, the streamer
    // wrote at least one frame, and it rejected the header. dbg_aud_wr_peak is a
    // sticky high-water mark of aud_fifo_wrusedw[8:5], so a FIFO that filled and
    // drained again still reads back how full it got.
    output [3:0]                dbg_audio_chain,
    output reg [3:0]            dbg_aud_wr_peak,

    // chain == 8 says the hand-off gate never fired, but not which of its
    // inputs held it off. dbg_gate is {bmp_ready, ~load_busy, scan_done,
    // display_valid} and dbg_loaded_cnt is img_loaded_count, so one more
    // readout names the stuck term instead of guessing at it.
    output [3:0]                dbg_gate,
    output [3:0]                dbg_loaded_cnt,

    // count < 4 on hardware means the fourth picture never committed, which is
    // a scan problem (found_cnt < 4) or a load problem (next_idx ran past the
    // last found entry after the retries gave up). dbg_fail says which load
    // failure mode fired: bit3 the 1 s no-progress watchdog, bit2 a rejected
    // header sector, bits1:0 the retry counter at the moment of reading.
    output [3:0]                dbg_fail,
    output [3:0]                dbg_found_cnt,
    output [3:0]                dbg_next_idx,

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
// Two disjoint consumers of the single SD sector-read port: bmp_read (pictures)
// and sd_audio_stream (music). They never run at the same time -- audio_phase
// only rises after the last picture is committed -- so a 2:1 mux on the request
// and address is enough; the data/valid/end come back from sd_card_top and are
// broadcast to both (the idle consumer ignores them).
wire             bmp_sd_sec_read;
wire [31:0]      bmp_sd_sec_read_addr;
wire             aud_sd_sec_read;
wire [31:0]      aud_sd_sec_read_addr;
wire             aud_dbg_ever_we;
wire             aud_dbg_fault;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire [15:0]      bmp_src_width;
wire [15:0]      bmp_src_height;
wire             bmp_src_dim_valid;
wire             scaler_dst_valid;
wire [23:0]      scaler_dst_pixel;
wire             sd_init_done;
wire             bmp_ready;
wire [3:0]       bmp_state_code;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;
wire             scan_found_wav_valid;
wire [31:0]      scan_found_wav_sector;
wire [31:0]      scan_found_wav_size;
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
reg [2:0]        load_retry_cnt;

// WAV directory entry captured during the scan, and the sticky flag that hands
// the SD port over to sd_audio_stream once every picture is loaded.
reg              wav_found;
reg [31:0]       wav_sector;
reg [31:0]       wav_size;
reg              audio_phase;
reg              dbg_stall_seen;
reg              dbg_hdr_seen;

reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_loaded;
wire       source_done_now;
wire       write_done_now;
wire       load_complete_now;
wire       load_progress;
wire [2:0] loaded_count_plus_one;
wire       load_stall_hit;
wire       load_gave_up;

// The scaler owns the write stream now. It emits exactly FRAME_WIDTH *
// FRAME_HEIGHT pixels per image whatever the source geometry, which is what
// keeps write_len and the WRITE_V_FLIP row addressing in frame_fifo_write
// valid without any change there. bmp_data_wr_en only feeds the scaler.
assign write_en   = scaler_dst_valid;
assign write_data = {scaler_dst_pixel, 8'b0};
// SD sector-read port ownership: pictures first, music after they are all in.
assign sd_sec_read      = audio_phase ? aud_sd_sec_read      : bmp_sd_sec_read;
assign sd_sec_read_addr = audio_phase ? aud_sd_sec_read_addr : bmp_sd_sec_read_addr;
assign dbg_audio_chain  = {wav_found, audio_phase, aud_dbg_ever_we, aud_dbg_fault};
assign dbg_gate         = {bmp_ready, ~load_busy, scan_done, display_valid};
assign dbg_loaded_cnt   = {1'b0, img_loaded_count};
assign dbg_fail         = {dbg_stall_seen, dbg_hdr_seen, load_retry_cnt[1:0]};
assign dbg_found_cnt    = {1'b0, img_found_count};
assign dbg_next_idx     = {1'b0, next_load_idx};
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_loaded = next_index_limited(img_idx, img_loaded_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign source_done_now = source_done_seen | (load_busy && bmp_ready);
assign write_done_now  = write_done_seen  | (load_busy && write_finish_pulse);
assign load_complete_now = load_busy && source_done_now && write_done_now;
// The stall watchdog is fed by anything that shows the load is still moving.
// scaler_dst_valid is included because the source stream ends before the
// destination stream does whenever the scaler has black border rows left to
// emit.
assign load_progress = bmp_data_wr_en || scaler_dst_valid || write_finish_pulse || write_req_ack;
assign loaded_count_plus_one = img_loaded_count + 3'd1;
// The 1s silence watchdog. Named here rather than spelled out inline because
// the retry decision below has to cover it as well as load_failed: from this
// module's point of view a picture that stopped moving and a picture whose
// header came back unreadable are the same event, an attempt that did not
// produce a frame buffer worth keeping.
assign load_stall_hit = load_busy && !load_progress &&
                        (load_stall_cnt >= CLK_FREQ_HZ - 1);
assign load_gave_up   = (load_busy && load_failed) || load_stall_hit;
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
        load_retry_cnt        <= 3'd0;
        display_valid         <= 1'b0;
        wav_found             <= 1'b0;
        wav_sector            <= 32'd0;
        wav_size              <= 32'd0;
        audio_phase           <= 1'b0;
        dbg_stall_seen        <= 1'b0;
        dbg_hdr_seen          <= 1'b0;
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
            load_retry_cnt        <= 3'd0;
            display_valid         <= 1'b0;
            scan_raw_only         <= 1'b0;
            raw_fallback_started  <= 1'b0;
            wav_found             <= 1'b0;
            wav_sector            <= 32'd0;
            wav_size              <= 32'd0;
            audio_phase           <= 1'b0;
            dbg_stall_seen        <= 1'b0;
            dbg_hdr_seen          <= 1'b0;
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

            // Capture the WAV directory entry (sector + byte size) the first
            // time the scan reports it. bmp_read only ever emits this once.
            if (scan_found_wav_valid) begin
                wav_found  <= 1'b1;
                wav_sector <= scan_found_wav_sector;
                wav_size   <= scan_found_wav_size;
            end

            // Hand the SD sector-read port to the music streamer only once every
            // picture is committed and bmp_read has gone idle, so the two
            // consumers never overlap. Sticky until reset / card-pull re-scan,
            // which keeps music playing across the whole slideshow and the
            // single-track loop.
            if (!audio_phase && wav_found && bmp_ready && !load_busy &&
                (img_loaded_count >= SCAN_TARGET_COUNT))
                audio_phase <= 1'b1;

            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            if (load_gave_up) begin
                // bmp_read rejected the header sector, or the picture stopped
                // moving for a whole second. Either way this attempt left
                // nothing worth keeping in the frame buffer.
                if (load_stall_hit) dbg_stall_seen <= 1'b1;
                if (load_failed)    dbg_hdr_seen   <= 1'b1;
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;
                // Only the stall path needs to be told to let go: on
                // load_failed bmp_read has already put itself back in
                // ST_IDLE.
                load_abort       <= load_stall_hit;

                // Walking next_load_idx back re-arms the same picture: the
                // re-arm branch below reads sector_lut(next_load_idx) and
                // load_buf_idx from img_loaded_count, which did not advance,
                // so the retry re-reads the same file into the same buffer.
                if (load_retry_cnt < LOAD_MAX_RETRY) begin
                    load_retry_cnt <= load_retry_cnt + 3'd1;
                    next_load_idx  <= next_load_idx - 3'd1;
                end else begin
                    load_retry_cnt <= 3'd0;
                end
            end else if (load_complete_now) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_stall_cnt   <= 32'd0;
                load_retry_cnt   <= 3'd0;

                if (img_loaded_count < SCAN_TARGET_COUNT)
                    img_loaded_count <= loaded_count_plus_one;

                if (!first_image_committed && (load_buf_idx == 2'd0)) begin
                    disp_buf_idx          <= load_buf_idx;
                    img_idx               <= load_buf_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                end
            end else if (load_busy) begin
                if (load_progress)
                    load_stall_cnt <= 32'd0;
                else
                    load_stall_cnt <= load_stall_cnt + 32'd1;
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
                load_retry_cnt        <= 3'd0;
                scan_raw_only         <= 1'b0;
                raw_fallback_started  <= 1'b0;
            end else begin
                if ((key_auto_press || cmd_auto_pulse) && first_image_committed && (img_found_count > 3'd1)) begin
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

                if ((key_next_press || cmd_next_pulse) && first_image_committed && (img_loaded_count > 3'd1)) begin
                    img_idx      <= next_from_loaded;
                    disp_buf_idx <= next_from_loaded;
                    auto_cnt     <= 32'd0;
                end

                if (cmd_img_sel_pulse && first_image_committed &&
                    ({1'b0, cmd_img_sel} < img_loaded_count)) begin
                    img_idx      <= cmd_img_sel;
                    disp_buf_idx <= cmd_img_sel;
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
                    load_retry_cnt       <= 3'd0;
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
    .scan_found_wav_valid   (scan_found_wav_valid),
    .scan_found_wav_sector  (scan_found_wav_sector),
    .scan_found_wav_size    (scan_found_wav_size),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),
    .load_failed            (load_failed),

    .sd_init_done           (sd_init_done),
    .state_code             (bmp_state_code),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sd_sec_read),
    .sd_sec_read_addr       (bmp_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .src_width              (bmp_src_width),
    .src_height             (bmp_src_height),
    .src_dim_valid          (bmp_src_dim_valid)
);

// Nearest-neighbour scaler, stage 3. Destination geometry is fixed at 640x480
// to match FRAME_WIDTH/FRAME_HEIGHT in frame_read_write, because the
// WRITE_V_FLIP row addressing there assumes exactly 640 pixels per stream row.
scaler_nn #(
    .DST_W                  (640),
    .DST_H                  (480),
    .MAX_UPSCALE            (4),
    // 4096 parked source pixels. Sized in scaler_nn.v for the black border
    // rows, which emit without consuming: off_y rows of 2 * DST_W cycles park
    // off_y * 1280 / 96 pixels of a source stream that cannot be paused.
    .SK_AW                  (12),
    .STALL_THRESH           (384)
) scaler_nn_m0 (
    .clk                    (clk),
    .rst                    (rst),
    .i_src_valid            (bmp_data_wr_en),
    .i_src_pixel            (bmp_data),
    .i_dim_valid            (bmp_src_dim_valid),
    .i_src_w                (bmp_src_width),
    .i_src_h                (bmp_src_height),
    .i_fifo_usedw           (write_fifo_usedw),
    .o_dst_valid            (scaler_dst_valid),
    .o_dst_pixel            (scaler_dst_pixel),
    .o_busy                 (),
    .o_done                 (),
    // Sticky "the elastic buffer overflowed, this picture is wrong" flag, left
    // open on purpose: it is a simulation and bring-up hook, and 4096 entries
    // cover the border row bound for every geometry bmp_read accepts. Hook it
    // to the OSD if a source ever needs more slack than that.
    .o_overflow             ()
);

// Sticky high-water mark of the audio FIFO write side. The streamer refills in
// 128-word sectors and the read side drains at 48 kHz, so the live occupancy is
// useless on a 200 Hz multiplexed display; the peak is not. Zero here means not
// one frame ever reached the FIFO.
always @(posedge clk or posedge rst) begin
    if (rst)
        dbg_aud_wr_peak <= 4'd0;
    else if (aud_fifo_wrusedw[8:5] > dbg_aud_wr_peak)
        dbg_aud_wr_peak <= aud_fifo_wrusedw[8:5];
end

// Music streamer. Shares the SD sector-read port with bmp_read via the mux
// above; start is the sticky audio_phase level, so it begins only after the
// last picture is committed and then loops the single track forever. Its FIFO
// write side is routed straight out to the top level, where the async FIFO
// crosses into video_clk.
sd_audio_stream #(
    .HDR_LEN                (44),
    .PAUSE_THRESH           (9'd256)
) sd_audio_stream_m0 (
    .clk                    (clk),
    .rst                    (rst),
    .start                  (audio_phase),
    .wav_start_sector       (wav_sector),
    .wav_size               (wav_size),
    .sd_sec_read            (aud_sd_sec_read),
    .sd_sec_read_addr       (aud_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .fifo_we                (aud_fifo_we),
    .fifo_di                (aud_fifo_di),
    .fifo_wrusedw           (aud_fifo_wrusedw),
    .dbg_ever_we            (aud_dbg_ever_we),
    .dbg_fault              (aud_dbg_fault)
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
