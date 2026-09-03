module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // Scan FAT32 root directory and find the first scan_target_count BMP files.
    input                       scan_start,
    input                       scan_raw_only,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // Load one BMP from the specified first data sector.
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,
    output reg                  load_failed,

    input                       sd_init_done,
    output reg [3:0]            state_code,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,

    // Source geometry parsed from the BMP header and handed to scaler_nn.
    // src_dim_valid is a one-cycle pulse raised at the same moment write_req
    // goes high, i.e. at the end of ST_LOAD_HDR, so the scaler is armed a full
    // SD command ahead of the first pixel of ST_LOAD_DATA.
    output reg [15:0]           src_width,
    output reg [15:0]           src_height,
    output reg                  src_dim_valid
);

localparam ST_IDLE       = 4'd0;
localparam ST_SCAN_BOOT  = 4'd1;
localparam ST_SCAN_DIR   = 4'd2;
localparam ST_SCAN_RAW   = 4'd3;
localparam ST_LOAD_HDR   = 4'd4;
localparam ST_LOAD_WAIT  = 4'd5;
localparam ST_LOAD_DATA  = 4'd6;
localparam ST_SCAN_ROOT  = 4'd7;

localparam [7:0] ROOT_SCAN_MAX_SECTORS = 8'd128;

// Accepted source geometry. The lower bound is what guarantees scaler_nn can
// drain a destination row before the next source row finishes arriving (see
// the timing argument in scaler_nn.v); the upper bound is the largest frame
// the SD read timeout in sd_card_bmp.v is sized for. A top-down BMP encodes a
// negative biHeight, whose upper half is non-zero, so it is rejected here.
localparam [15:0] SRC_W_MIN = 16'd64;
localparam [15:0] SRC_W_MAX = 16'd1920;
localparam [15:0] SRC_H_MIN = 16'd64;
localparam [15:0] SRC_H_MAX = 16'd1080;

reg [3:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

reg [31:0] boot_sector_lba;
reg        tried_mbr;
reg [15:0] bpb_bytes_per_sector;
reg [7:0]  bpb_sec_per_cluster;
reg [15:0] bpb_reserved_sectors;
reg [7:0]  bpb_num_fats;
reg [15:0] bpb_fat_size16;
reg [31:0] bpb_fat_size32;
reg [31:0] bpb_root_cluster;
reg [7:0]  boot_sig0;
reg [7:0]  boot_sig1;
reg [7:0]  mbr_part_type;
reg [31:0] mbr_part_lba;
reg [31:0] data_start_sector;
reg [31:0] root_dir_sector;
reg [31:0] dir_sector;
reg [7:0]  dir_sector_count;

reg [7:0]  dir_first_byte;
reg [7:0]  dir_ext0;
reg [7:0]  dir_ext1;
reg [7:0]  dir_ext2;
reg [7:0]  dir_attr;
reg [15:0] dir_cluster_hi;
reg [15:0] dir_cluster_lo;
reg [31:0] dir_file_size;

wire reading_sector;
wire header_match_c;
reg  header_match_r;
reg  width_ok_r;
reg  height_ok_r;
wire bmp_data_valid;
wire in_pixel_region;
wire [31:0] w3_calc;
wire [31:0] stride_calc;
reg  [15:0] src_w3;
reg  [15:0] src_stride;
reg  [15:0] row_byte_cnt;
wire [31:0] file_sector_count;
reg  [31:0] file_sector_count_r;
reg  [31:0] next_scan_sector_if_match;
reg  [31:0] next_scan_sector_if_miss;
wire [31:0] bpb_fat_size;
wire [31:0] bpb_fat_area_sectors;
wire [31:0] bpb_data_start_calc;
wire [31:0] bpb_root_dir_calc;
reg  [31:0] root_cluster_offset_r;
reg  [31:0] dir_cluster_offset_r;
wire        boot_is_fat32;
wire        mbr_has_partition;
reg         boot_geom_ok_r;
reg         mbr_geom_ok_r;
wire [31:0] dir_entry_cluster;
wire [31:0] dir_file_size_now;
wire        dir_ext_is_bmp;
wire        dir_entry_is_file;
wire        dir_entry_is_bmp_now;
wire [31:0] dir_file_sector_now;

assign ready = (state == ST_IDLE);
assign reading_sector = (state == ST_SCAN_BOOT) ||
                        (state == ST_SCAN_DIR)  ||
                        (state == ST_SCAN_RAW)  ||
                        (state == ST_LOAD_HDR);

// Header validation is now a range check on the parsed geometry instead of an
// equality check against a hard-coded display size, which is what lets the
// scaler accept arbitrary resolutions. The geometry compares are registered:
// four wide comparators feeding straight into the ST_SCAN_RAW / ST_LOAD_HDR
// decision would land in the sd_card_clk domain, which only has about 4%
// margin. The parsed fields settle at rd_cnt == 33 while header_match_r is not
// sampled until sd_sec_read_end near rd_cnt == 511, so two cycles of latency
// are invisible.
assign header_match_c = (header_0 == "B") &&
                        (header_1 == "M") &&
                        width_ok_r &&
                        height_ok_r &&
                        (bit_count   == 16'd24) &&
                        (compression == 32'd0);

// Padding gate. A 24-bit BMP row is padded up to a multiple of four bytes, so
// any width that is not itself a multiple of four carries padding bytes that
// must not be mistaken for pixel data. 640 * 3 = 1920 happens to be aligned,
// which is why the previous version got away without this.
assign in_pixel_region = (bmp_len_cnt >= pixel_offset) &&
                         (bmp_len_cnt <  file_len);
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        in_pixel_region &&
                        (row_byte_cnt < src_w3);

assign w3_calc     = width + (width << 1);
assign stride_calc = (w3_calc + 32'd3) & 32'hffff_fffc;
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);

// ---------------------------------------------------------------------------
// Two-stage pipeline for the ST_SCAN_RAW sector address arithmetic.
//
// This used to be one purely combinational chain:
//   file_len -> (+511) -> >>9 -> mux -> file_sector_count
//            -> (+scan_sector)          -> next_scan_sector_if_match
//            -> (> scan_max_sector)     -> scan_done
// Two 32-bit adders plus a 32-bit comparator in series measured at Logic
// Level 10 (ADDER=4 LUT5=3 LUT4=3) with a 9.686ns data path against a 10ns
// budget -- the worst path of the sd_card_clk domain (Fmax 100.341MHz, only
// 0.034ns of margin).
//
// Splitting it into two register stages leaves a single adder per stage.
// This is functionally exact, not an approximation, because both operands are
// stable long before the result is consumed:
//   * file_len is written at the very beginning of the streamed sector
//     (rd_cnt = 2..5, see the header parse block above), whereas
//     next_scan_sector_if_match is only consumed at sd_sec_read_end, i.e.
//     after rd_cnt has walked the whole 512-byte sector. That is several
//     hundred data_valid cycles of slack.
//   * scan_sector does not change during a sector read at all; it is only
//     updated at sd_sec_read_end or on entry to ST_SCAN_RAW.
// So the extra 1~2 cycles of latency are far shorter than the time until the
// consumer samples the value, and no behaviour changes.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        file_sector_count_r       <= 32'd1;
        next_scan_sector_if_match <= 32'd0;
        next_scan_sector_if_miss  <= 32'd0;
    end else begin
        file_sector_count_r       <= file_sector_count;
        next_scan_sector_if_match <= scan_sector + file_sector_count_r;
        next_scan_sector_if_miss  <= scan_sector + 32'd1;
    end
end

assign bpb_fat_size = (bpb_fat_size16 != 16'd0) ? {16'd0, bpb_fat_size16} : bpb_fat_size32;
assign bpb_fat_area_sectors = (bpb_num_fats == 8'd1) ? bpb_fat_size :
                              (bpb_num_fats == 8'd2) ? (bpb_fat_size << 1) :
                                                        (bpb_fat_size << 1);
assign bpb_data_start_calc = boot_sector_lba +
                             {16'd0, bpb_reserved_sectors} +
                             bpb_fat_area_sectors;
// ---------------------------------------------------------------------------
// Registered cluster->sector offset for the FAT32 root directory.
//
// bpb_root_dir_calc used to be built fully combinationally:
//   bpb_root_cluster -> (-2) -> 8-way shift mux on bpb_sec_per_cluster
//                    -> (+ data_start_sector) -> dir_sector / sd_sec_read_addr
// That measured at Logic Level 9 (subtractor + 3 mux levels + adder) with a
// 13.971ns arrival against the 10ns sd_card_clk budget -- the worst path of the
// domain once the ST_SCAN_RAW chain above was pipelined.
//
// Only the offset half is registered, NOT bpb_root_dir_calc as a whole. This
// distinction is load-bearing:
//   * bpb_sec_per_cluster is parsed at rd_cnt == 13 and bpb_root_cluster at
//     rd_cnt == 44..47, so the offset is settled roughly 460 cycles before
//     sd_sec_read_end releases ST_SCAN_BOOT. One cycle of latency is invisible.
//   * data_start_sector, on the other hand, is loaded at that very
//     sd_sec_read_end (ST_SCAN_BOOT), and ST_SCAN_ROOT consumes
//     bpb_root_dir_calc on the immediately following cycle. Registering the
//     whole sum would therefore sample a stale data_start_sector and compute
//     the wrong root directory sector.
// Keeping the final adder combinational preserves the original behaviour
// exactly while removing the subtractor and the shift mux from the path.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) root_cluster_offset_r <= 32'd0;
    else     root_cluster_offset_r <= cluster_sector_offset(bpb_root_cluster - 32'd2, bpb_sec_per_cluster);
end

assign bpb_root_dir_calc = data_start_sector + root_cluster_offset_r;

// ---------------------------------------------------------------------------
// Same registered-offset treatment for the per-entry directory cluster.
//
// dir_file_sector_now has exactly the structure that bpb_root_dir_calc used to
// have (32-bit subtractor -> 8-way shift mux on bpb_sec_per_cluster -> 32-bit
// adder), and after the two fixes above it became the worst remaining path of
// the sd_card_clk domain:
//   bpb_sec_per_cluster_reg[5] -> scan_found_sector_reg[28], 9.617ns, +0.103ns
// That leaves only 1.04% margin at 100MHz, which is not enough headroom to
// absorb the scaler that stage 3 adds to this same clock domain.
//
// Only the offset half is registered. Safety margin, verified against the
// directory-entry parse sequence:
//   * dir_cluster_hi is captured at rd_cnt[4:0] == 20/21 and dir_cluster_lo at
//     == 26/27, while the single consumer (scan_found_sector <= ) samples at
//     == 31. The cluster number is therefore stable for 4 cycles before use,
//     so one cycle of latency is invisible and the register still holds the
//     value derived from the same entry.
//   * data_start_sector is kept in the combinational final adder, exactly as
//     for bpb_root_dir_calc, so no staleness can reach the sector address.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) dir_cluster_offset_r <= 32'd0;
    else     dir_cluster_offset_r <= cluster_sector_offset(dir_entry_cluster - 32'd2, bpb_sec_per_cluster);
end
// Everything boot_is_fat32 tests apart from the two signature bytes is
// captured by rd_cnt 47, and mbr_has_partition apart from nothing at all is
// captured by rd_cnt 457, while the FSM only acts on either of them at
// sd_sec_read_end past byte 511. Pre-registering that half is therefore
// invisible to the state machine, and it lifts the 32-bit bpb_root_cluster
// comparator carry chain out of the sd_card_clk critical path: that chain plus
// the FSM decode behind it was the worst path in the design once the scaler
// raised utilisation enough for placement to spread it out.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        boot_geom_ok_r <= 1'b0;
        mbr_geom_ok_r  <= 1'b0;
    end else begin
        boot_geom_ok_r <= (bpb_bytes_per_sector == 16'd512) &&
                          (bpb_sec_per_cluster  != 8'd0)    &&
                          (bpb_num_fats         != 8'd0)    &&
                          (bpb_fat_size         != 32'd0)   &&
                          (bpb_root_cluster     >= 32'd2);
        mbr_geom_ok_r  <= (mbr_part_type != 8'd0) && (mbr_part_lba != 32'd0);
    end
end
assign boot_is_fat32 = (boot_sig0 == 8'h55) &&
                       (boot_sig1 == 8'haa) &&
                       boot_geom_ok_r;
assign mbr_has_partition = mbr_geom_ok_r;

assign dir_entry_cluster = {dir_cluster_hi, dir_cluster_lo};
assign dir_file_size_now = {sd_sec_read_data, dir_file_size[23:0]};
assign dir_ext_is_bmp = ((dir_ext0 == "B") || (dir_ext0 == "b")) &&
                        ((dir_ext1 == "M") || (dir_ext1 == "m")) &&
                        ((dir_ext2 == "P") || (dir_ext2 == "p"));
assign dir_entry_is_file = (dir_first_byte != 8'h00) &&
                           (dir_first_byte != 8'he5) &&
                           (dir_attr != 8'h0f) &&
                           (!dir_attr[3]) &&
                           (!dir_attr[4]) &&
                           (dir_entry_cluster >= 32'd2) &&
                           (dir_file_size_now != 32'd0);
assign dir_entry_is_bmp_now = dir_entry_is_file && dir_ext_is_bmp;
assign dir_file_sector_now = data_start_sector + dir_cluster_offset_r;

function [31:0] cluster_sector_offset;
    input [31:0] cluster_delta;
    input [7:0]  sectors_per_cluster;
    begin
        case (sectors_per_cluster)
            8'd1:    cluster_sector_offset = cluster_delta;
            8'd2:    cluster_sector_offset = cluster_delta << 1;
            8'd4:    cluster_sector_offset = cluster_delta << 2;
            8'd8:    cluster_sector_offset = cluster_delta << 3;
            8'd16:   cluster_sector_offset = cluster_delta << 4;
            8'd32:   cluster_sector_offset = cluster_delta << 5;
            8'd64:   cluster_sector_offset = cluster_delta << 6;
            8'd128:  cluster_sector_offset = cluster_delta << 7;
            default: cluster_sector_offset = cluster_delta;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if (reading_sector) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN_RAW) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;
            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;
            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;
            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;
            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;
            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;
            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bpb_bytes_per_sector <= 16'd0;
        bpb_sec_per_cluster  <= 8'd0;
        bpb_reserved_sectors <= 16'd0;
        bpb_num_fats         <= 8'd0;
        bpb_fat_size16       <= 16'd0;
        bpb_fat_size32       <= 32'd0;
        bpb_root_cluster     <= 32'd0;
        boot_sig0            <= 8'd0;
        boot_sig1            <= 8'd0;
        mbr_part_type        <= 8'd0;
        mbr_part_lba         <= 32'd0;
    end else if ((state == ST_SCAN_BOOT) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd11: bpb_bytes_per_sector[7:0] <= sd_sec_read_data;
            10'd12: bpb_bytes_per_sector[15:8] <= sd_sec_read_data;
            10'd13: bpb_sec_per_cluster <= sd_sec_read_data;
            10'd14: bpb_reserved_sectors[7:0] <= sd_sec_read_data;
            10'd15: bpb_reserved_sectors[15:8] <= sd_sec_read_data;
            10'd16: bpb_num_fats <= sd_sec_read_data;
            10'd22: bpb_fat_size16[7:0] <= sd_sec_read_data;
            10'd23: bpb_fat_size16[15:8] <= sd_sec_read_data;
            10'd36: bpb_fat_size32[7:0] <= sd_sec_read_data;
            10'd37: bpb_fat_size32[15:8] <= sd_sec_read_data;
            10'd38: bpb_fat_size32[23:16] <= sd_sec_read_data;
            10'd39: bpb_fat_size32[31:24] <= sd_sec_read_data;
            10'd44: bpb_root_cluster[7:0] <= sd_sec_read_data;
            10'd45: bpb_root_cluster[15:8] <= sd_sec_read_data;
            10'd46: bpb_root_cluster[23:16] <= sd_sec_read_data;
            10'd47: bpb_root_cluster[31:24] <= sd_sec_read_data;
            10'd450: mbr_part_type <= sd_sec_read_data;
            10'd454: mbr_part_lba[7:0] <= sd_sec_read_data;
            10'd455: mbr_part_lba[15:8] <= sd_sec_read_data;
            10'd456: mbr_part_lba[23:16] <= sd_sec_read_data;
            10'd457: mbr_part_lba[31:24] <= sd_sec_read_data;
            10'd510: boot_sig0 <= sd_sec_read_data;
            10'd511: boot_sig1 <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        dir_first_byte <= 8'd0;
        dir_ext0       <= 8'd0;
        dir_ext1       <= 8'd0;
        dir_ext2       <= 8'd0;
        dir_attr       <= 8'd0;
        dir_cluster_hi <= 16'd0;
        dir_cluster_lo <= 16'd0;
        dir_file_size  <= 32'd0;
    end else if ((state == ST_SCAN_DIR) && sd_sec_read_data_valid) begin
        case (rd_cnt[4:0])
            5'd0 : dir_first_byte <= sd_sec_read_data;
            5'd8 : dir_ext0 <= sd_sec_read_data;
            5'd9 : dir_ext1 <= sd_sec_read_data;
            5'd10: dir_ext2 <= sd_sec_read_data;
            5'd11: dir_attr <= sd_sec_read_data;
            5'd20: dir_cluster_hi[7:0] <= sd_sec_read_data;
            5'd21: dir_cluster_hi[15:8] <= sd_sec_read_data;
            5'd26: dir_cluster_lo[7:0] <= sd_sec_read_data;
            5'd27: dir_cluster_lo[15:8] <= sd_sec_read_data;
            5'd28: dir_file_size[7:0] <= sd_sec_read_data;
            5'd29: dir_file_size[15:8] <= sd_sec_read_data;
            5'd30: dir_file_size[23:16] <= sd_sec_read_data;
            5'd31: dir_file_size[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

// Byte index inside the current source row, used to drop the 4-byte alignment
// padding. It advances on every byte of the pixel region, padding included,
// and wraps at the row stride, which keeps it aligned with bmp_len_cnt.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        row_byte_cnt <= 16'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid && in_pixel_region) begin
            if ((row_byte_cnt + 16'd1) >= src_stride)
                row_byte_cnt <= 16'd0;
            else
                row_byte_cnt <= row_byte_cnt + 16'd1;
        end
    end else begin
        row_byte_cnt <= 16'd0;
    end
end

// Registered geometry validation plus the derived row pitch. All of these are
// quasi-static for the whole image: width and height are parsed once during
// ST_LOAD_HDR and never change until the next header.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        width_ok_r  <= 1'b0;
        height_ok_r <= 1'b0;
        src_w3      <= 16'd0;
        src_stride  <= 16'd0;
        src_width   <= 16'd0;
        src_height  <= 16'd0;
    end else begin
        width_ok_r  <= (width[31:16]  == 16'd0) &&
                       (width[15:0]   >= SRC_W_MIN) &&
                       (width[15:0]   <= SRC_W_MAX);
        height_ok_r <= (height[31:16] == 16'd0) &&
                       (height[15:0]  >= SRC_H_MIN) &&
                       (height[15:0]  <= SRC_H_MAX);
        src_w3      <= w3_calc[15:0];
        src_stride  <= stride_calc[15:0];
        src_width   <= width[15:0];
        src_height  <= height[15:0];
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) header_match_r <= 1'b0;
    else     header_match_r <= header_match_c;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                <= ST_IDLE;
        state_code           <= 4'd0;
        sd_sec_read          <= 1'b0;
        sd_sec_read_addr     <= 32'd0;
        write_req            <= 1'b0;
        load_failed          <= 1'b0;
        src_dim_valid        <= 1'b0;
        scan_done            <= 1'b0;
        scan_found_valid     <= 1'b0;
        scan_found_sector    <= 32'd0;
        scan_found_total     <= 3'd0;
        scan_sector          <= 32'd0;
        load_sector_latched  <= 32'd0;
        boot_sector_lba      <= 32'd0;
        tried_mbr            <= 1'b0;
        data_start_sector    <= 32'd0;
        root_dir_sector      <= 32'd0;
        dir_sector           <= 32'd0;
        dir_sector_count     <= 8'd0;
    end else if (!sd_init_done) begin
        state                <= ST_IDLE;
        state_code           <= 4'd0;
        sd_sec_read          <= 1'b0;
        sd_sec_read_addr     <= 32'd0;
        write_req            <= 1'b0;
        load_failed          <= 1'b0;
        src_dim_valid        <= 1'b0;
        scan_done            <= 1'b0;
        scan_found_valid     <= 1'b0;
        scan_found_sector    <= 32'd0;
        scan_found_total     <= 3'd0;
        scan_sector          <= 32'd0;
        load_sector_latched  <= 32'd0;
        boot_sector_lba      <= 32'd0;
        tried_mbr            <= 1'b0;
        data_start_sector    <= 32'd0;
        root_dir_sector      <= 32'd0;
        dir_sector           <= 32'd0;
        dir_sector_count     <= 8'd0;
    end else if (load_abort) begin
        state                <= ST_IDLE;
        state_code           <= 4'd1;
        sd_sec_read          <= 1'b0;
        write_req            <= 1'b0;
        load_failed          <= 1'b0;
        src_dim_valid        <= 1'b0;
        scan_found_valid     <= 1'b0;
        load_sector_latched  <= 32'd0;
    end else begin
        scan_found_valid <= 1'b0;
        load_failed      <= 1'b0;
        src_dim_valid    <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done         <= 1'b0;
                    scan_found_total  <= 3'd0;
                    boot_sector_lba   <= scan_start_sector;
                    tried_mbr         <= 1'b0;
                    sd_sec_read_addr  <= scan_start_sector;
                    if (scan_target_count == 3'd0) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else if (scan_raw_only) begin
                        scan_sector <= scan_start_sector;
                        state       <= ST_SCAN_RAW;
                    end else begin
                        state <= ST_SCAN_BOOT;
                    end
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN_BOOT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (boot_is_fat32) begin
                        data_start_sector <= bpb_data_start_calc;
                        state             <= ST_SCAN_ROOT;
                    end else if (!tried_mbr && mbr_has_partition) begin
                        tried_mbr        <= 1'b1;
                        boot_sector_lba  <= mbr_part_lba;
                        sd_sec_read_addr <= mbr_part_lba;
                        state            <= ST_SCAN_BOOT;
                    end else begin
                        scan_sector      <= scan_start_sector;
                        sd_sec_read_addr <= scan_start_sector;
                        state            <= ST_SCAN_RAW;
                    end
                end
            end

            ST_SCAN_ROOT: begin
                state_code        <= 4'd2;
                root_dir_sector   <= bpb_root_dir_calc;
                dir_sector        <= bpb_root_dir_calc;
                dir_sector_count  <= 8'd0;
                sd_sec_read_addr  <= bpb_root_dir_calc;
                state             <= ST_SCAN_DIR;
            end

            ST_SCAN_DIR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_data_valid && (rd_cnt[4:0] == 5'd31)) begin
                    if (dir_first_byte == 8'h00) begin
                        scan_done   <= 1'b1;
                        sd_sec_read <= 1'b0;
                        state       <= ST_IDLE;
                    end else if (dir_entry_is_bmp_now) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= dir_file_sector_now;
                        scan_found_total  <= scan_found_total + 3'd1;
                        if (scan_found_total + 3'd1 >= scan_target_count) begin
                            scan_done   <= 1'b1;
                            sd_sec_read <= 1'b0;
                            state       <= ST_IDLE;
                        end
                    end
                end

                if (sd_sec_read_end && !scan_done && (state == ST_SCAN_DIR)) begin
                    sd_sec_read <= 1'b0;
                    if (dir_sector_count >= ROOT_SCAN_MAX_SECTORS - 8'd1) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        dir_sector       <= dir_sector + 32'd1;
                        dir_sector_count <= dir_sector_count + 8'd1;
                        sd_sec_read_addr <= dir_sector + 32'd1;
                    end
                end
            end

            ST_SCAN_RAW: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match_r) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        if (scan_sector >= scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match_r) begin
                        src_dim_valid    <= 1'b1;
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        load_failed <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
