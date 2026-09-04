// Stream uncompressed PCM (a canonical 44-byte-header WAV) off the TF card and
// push stereo frames into an async FIFO that crosses into the video_clk audio
// domain. Lives in the sd_card_clk domain and shares the single SD sector-read
// port with bmp_read; sd_card_bmp muxes the port to this module only after all
// pictures are loaded, so the two consumers are strictly time-disjoint.
//
// Contract with the offline tool (doc/convert/convert_audio_to_wav.py):
//   48000 Hz, stereo, 16-bit little-endian, canonical 44-byte RIFF/WAVE header.
//   PCM byte length = wav_size - 44. 44 and 512 are both multiples of 4, so the
//   4-byte stereo-frame alignment survives the header skip and every sector
//   boundary; no cross-sector frame bookkeeping is needed.
//
// The card reader does NOT follow FAT32 cluster chains (it advances LBA by +1),
// so the WAV must be physically contiguous. The offline sync writes MUSIC.WAV
// first onto a freshly cleaned card to guarantee that.
module sd_audio_stream #(
    parameter integer HDR_LEN      = 44,   // canonical WAV header bytes to skip
    // Pause sector reads at a sector boundary once the FIFO write-side holds
    // this many words. A sector contributes at most 512/4 = 128 frames, so 256
    // caps occupancy at 384 of 512 and can never overflow even if the read side
    // (video_clk) stalls. The drain is 48 kHz vs a ~22 MB/s SD link, so this
    // threshold is rarely reached; it is pure safety.
    parameter [8:0]   PAUSE_THRESH = 9'd256
)(
    input  wire        clk,             // sd_card_clk (100 MHz)
    input  wire        rst,
    input  wire        start,           // level: audio phase active
    input  wire [31:0] wav_start_sector,// first data sector (LBA) of the WAV
    input  wire [31:0] wav_size,        // full file size in bytes

    // SD sector-read bus (muxed onto sd_card_top by sd_card_bmp during audio phase)
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    // FIFO write side
    output reg         fifo_we,
    output reg  [31:0] fifo_di,         // {R[15:0], L[15:0]}
    input  wire [8:0]  fifo_wrusedw
);

localparam [1:0] S_IDLE  = 2'd0;
localparam [1:0] S_READ  = 2'd1;
localparam [1:0] S_WAIT  = 2'd2;   // backpressure: hold between sectors
localparam [1:0] S_FAULT = 2'd3;   // bad/missing header: silent, terminal

localparam [5:0] HDR_SKIP = HDR_LEN;   // 44 fits in 6 bits

reg [1:0]  state;
reg [31:0] pcm_total;     // wav_size - HDR_LEN
reg [31:0] pcm_cnt;       // PCM bytes emitted this pass
reg [5:0]  hdr_skip;      // header bytes left to skip this pass (0..44)
reg [5:0]  hdr_cnt;       // header bytes consumed this pass, for magic capture
reg [1:0]  byte_phase;    // position within the 4-byte stereo frame
reg        magic_done;    // header verified on the first pass
reg [7:0]  riff0, riff1, riff2, riff3;
reg [7:0]  wave0, wave1, wave2, wave3;
reg [7:0]  l_lo, l_hi, r_lo;

wire magic_ok = (riff0 == "R") && (riff1 == "I") && (riff2 == "F") && (riff3 == "F") &&
                (wave0 == "W") && (wave1 == "A") && (wave2 == "V") && (wave3 == "E");
wire wav_usable = (wav_size > (HDR_LEN + 4));
wire pause_now  = (fifo_wrusedw >= PAUSE_THRESH);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state          <= S_IDLE;
        sd_sec_read    <= 1'b0;
        sd_sec_read_addr <= 32'd0;
        fifo_we        <= 1'b0;
        fifo_di        <= 32'd0;
        pcm_total      <= 32'd0;
        pcm_cnt        <= 32'd0;
        hdr_skip       <= 6'd0;
        hdr_cnt        <= 6'd0;
        byte_phase     <= 2'd0;
        magic_done     <= 1'b0;
        riff0 <= 8'd0; riff1 <= 8'd0; riff2 <= 8'd0; riff3 <= 8'd0;
        wave0 <= 8'd0; wave1 <= 8'd0; wave2 <= 8'd0; wave3 <= 8'd0;
        l_lo  <= 8'd0; l_hi  <= 8'd0; r_lo  <= 8'd0;
    end else begin
        fifo_we <= 1'b0;   // single-cycle write pulse

        case (state)
            S_IDLE: begin
                sd_sec_read <= 1'b0;
                if (start) begin
                    if (!wav_usable) begin
                        state <= S_FAULT;
                    end else begin
                        pcm_total      <= wav_size - HDR_LEN;
                        pcm_cnt        <= 32'd0;
                        hdr_skip       <= HDR_SKIP;
                        hdr_cnt        <= 6'd0;
                        byte_phase     <= 2'd0;
                        sd_sec_read_addr <= wav_start_sector;
                        state          <= S_READ;
                    end
                end
            end

            S_READ: begin
                sd_sec_read <= 1'b1;

                if (sd_sec_read_data_valid) begin
                    if (hdr_skip != 6'd0) begin
                        // Header region: skip bytes, capture RIFF/WAVE magic.
                        case (hdr_cnt)
                            6'd0 : riff0 <= sd_sec_read_data;
                            6'd1 : riff1 <= sd_sec_read_data;
                            6'd2 : riff2 <= sd_sec_read_data;
                            6'd3 : riff3 <= sd_sec_read_data;
                            6'd8 : wave0 <= sd_sec_read_data;
                            6'd9 : wave1 <= sd_sec_read_data;
                            6'd10: wave2 <= sd_sec_read_data;
                            6'd11: wave3 <= sd_sec_read_data;
                            default: ;
                        endcase
                        hdr_cnt  <= hdr_cnt + 6'd1;
                        hdr_skip <= hdr_skip - 6'd1;
                        // Last header byte: verify magic once. A failed check is
                        // terminal so a wrong/garbage file never spins the SD bus.
                        if (hdr_skip == 6'd1 && !magic_done) begin
                            magic_done <= 1'b1;
                            if (!magic_ok) begin
                                sd_sec_read <= 1'b0;
                                state       <= S_FAULT;
                            end
                        end
                    end else if (pcm_cnt < pcm_total) begin
                        // PCM region: assemble 4 little-endian bytes into one
                        // {R,L} word and pulse fifo_we on the 4th byte.
                        case (byte_phase)
                            2'd0: begin l_lo     <= sd_sec_read_data; byte_phase <= 2'd1; end
                            2'd1: begin l_hi     <= sd_sec_read_data; byte_phase <= 2'd2; end
                            2'd2: begin r_lo     <= sd_sec_read_data; byte_phase <= 2'd3; end
                            2'd3: begin
                                fifo_di    <= {sd_sec_read_data, r_lo, l_hi, l_lo};
                                fifo_we    <= 1'b1;
                                byte_phase <= 2'd0;
                                pcm_cnt    <= pcm_cnt + 32'd4;
                            end
                        endcase
                    end
                    // else: past EOF inside the tail of the last sector; ignore.
                end

                if (sd_sec_read_end) begin
                    // Deassert for this cycle (the reader re-issues CMD17 for the
                    // same address if sd_sec_read is still high in its wait state)
                    // and advance, exactly like bmp_read's ST_LOAD_DATA.
                    sd_sec_read <= 1'b0;
                    if (pcm_cnt >= pcm_total) begin
                        // End of song: rewind for single-track loop. The header is
                        // re-skipped each pass; magic is only checked the first time.
                        sd_sec_read_addr <= wav_start_sector;
                        pcm_cnt          <= 32'd0;
                        hdr_skip         <= HDR_SKIP;
                        hdr_cnt          <= 6'd0;
                        byte_phase       <= 2'd0;
                        state            <= S_READ;
                    end else if (pause_now) begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                        state            <= S_WAIT;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                        state            <= S_READ;
                    end
                end
            end

            S_WAIT: begin
                sd_sec_read <= 1'b0;
                if (!pause_now)
                    state <= S_READ;
            end

            S_FAULT: begin
                sd_sec_read <= 1'b0;   // silent forever; no WAV / bad header
            end

            default: begin
                sd_sec_read <= 1'b0;
                state       <= S_IDLE;
            end
        endcase
    end
end

endmodule
