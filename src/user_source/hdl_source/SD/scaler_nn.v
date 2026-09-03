// ---------------------------------------------------------------------------
// scaler_nn.v
//
// Nearest-neighbour scaler placed on the SD-card WRITE side, between the
// bmp_read pixel stream and the frame write FIFO. The SDRAM frame buffers stay
// a fixed DST_W x DST_H, so nothing downstream of this module changes: the
// display video chain, frame_fifo_read and its rd_delay assumption on the
// encrypted SDRAM PHY are all left untouched, and the transition effects of
// stage 4 keep working against a fixed geometry.
//
// A bilinear upgrade only has to reproduce this port list; the line buffer
// becomes two source-row buffers and the Bresenham accumulators become
// interpolation weights.
//
// Why the source stream needs an elastic buffer
//   The destination cursor is the master, but the source stream is a slave that
//   can never be paused: bmp_read pushes a pixel whenever the SPI reader hands
//   it one, at a fixed 96 sd_card_clk cycles per pixel (SPI_HIGH_SPEED_DIV = 0
//   makes SCK sys_clk/4 = 25MHz, and a 24-bit pixel is three bytes). The
//   sequencer consumes source pixels only while it fills the line buffer, so
//   every destination row it emits without a fill is a window in which the
//   arrivals have to be parked. Two kinds of row do not fill:
//
//     - vertically repeated rows, up to MAX_UPSCALE - 1 of them in a row, and
//     - black border rows, up to off_y = (DST_H - th) / 2 of them at the top.
//
//   The border rows dominate, and they are why the first 256 entry buffer was
//   undersized. A run of N non-filling rows lasts N * 2 * DST_W cycles and
//   therefore parks N * 2 * DST_W / 96 pixels; the theoretical worst case
//   off_y = 239 on its own needs 239 * 1280 / 96 = 3187 entries, against the
//   40 pixels that the repeated rows alone would suggest. Letting the position
//   counter free-run instead of parking drops the first columns of the next
//   source row: the row is then filled starting from the column the stream had
//   already reached, which shifts the picture left and runs sx_target past
//   src_w at the right edge, where the defensive bail-out leaves a stripe of
//   the previous row visible.
//
//   SK_DEPTH is 4096, which covers that bound for every source geometry and so
//   does not silently depend on the SRC_W_MIN / SRC_H_MIN limits over in
//   bmp_read.v. Backpressure from i_fifo_usedw stretches the non-filling
//   windows and lets the backlog grow past the border bound; it is then capped
//   by the source pixel count, so the residual exposure is a very narrow source
//   (src_w near 64, where one fill consumes almost nothing) stretched over many
//   rows while the write FIFO is held near full.
//
// Measured occupancy, and why it is a non-issue in practice
//   th = min(src_h * MAX_UPSCALE, DST_H), so any source with src_h >= 120 gets
//   th = DST_H and therefore off_y = 0: no border rows at all, no emission gap,
//   no backlog. That covers the whole acceptance list (640x480, 800x600,
//   1920x1080, 320x240) and every real photograph. The exposed band is only
//   src_h in [64, 119], i.e. icon sized pictures.
//
//   Peak occupancy measured by tools/sim_scaler_nn.py pass B, at the real 96
//   cycles per pixel and the worst geometry in each row (64x64, off_y = 112):
//
//     emission rate              worst peak   margin   gated?
//     -------------------------- ------------ -------- -------
//     full rate, 2 cyc/px                1486   2.76x   yes
//     67% of cycles (2 of 3)             2281   1.80x   yes
//     33% of cycles (1 of 3)             2285   1.79x   yes
//     14% of cycles (1 of 7)             4096   1.00x   no, probe
//
//   The three gated rows are what the write path can actually sustain, and the
//   curve is flat between them because the backlog is set by the border run and
//   then self-balances. The 14% row is a deliberate probe past the sustainable
//   point: it needs about 50 MB/s of SDRAM write bandwidth on top of the 73.7
//   MB/s the display read side takes unconditionally, and 1920x64 overflows
//   there. The sim reports it but does not gate on it. Note that even at that
//   probe rate an off_y = 0 geometry peaks at 93 entries, 44x under the depth.
//
//   o_overflow latches if the bound is ever broken; it is a simulation and
//   bring-up hook, not a panel indicator.
//
// Stream-order isomorphism
//   bmp_read emits pixels in raw file order and frame_fifo_write applies
//   WRITE_V_FLIP to cancel the BMP bottom-up storage. This module maps source
//   stream row sy onto destination stream row dy with the same orientation, so
//   the existing flip logic stays correct for every source resolution.
//
// Backpressure
//   i_fifo_usedw is the write FIFO's wrusedw, which is only 9 bits for a
//   512-deep FIFO and therefore wraps to zero when the FIFO is completely
//   full. STALL_THRESH is placed far enough below the wrap point (384) that
//   the registered two-cycle reaction latency can only ever reach about 386
//   entries, so the ambiguous full condition is unreachable by construction.
// ---------------------------------------------------------------------------

module scaler_nn #(
    parameter integer DST_W        = 640,
    parameter integer DST_H        = 480,
    parameter integer MAX_UPSCALE  = 4,     // must stay a power of two
    parameter integer LB_AW        = 10,    // 1024 entries, power of two so TD infers BSRAM
    parameter integer SK_AW        = 12,    // 4096 parked source pixels, see the sizing note above
    parameter integer STALL_THRESH = 384    // write FIFO is 512 deep
)(
    input                      clk,
    input                      rst,

    // Source pixel stream, same clock domain, straight from bmp_read.
    input                      i_src_valid,
    input      [23:0]          i_src_pixel,

    // Arm pulse: source geometry is valid, start emitting one destination frame.
    input                      i_dim_valid,
    input      [15:0]          i_src_w,
    input      [15:0]          i_src_h,

    // Write FIFO occupancy, used for backpressure.
    input      [8:0]           i_fifo_usedw,

    // Destination pixel stream: exactly DST_W * DST_H pixels per frame.
    output reg                 o_dst_valid,
    output reg [23:0]          o_dst_pixel,
    output reg                 o_busy,
    output reg                 o_done,

    // Sticky per image: the elastic buffer overflowed, so the picture is wrong.
    output reg                 o_overflow
);

localparam integer LB_DEPTH = (1 << LB_AW);
localparam integer SK_DEPTH = (1 << SK_AW);

localparam [9:0]  DX_LAST = DST_W[9:0]  - 10'd1;
localparam [9:0]  DY_LAST = DST_H[9:0]  - 10'd1;
localparam [8:0]  STALL_AT = STALL_THRESH[8:0];

// Sized copy so the full comparison below does not bit-select a parameter.
localparam [SK_AW:0] SK_COUNT_MAX = SK_DEPTH;

// Width of the saturating S_FILL_W progress watchdog, in sd_card_clk cycles.
//
// This was 16 (32768 cycles = 327.68us) and that was wrong by two to three
// orders of magnitude. The wait it has to ride out is not set by the pixel
// arrival period, it is set by how long the SD reader goes silent between two
// single block reads: sd_card_sec_read_write re-issues CMD17 for every sector
// and then waits for the card's start token, and the SPI layer has no timeout
// of its own (its `timer` register is declared and never used), so the bound
// is the card's, not ours. The SD physical spec allows a read access time on
// the order of 100ms. A 327.68us threshold therefore fires on an entirely
// healthy card, once per sector gap, i.e. several times per destination row.
//
// What firing costs: the bail-out below declares the row complete with only
// `dx` of `tw` line buffer entries written, and line_buf is never cleared, so
// the drain then emits BSRAM that was never written. TD warns about exactly
// this (SYN-6562, the BSRAM init value is dropped because gate eram_init is
// off), and uninitialised BSRAM returns an arbitrary value that is FIXED per
// address. Every row reads the same addresses, so the result is a field of
// vertical colour stripes that is identical on every row and starts at a fixed
// column -- which is precisely the corruption seen on the panel.
//
// tools/sim_scaler_nn.py measured a 281 cycle peak against the old threshold
// and reported 117x of margin. That number was meaningless: the sim drove the
// source as a pulse exactly every 96 cycles with no gap at all, so it could
// not produce a silence longer than 96 cycles. Pass F now models the byte
// level arrival including the per-sector gap.
//
// 27 bits gives a threshold of 2^26 = 67.1M cycles = 671.1ms. The silence to
// ride out is no longer one card read access time, it is the whole retry budget
// sd_card_sec_read_write can spend on a single sector: RD_RETRY_MAX = 2 means 3
// attempts, each able to burn READ_TIMEOUT_MAX = 100ms in sd_card_cmd's
// S_READ_WAIT before it reports cmd_req_error, with an S_RETRY_GAP of
// 2^13 = 81.92us between them -- 300.4ms worst case. That budget was sized
// against sd_card_bmp's load_stall_cnt and never against this counter, which
// watches the same silence through a window 1.79x tighter. At the previous 25
// bits (167.8ms) any sector needing two attempts stalls the fill past the
// threshold, and the bail-out below then truncates that row. It truncates once
// per stalled row, so rows carrying the picture interleave with black ones: a
// HORIZONTAL line texture on the panel, which is what distinguishes this failure
// from the vertical stripes the old 16 bit value produced.
//
// Both sides of this parameter are bounded, and 27 is the top of the legal
// range. It must exceed the 30.04M cycle retry budget, and it must stay inside
// the one second (100M cycle) load_stall_cnt in sd_card_bmp, which remains the
// authority on a genuinely dead source and is fed by bmp_data_wr_en and
// scaler_dst_valid. 2^25 = 33.6M and 2^26 = 67.1M satisfy both; 2^27 = 134.2M
// does not, because the load would then be abandoned before the scaler ever
// bailed out. 27 leaves 2.23x over the retry budget and 1.49x under
// load_stall_cnt. A saturated counter bit is still a single LUT input instead
// of a wide equality compare.
localparam integer FILL_WAIT_AW = 27;

localparam [3:0] S_IDLE     = 4'd0;
localparam [3:0] S_ROW      = 4'd1;
localparam [3:0] S_SKIP     = 4'd2;
localparam [3:0] S_FILL_W   = 4'd3;
localparam [3:0] S_FILL_E   = 4'd4;
localparam [3:0] S_FILL_SUB = 4'd5;
localparam [3:0] S_YSTEP    = 4'd6;
localparam [3:0] S_YSUB     = 4'd7;
localparam [3:0] S_DRAIN_RD = 4'd8;
localparam [3:0] S_DRAIN_EM = 4'd9;
localparam [3:0] S_DONE     = 4'd10;
localparam [3:0] S_FILL_C   = 4'd11;    // absorbs the elastic buffer read latency

reg [3:0]  state;

// Geometry, quasi-static for the whole frame.
reg [15:0] src_w;
reg [15:0] src_h;
reg [9:0]  tw;              // width  of the active destination region
reg [9:0]  th;              // height of the active destination region
reg [9:0]  off_x;           // left black border width
reg [9:0]  off_y;           // top  black border height

// Destination cursors.
reg [9:0]  dy;
reg [9:0]  dx_out;
reg        row_is_border;

// Bresenham state.
reg [9:0]  dx;              // line buffer fill cursor
reg [15:0] sy_target;       // source row  needed by the current destination row
reg [15:0] sx_target;       // source column needed by the current fill cursor
reg [15:0] x_acc;
reg [15:0] y_acc;

// Line buffer bookkeeping.
reg [15:0] filled_sy;       // source row currently stored in the line buffer
reg        lb_valid;
reg [23:0] held_pixel;      // source pixel captured for sx_target
reg [15:0] held_sx;
reg        held_valid;

// ---------------------------------------------------------------------------
// Elastic buffer for the source stream, plus the raster position of its head.
// The buffer stores pixels only; the position is a plain counter that advances
// on pop, which keeps the head coordinates out of the memory and available for
// the combinational decisions below.
// ---------------------------------------------------------------------------
reg  [23:0]      skid [0:SK_DEPTH-1];
reg  [SK_AW-1:0] sk_wr;
reg  [SK_AW-1:0] sk_rd;
reg  [SK_AW:0]   sk_count;
reg  [23:0]      sk_rdata;
reg  [15:0]      head_px_idx;
reg  [15:0]      head_row_idx;

reg [8:0]  usedw_r;
reg [FILL_WAIT_AW-1:0] fill_wait;

// ---------------------------------------------------------------------------
// Geometry derived from the arm-pulse inputs. MAX_UPSCALE is a power of two so
// the multiply is a plain shift, and clamping it is what bounds the Bresenham
// carry loops below to three iterations each.
// ---------------------------------------------------------------------------
wire [15:0] w_up     = {i_src_w[15 - 2:0], 2'b00};      // i_src_w * MAX_UPSCALE
wire [15:0] h_up     = {i_src_h[15 - 2:0], 2'b00};      // i_src_h * MAX_UPSCALE
wire [9:0]  tw_new   = (w_up < DST_W) ? w_up[9:0] : DST_W[9:0];
wire [9:0]  th_new   = (h_up < DST_H) ? h_up[9:0] : DST_H[9:0];
wire [9:0]  off_x_new = (DST_W[9:0] - tw_new) >> 1;
wire [9:0]  off_y_new = (DST_H[9:0] - th_new) >> 1;

wire        stall          = (usedw_r >= STALL_AT);
wire [9:0]  active_end_y   = off_y + th;    // first border row at the bottom

// First border column on the right of THE CURRENT ROW. This is a register
// rather than the combinational off_x + tw it replaces, because a fill
// bail-out shortens the row: only the first `dx` line buffer entries were
// written, and everything past them must be forced black. Reusing the compare
// that dx_out_active already performs costs no extra logic in the emission
// path -- it actually takes the off_x + tw adder out of it -- and it is what
// guarantees the drain can never emit an uninitialised line_buf entry.
//
// It is written only where a row's fill extent becomes known, so a vertically
// repeated row inherits the extent of the row it repeats and a border row is
// unaffected (row_is_border already forces black there).
reg  [9:0]  row_end_x;
wire        dy_active      = (dy     >= off_y) && (dy     < active_end_y);
wire        dx_out_active  = (dx_out >= off_x) && (dx_out < row_end_x);

// ---------------------------------------------------------------------------
// Pop / push decoding. Discarding is one pixel per cycle, which is far faster
// than the 96-cycle arrival period, so the head can never fall behind even
// while a horizontal downscale throws away two of every three source columns.
// ---------------------------------------------------------------------------
wire sk_empty = (sk_count == 0);
wire sk_full  = (sk_count == SK_COUNT_MAX);
wire sk_push  = i_src_valid && !sk_full;

// The pixel wanted by sx_target is already parked from an earlier column.
wire fill_reuse = (state == S_FILL_W) && held_valid && (held_sx == sx_target);

// Progress watchdog for the fill. sx_target is floor(k * src_w / tw) with
// k <= tw - 1, so it never passes src_w - 1 and the wanted pixel always exists
// in the row the head is on: the only way S_FILL_W stops advancing is the
// source stream itself dying, which a truncated pixel region can do. Draining
// the partially filled row then beats locking up the write channel forever.
//
// This replaced a combinational (head_row_idx > sy_target) bail-out. That test
// was dead code by the same sx_target bound, but its 16-bit magnitude compare
// and the sk_empty decode sat in the clock-enable cone of dx_out, which is
// cleared on the bail branch, and together they cost 0.75ns of sd_card_clk
// slack. A saturated counter bit is a single LUT input instead.
wire fill_stuck = (state == S_FILL_W) && fill_wait[FILL_WAIT_AW-1];

wire head_on_row = !sk_empty && (head_row_idx == sy_target);

wire pop_capture = (state == S_FILL_W) && !fill_reuse && !fill_stuck &&
                   head_on_row && (head_px_idx >= sx_target);

wire pop_discard = !sk_empty && !pop_capture &&
                   ((state == S_SKIP) || (state == S_FILL_W)) &&
                   ((head_row_idx < sy_target) ||
                    (head_on_row && (head_px_idx < sx_target)));

wire sk_pop = pop_capture || pop_discard;

// ---------------------------------------------------------------------------
// Line buffer: one destination row, simple dual port so TD maps it to BSRAM.
// Depth is a power of two on purpose; a 640-entry array would risk falling
// back to distributed RAM.
//
// Neither this array nor the elastic buffer below is initialised, and TD warns
// (SYN-6562) that the BSRAM init value is dropped because gate eram_init is
// off. That is safe here: line_buf is only read while lb_valid says the row
// being drained was written, and skid is only read at sk_rd, which the pointer
// block zeroes together with sk_count, so no read can reach an entry that was
// not written first.
// ---------------------------------------------------------------------------
wire [LB_AW-1:0] lb_raddr = dx_out_active ? (dx_out - off_x) : {LB_AW{1'b0}};
wire             lb_we    = (state == S_FILL_E);
wire             lb_re    = (state == S_DRAIN_RD);

reg [23:0] line_buf [0:LB_DEPTH-1];
reg [23:0] lb_rdata;

always @(posedge clk) begin
    if (lb_we) line_buf[dx[LB_AW-1:0]] <= held_pixel;
end

always @(posedge clk) begin
    if (lb_re) lb_rdata <= line_buf[lb_raddr];
end

// Elastic buffer ports. Write and read are separate blocks so TD infers one
// simple dual port BSRAM; the read address is sampled before sk_rd increments,
// which is what makes the one-cycle S_FILL_C state sufficient.
always @(posedge clk) begin
    if (sk_push) skid[sk_wr] <= i_src_pixel;
end

always @(posedge clk) begin
    if (pop_capture) sk_rdata <= skid[sk_rd];
end

// ---------------------------------------------------------------------------
// Elastic buffer pointers and head position. Cleared on every arm pulse so
// leftovers from the previous image can never be mistaken for new pixels; the
// arm pulse itself comes from bmp_read at the end of ST_LOAD_HDR, a whole SD
// command ahead of the first pixel of ST_LOAD_DATA, so nothing in flight is
// lost by the clear.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        sk_wr        <= {SK_AW{1'b0}};
        sk_rd        <= {SK_AW{1'b0}};
        sk_count     <= {(SK_AW+1){1'b0}};
        head_px_idx  <= 16'd0;
        head_row_idx <= 16'd0;
        o_overflow   <= 1'b0;
    end else if (i_dim_valid) begin
        sk_wr        <= {SK_AW{1'b0}};
        sk_rd        <= {SK_AW{1'b0}};
        sk_count     <= {(SK_AW+1){1'b0}};
        head_px_idx  <= 16'd0;
        head_row_idx <= 16'd0;
        o_overflow   <= 1'b0;
    end else begin
        if (sk_push) sk_wr <= sk_wr + {{(SK_AW-1){1'b0}}, 1'b1};
        if (sk_pop)  sk_rd <= sk_rd + {{(SK_AW-1){1'b0}}, 1'b1};

        case ({sk_push, sk_pop})
            2'b10:   sk_count <= sk_count + {{SK_AW{1'b0}}, 1'b1};
            2'b01:   sk_count <= sk_count - {{SK_AW{1'b0}}, 1'b1};
            default: ;
        endcase

        if (sk_pop) begin
            if ((head_px_idx + 16'd1) >= src_w) begin
                head_px_idx  <= 16'd0;
                head_row_idx <= head_row_idx + 16'd1;
            end else begin
                head_px_idx  <= head_px_idx + 16'd1;
            end
        end

        if (i_src_valid && sk_full) o_overflow <= 1'b1;
    end
end

// ---------------------------------------------------------------------------
// Main sequencer. The destination cursor is the master; the elastic buffer is
// consumed only as far as the current destination row requires.
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state         <= S_IDLE;
        o_dst_valid   <= 1'b0;
        o_dst_pixel   <= 24'd0;
        o_busy        <= 1'b0;
        o_done        <= 1'b0;
        src_w         <= 16'd0;
        src_h         <= 16'd0;
        tw            <= 10'd0;
        th            <= 10'd0;
        off_x         <= 10'd0;
        off_y         <= 10'd0;
        dy            <= 10'd0;
        dx_out        <= 10'd0;
        dx            <= 10'd0;
        row_end_x     <= 10'd0;
        row_is_border <= 1'b0;
        sy_target     <= 16'd0;
        sx_target     <= 16'd0;
        x_acc         <= 16'd0;
        y_acc         <= 16'd0;
        filled_sy     <= 16'd0;
        lb_valid      <= 1'b0;
        held_pixel    <= 24'd0;
        held_sx       <= 16'd0;
        held_valid    <= 1'b0;
        usedw_r       <= 9'd0;
        fill_wait     <= {FILL_WAIT_AW{1'b0}};
    end else begin
        usedw_r     <= i_fifo_usedw;
        o_dst_valid <= 1'b0;    // every emission is a single-cycle pulse
        o_done      <= 1'b0;

        // Saturating, so the top bit stays a one-input "waited too long" test
        // instead of a wide equality compare.
        if ((state != S_FILL_W) || pop_capture || fill_reuse || fill_stuck)
            fill_wait <= {FILL_WAIT_AW{1'b0}};
        else if (!fill_wait[FILL_WAIT_AW-1])
            fill_wait <= fill_wait + {{(FILL_WAIT_AW-1){1'b0}}, 1'b1};

        if (i_dim_valid) begin
            // Re-arm for a new image.
            src_w        <= i_src_w;
            src_h        <= i_src_h;
            tw           <= tw_new;
            th           <= th_new;
            off_x        <= off_x_new;
            off_y        <= off_y_new;
            row_end_x    <= off_x_new + tw_new;
            dy           <= 10'd0;
            dx_out       <= 10'd0;
            dx           <= 10'd0;
            sy_target    <= 16'd0;
            sx_target    <= 16'd0;
            x_acc        <= 16'd0;
            y_acc        <= 16'd0;
            filled_sy    <= 16'd0;
            lb_valid     <= 1'b0;
            held_pixel   <= 24'd0;
            held_sx      <= 16'd0;
            held_valid   <= 1'b0;
            fill_wait    <= {FILL_WAIT_AW{1'b0}};
            o_busy       <= 1'b1;
            state        <= S_ROW;
        end else begin
            case (state)

            S_IDLE: ;   // waiting for i_dim_valid

            // ---------------------------------------------------------------
            // Classify the current destination row and pick where its pixels
            // come from: black, the row already in the line buffer, or a fill.
            // ---------------------------------------------------------------
            S_ROW: begin
                dx         <= 10'd0;
                dx_out     <= 10'd0;
                sx_target  <= 16'd0;
                x_acc      <= 16'd0;
                held_valid <= 1'b0;
                if (!dy_active) begin
                    row_is_border <= 1'b1;
                    state         <= S_DRAIN_RD;
                end else begin
                    row_is_border <= 1'b0;
                    if (lb_valid && (filled_sy == sy_target))
                        state <= S_DRAIN_RD;    // vertical upscale: repeat this row
                    else if (!sk_empty && (head_row_idx >= sy_target))
                        state <= S_FILL_W;      // the needed row is parked already
                    else
                        state <= S_SKIP;        // discard rows until it arrives
                end
            end

            // Vertical downscale: pop and discard whole source rows.
            S_SKIP: begin
                if (!sk_empty && (head_row_idx >= sy_target))
                    state <= S_FILL_W;
            end

            // ---------------------------------------------------------------
            // Wait until the source pixel wanted by sx_target reaches the head
            // of the elastic buffer. On upscale the same pixel feeds several
            // destination columns, so it is captured once and reused instead of
            // being popped again.
            // ---------------------------------------------------------------
            S_FILL_W: begin
                if (fill_reuse) begin
                    state <= S_FILL_E;
                end else if (fill_stuck) begin
                    // Partial row. Shorten the active region to what was really
                    // filled so the drain paints the rest black instead of
                    // reading line_buf entries nobody wrote.
                    lb_valid  <= 1'b1;
                    filled_sy <= sy_target;
                    row_end_x <= off_x + dx;
                    dx_out    <= 10'd0;
                    state     <= S_DRAIN_RD;
                end else if (pop_capture) begin
                    // pop_capture issued the buffer read; the pixel lands in
                    // sk_rdata next cycle, so held_sx is committed now and the
                    // data one state later.
                    held_sx    <= sx_target;
                    held_valid <= 1'b1;
                    state      <= S_FILL_C;
                end
                // Otherwise pop_discard is walking the head forward, or the
                // wanted pixel simply has not arrived yet.
            end

            S_FILL_C: begin
                held_pixel <= sk_rdata;
                state      <= S_FILL_E;
            end

            // Write one line buffer entry. The BRAM write itself is in the
            // dedicated always block above, gated by lb_we.
            S_FILL_E: begin
                dx    <= dx + 10'd1;
                x_acc <= x_acc + src_w;
                state <= S_FILL_SUB;
            end

            // Resolve the horizontal accumulator carries. Bounded by
            // ceil(src_w / tw) <= ceil(1920 / 640) = 3 iterations because tw is
            // clamped to at least src_w / MAX_UPSCALE... in the downscale
            // direction the clamp does not apply, but src_w itself is capped at
            // 1920 while tw is then always DST_W = 640.
            S_FILL_SUB: begin
                if (x_acc >= tw) begin
                    x_acc     <= x_acc - tw;
                    sx_target <= sx_target + 16'd1;
                end else if (dx >= tw) begin
                    lb_valid  <= 1'b1;
                    filled_sy <= sy_target;
                    row_end_x <= off_x + tw;
                    dx_out    <= 10'd0;
                    state     <= S_DRAIN_RD;
                end else begin
                    state <= S_FILL_W;
                end
            end

            // ---------------------------------------------------------------
            // Emit the destination row, two cycles per pixel: one to issue the
            // line buffer read, one to drive the output. Border columns and
            // border rows are forced to black here, which is what centres an
            // image whose upscale was clamped by MAX_UPSCALE.
            // ---------------------------------------------------------------
            S_DRAIN_RD: begin
                state <= S_DRAIN_EM;    // lb_rdata is captured on this edge
            end

            S_DRAIN_EM: begin
                if (!stall) begin
                    o_dst_valid <= 1'b1;
                    o_dst_pixel <= (row_is_border || !dx_out_active) ? 24'd0 : lb_rdata;
                    if (dx_out >= DX_LAST) begin
                        if (dy >= DY_LAST)
                            state <= S_DONE;
                        else begin
                            dy    <= dy + 10'd1;
                            state <= S_YSTEP;
                        end
                    end else begin
                        dx_out <= dx_out + 10'd1;
                        state  <= S_DRAIN_RD;
                    end
                end
            end

            // Advance the vertical accumulator, but only when the row just
            // emitted and the row about to be emitted are both inside the
            // active region. dy is already the new row here while
            // row_is_border still describes the previous one.
            S_YSTEP: begin
                if (!row_is_border && (dy < active_end_y)) begin
                    y_acc <= y_acc + src_h;
                    state <= S_YSUB;
                end else begin
                    state <= S_ROW;
                end
            end

            // Bounded by ceil(src_h / th) <= ceil(1080 / 480) = 3 iterations.
            S_YSUB: begin
                if (y_acc >= th) begin
                    y_acc     <= y_acc - th;
                    sy_target <= sy_target + 16'd1;
                end else begin
                    state <= S_ROW;
                end
            end

            S_DONE: begin
                o_done <= 1'b1;
                o_busy <= 1'b0;
                state  <= S_IDLE;
            end

            default: state <= S_IDLE;

            endcase
        end
    end
end

endmodule
