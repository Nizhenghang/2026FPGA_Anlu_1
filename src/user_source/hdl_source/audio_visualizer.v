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

// ==========================================================================
// 16-band log-spaced spectrum analyser.
//
// The bank, its fixed-point formats and the panel geometry are not hand-derived
// here: tools/gen_biquad_coeffs.py picks the coefficient format and proves the
// residual DC offset, tools/sim_biquad_bank.py walks the MAC finite state
// machine cycle by cycle, and tools/render_spectrum_preview.py draws the panel
// and measures it back. This file transcribes both.
// ==========================================================================

localparam NBAND       = 16;
localparam COEFF_W     = 18;      // signed Q1.16, chosen by gen_biquad_coeffs.py
localparam COEFF_FRAC  = 16;
localparam STATE_BITS  = 24;      // 16-bit output plus STATE_EXTRA of fraction
localparam STATE_EXTRA = 8;       // see "quantisation DC offset" below
localparam ACC_BITS    = 44;
localparam WIN_BITS    = 8;       // envelope window: 256 internal samples

// --------------------------------------------------------------------------
// Panel geometry. Mirrors render_spectrum_preview.py exactly.
// --------------------------------------------------------------------------
localparam PANEL_X = 28;
localparam PANEL_Y = 352;
localparam PANEL_W = 584;
localparam PANEL_H = 104;
localparam PANEL_X_LAST = PANEL_X + PANEL_W - 1;      // 611
localparam PANEL_Y_LAST = PANEL_Y + PANEL_H - 1;      // 455

localparam BAR_X = 44;
localparam BAR_Y = 362;
localparam BAR_W = 512;
localparam BAR_H = 64;
localparam BAR_X_LAST = BAR_X + BAR_W - 1;            // 555
localparam BAR_Y_LAST = BAR_Y + BAR_H - 1;            // 425

// Cell width is a power of two so bar_idx is a shift and not a divide. The bar
// width is the single knob; both offsets derive from it, so widening the bar
// eats its own gap rather than creeping into the next band.
localparam BAR_CELL   = 32;
localparam BAR_PX     = 24;
localparam BAR_OFF_LO = (BAR_CELL - BAR_PX) / 2;      // 4
localparam BAR_OFF_HI = BAR_OFF_LO + BAR_PX - 1;      // 27

localparam LEFT_X      = PANEL_X + 1;                 // 29
localparam LEFT_W      = BAR_X - PANEL_X - 1;         // 15
localparam LEFT_X_LAST = LEFT_X + LEFT_W - 1;         // 43
localparam RIGHT_X     = BAR_X + BAR_W;               // 556
localparam RIGHT_W     = PANEL_X_LAST - RIGHT_X;      // 55
localparam RIGHT_X_LAST = RIGHT_X + RIGHT_W - 1;      // 610

localparam METER_X = BAR_X;                           // 44
localparam METER_W = BAR_W;                           // 512
localparam METER_X_LAST = METER_X + METER_W - 1;      // 555
localparam METER_L_Y      = 428;
localparam METER_H        = 12;
localparam METER_L_Y_LAST = METER_L_Y + METER_H - 1;  // 439
localparam METER_R_Y      = 441;
localparam METER_R_Y_LAST = METER_R_Y + METER_H - 1;  // 452

localparam TICK_W = 4;

// A dB tick must sit on the topmost row a bar of that height actually lights.
// bar_pixel is `rel_y < bar_h`, so a bar of height h occupies rel_y 0..h-1 and
// its top row is BAR_Y_LAST - h + 1; dropping the +1 puts the tick one pixel
// above the bar it labels.
localparam DB6_ENV  = 64;                             // round(127 * 10^(-6/20))
localparam DB12_ENV = 32;
localparam DB24_ENV = 8;
localparam DB6_Y    = BAR_Y_LAST - (DB6_ENV  >> 1) + 1;   // 394
localparam DB12_Y   = BAR_Y_LAST - (DB12_ENV >> 1) + 1;   // 410
localparam DB24_Y   = BAR_Y_LAST - (DB24_ENV >> 1) + 1;   // 422

localparam [9:0] H_ACTIVE_W       = H_ACTIVE;
localparam [9:0] V_ACTIVE_W       = V_ACTIVE;
localparam [9:0] PANEL_X_W        = PANEL_X;
localparam [9:0] PANEL_Y_W        = PANEL_Y;
localparam [9:0] PANEL_X_LAST_W   = PANEL_X_LAST;
localparam [9:0] PANEL_Y_LAST_W   = PANEL_Y_LAST;
localparam [9:0] BAR_X_W          = BAR_X;
localparam [9:0] BAR_Y_W          = BAR_Y;
localparam [9:0] BAR_X_LAST_W     = BAR_X_LAST;
localparam [9:0] BAR_Y_LAST_W     = BAR_Y_LAST;
localparam [9:0] LEFT_X_W         = LEFT_X;
localparam [9:0] LEFT_X_LAST_W    = LEFT_X_LAST;
localparam [9:0] RIGHT_X_LAST_W   = RIGHT_X_LAST;
localparam [9:0] METER_X_W        = METER_X;
localparam [9:0] METER_X_LAST_W   = METER_X_LAST;
localparam [9:0] METER_L_Y_W      = METER_L_Y;
localparam [9:0] METER_L_Y_LAST_W = METER_L_Y_LAST;
localparam [9:0] METER_R_Y_W      = METER_R_Y;
localparam [9:0] METER_R_Y_LAST_W = METER_R_Y_LAST;
localparam [9:0] DB6_Y_W          = DB6_Y;
localparam [9:0] DB12_Y_W         = DB12_Y;
localparam [9:0] DB24_Y_W         = DB24_Y;
localparam [9:0] TICK_LEFT_HI_W   = LEFT_X + TICK_W - 1;          // 32
localparam [9:0] TICK_RIGHT_LO_W  = RIGHT_X_LAST - TICK_W + 1;    // 607

// --------------------------------------------------------------------------
// Coefficient ROM, generated by tools/gen_biquad_coeffs.py -- do not edit.
//
// RBJ constant-0dB-peak bandpass, 100 Hz .. 7000 Hz, Q = 4.0, internal rate
// 24000 Hz. b1 == 0 and b2 == -b0 hold exactly, so
//   y[n] = B0*(x[n] - x[n-2]) + NA1*y[n-1] + NA2*y[n-2]
// and (x[n] - x[n-2]) is computed once and shared by all 16 bands.
// --------------------------------------------------------------------------
function signed [COEFF_W-1:0] band_b0;
    input [3:0] idx;
    begin
        case (idx)
            4'd0 : band_b0 = 18'sd214;        //   100.0 Hz
            4'd1 : band_b0 = 18'sd283;        //   132.7 Hz
            4'd2 : band_b0 = 18'sd376;        //   176.2 Hz
            4'd3 : band_b0 = 18'sd498;        //   233.9 Hz
            4'd4 : band_b0 = 18'sd658;        //   310.5 Hz
            4'd5 : band_b0 = 18'sd870;        //   412.1 Hz
            4'd6 : band_b0 = 18'sd1149;       //   547.1 Hz
            4'd7 : band_b0 = 18'sd1512;       //   726.2 Hz
            4'd8 : band_b0 = 18'sd1984;       //   963.9 Hz
            4'd9 : band_b0 = 18'sd2587;       //  1279.6 Hz
            4'd10: band_b0 = 18'sd3344;       //  1698.5 Hz
            4'd11: band_b0 = 18'sd4263;       //  2254.6 Hz
            4'd12: band_b0 = 18'sd5313;       //  2992.8 Hz
            4'd13: band_b0 = 18'sd6377;       //  3972.7 Hz
            4'd14: band_b0 = 18'sd7165;       //  5273.4 Hz
            4'd15: band_b0 = 18'sd7060;       //  7000.0 Hz
            default: band_b0 = 18'sd0;
        endcase
    end
endfunction

function signed [COEFF_W-1:0] band_na1;
    input [3:0] idx;
    begin
        case (idx)
            4'd0 : band_na1 = 18'sd130600;     //   100.0 Hz
            4'd1 : band_na1 = 18'sd130426;     //   132.7 Hz
            4'd2 : band_na1 = 18'sd130182;     //   176.2 Hz
            4'd3 : band_na1 = 18'sd129833;     //   233.9 Hz
            4'd4 : band_na1 = 18'sd129327;     //   310.5 Hz
            4'd5 : band_na1 = 18'sd128579;     //   412.1 Hz
            4'd6 : band_na1 = 18'sd127456;     //   547.1 Hz
            4'd7 : band_na1 = 18'sd125740;     //   726.2 Hz
            4'd8 : band_na1 = 18'sd123079;     //   963.9 Hz
            4'd9 : band_na1 = 18'sd118900;     //  1279.6 Hz
            4'd10: band_na1 = 18'sd112288;     //  1698.5 Hz
            4'd11: band_na1 = 18'sd101811;     //  2254.6 Hz
            4'd12: band_na1 = 18'sd85328;      //  2992.8 Hz
            4'd13: band_na1 = 18'sd59890;      //  3972.7 Hz
            4'd14: band_na1 = 18'sd22073;      //  5273.4 Hz
            4'd15: band_na1 = -18'sd30269;     //  7000.0 Hz
            default: band_na1 = 18'sd0;
        endcase
    end
endfunction

function signed [COEFF_W-1:0] band_na2;
    input [3:0] idx;
    begin
        case (idx)
            4'd0 : band_na2 = -18'sd65109;     //   100.0 Hz
            4'd1 : band_na2 = -18'sd64969;     //   132.7 Hz
            4'd2 : band_na2 = -18'sd64785;     //   176.2 Hz
            4'd3 : band_na2 = -18'sd64541;     //   233.9 Hz
            4'd4 : band_na2 = -18'sd64219;     //   310.5 Hz
            4'd5 : band_na2 = -18'sd63795;     //   412.1 Hz
            4'd6 : band_na2 = -18'sd63238;     //   547.1 Hz
            4'd7 : band_na2 = -18'sd62511;     //   726.2 Hz
            4'd8 : band_na2 = -18'sd61569;     //   963.9 Hz
            4'd9 : band_na2 = -18'sd60362;     //  1279.6 Hz
            4'd10: band_na2 = -18'sd58848;     //  1698.5 Hz
            4'd11: band_na2 = -18'sd57010;     //  2254.6 Hz
            4'd12: band_na2 = -18'sd54910;     //  2992.8 Hz
            4'd13: band_na2 = -18'sd52781;     //  3972.7 Hz
            4'd14: band_na2 = -18'sd51206;     //  5273.4 Hz
            4'd15: band_na2 = -18'sd51415;     //  7000.0 Hz
            default: band_na2 = 18'sd0;
        endcase
    end
endfunction

// Two-sided envelope tracker, called once per 256-sample window (93.75 Hz), not
// once per sample. Both branches need the minimum step of 1: without it the
// release stalls at any env below 2**4 and pins every band at 15 forever once
// the audio stops, leaving a 7 px bar standing in silence.
function [7:0] smooth;
    input [7:0]  env_in;
    input [15:0] target_in;
    reg   [15:0] env_x;
    reg   [15:0] step;
    reg   [15:0] sum;
    begin
        env_x = {8'd0, env_in};
        if (target_in > env_x) begin
            step = (target_in - env_x) >> 1;              // attack, tau = 2 windows
            if (step == 16'd0) step = 16'd1;
            sum  = env_x + step;
            smooth = (sum > 16'd127) ? 8'd127 : sum[7:0];
        end else if (target_in < env_x) begin
            step = env_x >> 4;                            // release, tau = 16 windows
            if (step == 16'd0) step = 16'd1;
            smooth = (env_x > step) ? (env_x - step) : 16'd0;
        end else begin
            smooth = env_in;
        end
    end
endfunction

// --------------------------------------------------------------------------
// MAC finite state machine.
// --------------------------------------------------------------------------
localparam ST_IDLE    = 3'd0;
localparam ST_LAUNCH0 = 3'd1;
localparam ST_LAUNCH1 = 3'd2;
localparam ST_LAUNCH2 = 3'd3;
localparam ST_ROUND   = 3'd4;
localparam ST_WRITE   = 3'd5;

reg [2:0]  mac_state;
reg [3:0]  band_cnt;
reg signed [COEFF_W-1:0]  ma;
reg signed [24:0]         mb;
reg signed [ACC_BITS-1:0] prod;
reg signed [ACC_BITS-1:0] acc;

reg               decim;
reg signed [15:0] mix16_prev;
reg signed [15:0] xin_lat;
reg signed [24:0] dx8_lat;
reg signed [15:0] x_n1;
reg signed [15:0] x_n2;

reg signed [STATE_BITS-1:0] y_n1 [0:NBAND-1];
reg signed [STATE_BITS-1:0] y_n2 [0:NBAND-1];

reg [WIN_BITS-1:0] win_cnt;
reg [15:0] pk   [0:NBAND-1];
reg [7:0]  env  [0:NBAND-1];
reg [7:0]  peak [0:NBAND-1];
reg [2:0]  peak_vel [0:NBAND-1];
reg [7:0]  frame_cnt;

reg [7:0] pk_l;
reg [7:0] pk_r;
reg [7:0] lvl_l;
reg [7:0] lvl_r;

reg [9:0] x_pos;
reg [9:0] y_pos;
reg       de_d;

integer i;

// --------------------------------------------------------------------------
// Input scaling.
//
// I_audio_left/right are 24-bit signed. Taking the top 16 bits and summing puts
// the -12.5 dBFS test tone at +/-15625, which lands the envelope at 61 of 127
// and the bar at 30 of 63 px -- half scale, with headroom to climb. Full-scale
// audio saturates gracefully at the sum.
//
// The bit-select I_audio_left[23:8] is UNSIGNED in Verilog, so the sign bit has
// to be re-attached explicitly before the signed add or the negative half-cycle
// turns into a large positive number.
// --------------------------------------------------------------------------
wire signed [16:0] l_hi    = {I_audio_left[23],  I_audio_left[23:8]};
wire signed [16:0] r_hi    = {I_audio_right[23], I_audio_right[23:8]};
wire signed [17:0] mix_sum = l_hi + r_hi;
wire signed [15:0] mix16_w = (mix_sum > 18'sd32767)  ? 16'sd32767  :
                             (mix_sum < -18'sd32768) ? -16'sd32768 : mix_sum[15:0];

// 48 kHz -> 24 kHz by averaging two taps. Averaging cannot overflow, so this
// needs no saturator of its own. No anti-alias filter: the square wave's
// harmonics above 12 kHz are already -46 dBFS, which rounds to 0 on an 8-bit
// envelope.
wire signed [16:0] mix_pair = {mix16_w[15], mix16_w} +
                              {mix16_prev[15], mix16_prev};
wire signed [15:0] xin_w    = mix_pair >>> 1;

// The numerator term, shared by all 16 bands and computed once per sample.
wire signed [16:0] dx_w  = {xin_w[15], xin_w} - {x_n2[15], x_n2};
wire signed [24:0] dx8_w = {dx_w, 8'b00};

// L/R meters read the raw 24-bit channels: 8388607 >> 16 == 127, the same 0..127
// range the band envelopes use. Only the single value 128 needs clamping, and it
// arises only from the most-negative 24-bit sample.
wire [23:0] abs_l  = I_audio_left[23]  ? (~I_audio_left  + 24'd1) : I_audio_left;
wire [23:0] abs_r  = I_audio_right[23] ? (~I_audio_right + 24'd1) : I_audio_right;
wire [7:0]  abs_l8 = (abs_l[23:16] == 8'd128) ? 8'd127 : abs_l[23:16];
wire [7:0]  abs_r8 = (abs_r[23:16] == 8'd128) ? 8'd127 : abs_r[23:16];

// --------------------------------------------------------------------------
// Write-back path.
//
// The accumulator cannot overflow and needs no saturator of its own: |ma| is at
// most 2**17 and |mb| at most 2**24, so each product is bounded by 2**41 and the
// sum of three by 3*2**41 = 6.6e12, inside the 44-bit signed range of 8.8e12.
// Rounding is applied once, after the last term -- rounding each term separately
// would inject three biases per sample instead of one and bring back the DC
// offset that STATE_EXTRA exists to suppress.
//
// STATE_EXTRA is the reason the state is 24 bits rather than 16. Requantising y
// every sample injects a half-LSB bias at a summing point whose DC gain is
// roughly (fs / 2*pi*f0)**2, independent of Q. At 100 Hz that is 1458x; the
// extra 8 bits of fraction divide it by 256, leaving 3 LSB of residual against
// the 256 needed to lift a single pixel of bar.
// --------------------------------------------------------------------------
wire signed [ACC_BITS-1:0]  acc_final = acc + prod + 44'sd32768;
wire signed [ACC_BITS-1:0]  y_pre     = acc_final >>> COEFF_FRAC;
wire signed [STATE_BITS-1:0] y_new    = (y_pre > 44'sd8388607)   ? 24'sd8388607  :
                                       (y_pre < -44'sd8388608)  ? -24'sd8388608 :
                                                                  y_pre[23:0];
wire [23:0] y_abs  = y_new[23] ? (~y_new + 24'd1) : y_new;

// TWO shifts, not one, and getting this wrong is silent. y_new is the 24-bit Q8
// filter STATE, so y_abs[23:8] is the 16-bit filter output and y_abs[23:16] is
// that output scaled into the envelope's 0..127 range. Taking y_abs[23:8]
// instead leaves the target 256x too big: any band whose output exceeds 127
// (-48 dBFS, i.e. essentially any audible band) then pins env at 127, bar_h at
// 63, and the whole analyser becomes a static full-height colour wall while the
// L/R meters -- which read the raw channels and never touch this path -- keep
// working normally. The state's Q8 scaling accounts for the first shift; the
// envelope's own >>8 is the second.
wire [15:0] target = {8'd0, y_abs[23:16]};

wire       run_last = (band_cnt == 4'd15);
wire       win_tick = (win_cnt == {WIN_BITS{1'b1}});
wire [7:0] frame_cnt_nxt = frame_cnt + 8'd1;

wire [15:0] pk_target = (pk[band_cnt] > target) ? pk[band_cnt] : target;

// --------------------------------------------------------------------------
// Analysis: runs on I_audio_valid and I_frame_start, independent of the raster.
// --------------------------------------------------------------------------
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        decim      <= 1'b0;
        mix16_prev <= 16'sd0;
        xin_lat    <= 16'sd0;
        dx8_lat    <= 25'sd0;
        x_n1       <= 16'sd0;
        x_n2       <= 16'sd0;
        mac_state  <= ST_IDLE;
        band_cnt   <= 4'd0;
        ma         <= {COEFF_W{1'b0}};
        mb         <= 25'sd0;
        prod       <= {ACC_BITS{1'b0}};
        acc        <= {ACC_BITS{1'b0}};
        win_cnt    <= {WIN_BITS{1'b0}};
        frame_cnt  <= 8'd0;
        pk_l       <= 8'd0;
        pk_r       <= 8'd0;
        lvl_l      <= 8'd0;
        lvl_r      <= 8'd0;
        for (i = 0; i < NBAND; i = i + 1) begin
            y_n1[i]      <= {STATE_BITS{1'b0}};
            y_n2[i]      <= {STATE_BITS{1'b0}};
            pk[i]        <= 16'd0;
            env[i]       <= 8'd0;
            peak[i]      <= 8'd0;
            peak_vel[i]  <= 3'd1;
        end
    end else begin
        // Peak caps fall with a velocity that ramps up, so a full-scale peak
        // drops in about half a second instead of the 4.25 s a 1-per-frame step
        // takes. frame_cnt_nxt, not frame_cnt: the model increments before it
        // reads the ramp phase.
        if (I_frame_start) begin
            frame_cnt <= frame_cnt_nxt;
            for (i = 0; i < NBAND; i = i + 1) begin
                if (peak[i] > env[i]) begin
                    // Guarded: peak 2 with velocity 4 must land on 0, not 254.
                    peak[i] <= (peak[i] > peak_vel[i]) ? (peak[i] - peak_vel[i])
                                                       : 8'd0;
                    if ((frame_cnt_nxt[1:0] == 2'd0) && (peak_vel[i] < 3'd4))
                        peak_vel[i] <= peak_vel[i] + 3'd1;
                end else begin
                    peak[i]     <= env[i];
                    peak_vel[i] <= 3'd1;
                end
            end
        end

        // One 48 kHz stereo pair. Every second pair launches an internal sample.
        //
        // The latch is unconditional but the FSM start is guarded, so a launch
        // landing mid-run would drop one sample rather than corrupt a running
        // filter. It cannot land mid-run: a full 16-band sweep costs 5 states x
        // 16 bands = 80 cycles, and launches are 1049 cycles apart
        // (25.175 MHz / 24 kHz / 2). The guard is belt and braces.
        if (I_audio_valid) begin
            if (abs_l8 > pk_l) pk_l <= abs_l8;
            if (abs_r8 > pk_r) pk_r <= abs_r8;

            if (decim) begin
                xin_lat <= xin_w;
                dx8_lat <= dx8_w;
                decim   <= 1'b0;
                if (mac_state == ST_IDLE) begin
                    mac_state <= ST_LAUNCH0;
                    band_cnt  <= 4'd0;
                    acc       <= {ACC_BITS{1'b0}};
                    prod      <= {ACC_BITS{1'b0}};
                end
            end else begin
                mix16_prev <= mix16_w;
                decim      <= 1'b1;
            end
        end

        // Ordered after I_audio_valid so that on the rare cycle where a window
        // closes and a new pair arrives together, the clear wins -- which is
        // what the model does, since step_sample runs after the peak-hold.
        case (mac_state)
            ST_IDLE: begin
                // Deliberately empty. This case executes after the launch above
                // in source order, so if ST_IDLE fell through to `default` its
                // `mac_state <= ST_IDLE` would be the later non-blocking
                // assignment and would cancel the launch every time.
            end
            ST_LAUNCH0: begin
                ma        <= band_b0(band_cnt);
                mb        <= dx8_lat;
                mac_state <= ST_LAUNCH1;
            end
            ST_LAUNCH1: begin
                prod      <= ma * mb;                     // B0 * dx8
                ma        <= band_na1(band_cnt);
                mb        <= y_n1[band_cnt];
                mac_state <= ST_LAUNCH2;
            end
            ST_LAUNCH2: begin
                acc       <= acc + prod;                  // folds in B0 * dx8
                prod      <= ma * mb;                     // NA1 * y[n-1]
                ma        <= band_na2(band_cnt);
                mb        <= y_n2[band_cnt];
                mac_state <= ST_ROUND;
            end
            ST_ROUND: begin
                acc       <= acc + prod;                  // folds in NA1 * y[n-1]
                prod      <= ma * mb;                     // NA2 * y[n-2]
                mac_state <= ST_WRITE;
            end
            ST_WRITE: begin
                // Fold, round, shift and saturate in ONE expression. The model
                // writes this as three sequential Python statements; three
                // `acc <=` here would leave only the last one in effect.
                y_n2[band_cnt] <= y_n1[band_cnt];
                y_n1[band_cnt] <= y_new;
                acc  <= {ACC_BITS{1'b0}};
                prod <= {ACC_BITS{1'b0}};

                if (win_tick) begin
                    env[band_cnt] <= smooth(env[band_cnt], pk_target);
                    pk[band_cnt]  <= 16'd0;
                end else begin
                    pk[band_cnt]  <= pk_target;
                end

                if (run_last) begin
                    mac_state <= ST_IDLE;
                    x_n2      <= x_n1;
                    x_n1      <= xin_lat;
                    win_cnt   <= win_cnt + 1'b1;
                    if (win_tick) begin
                        lvl_l <= smooth(lvl_l, {8'd0, pk_l});
                        lvl_r <= smooth(lvl_r, {8'd0, pk_r});
                        pk_l  <= 8'd0;
                        pk_r  <= 8'd0;
                    end
                end else begin
                    band_cnt  <= band_cnt + 4'd1;
                    mac_state <= ST_LAUNCH0;
                end
            end
            default: mac_state <= ST_IDLE;
        endcase
    end
end

// --------------------------------------------------------------------------
// Raster counters. Column 0 of every line is never visited: x_pos is preset to
// 1 on the rising edge of I_de.
// --------------------------------------------------------------------------
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        x_pos <= 10'd0;
        y_pos <= 10'd0;
        de_d  <= 1'b0;
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
    end
end

// --------------------------------------------------------------------------
// Panel pixel pipeline. Priority order mirrors stage_rgb in the preview: peak
// cap beats bar beats meter beats legend chip beats border beats tick beats
// grid, so the grid shows through only in the gaps between bars and in the
// headroom above them.
// --------------------------------------------------------------------------
wire in_active;
wire in_panel;
wire in_bars;
wire in_meter_l;
wire in_meter_r;
wire in_left;
wire panel_border;
wire grid_line;
wire db_tick;
wire chip_l;
wire chip_r;

wire [9:0] bar_rel_x;
wire [5:0] bar_rel_y;
wire [3:0] bar_idx;
wire [4:0] cell_x;
wire [7:0] bar_env;
wire [7:0] bar_peak;
wire [5:0] bar_h;
wire [5:0] peak_row;
wire       in_bar_col;
wire       bar_pixel;
wire       peak_pixel;
wire       meter_l_pixel;
wire       meter_r_pixel;

wire [9:0] meter_rel_x;
wire [9:0] meter_l_w;
wire [9:0] meter_r_w;

wire [7:0]  bg_r;
wire [7:0]  bg_g;
wire [7:0]  bg_b;
wire [23:0] panel_rgb;
wire [23:0] bar_rgb;
wire [23:0] stage_rgb;

assign in_active = I_de && (x_pos < H_ACTIVE_W) && (y_pos < V_ACTIVE_W);

assign in_panel = in_active &&
                  (x_pos >= PANEL_X_W) && (x_pos <= PANEL_X_LAST_W) &&
                  (y_pos >= PANEL_Y_W) && (y_pos <= PANEL_Y_LAST_W);

assign in_bars = in_panel &&
                 (x_pos >= BAR_X_W) && (x_pos <= BAR_X_LAST_W) &&
                 (y_pos >= BAR_Y_W) && (y_pos <= BAR_Y_LAST_W);

assign in_meter_l = in_panel &&
                    (x_pos >= METER_X_W) && (x_pos <= METER_X_LAST_W) &&
                    (y_pos >= METER_L_Y_W) && (y_pos <= METER_L_Y_LAST_W);

assign in_meter_r = in_panel &&
                    (x_pos >= METER_X_W) && (x_pos <= METER_X_LAST_W) &&
                    (y_pos >= METER_R_Y_W) && (y_pos <= METER_R_Y_LAST_W);

assign in_left = in_panel &&
                 (x_pos >= LEFT_X_W) && (x_pos <= LEFT_X_LAST_W);

assign panel_border = in_panel &&
                      ((x_pos == PANEL_X_W) || (x_pos == PANEL_X_LAST_W) ||
                       (y_pos == PANEL_Y_W) || (y_pos == PANEL_Y_LAST_W));

assign grid_line = in_panel && ((x_pos[5:0] == 6'd0) || (y_pos[4:0] == 5'd0));

// rel_y is 0 at the floor of the bar area and 63 at the top. Taking the low six
// bits of both operands is exact here: the true difference is always in 0..63,
// so any borrow out of bit 5 cancels.
assign bar_rel_x = x_pos - BAR_X_W;
assign bar_rel_y = BAR_Y_LAST_W[5:0] - y_pos[5:0];
assign bar_idx   = bar_rel_x[8:5];
assign cell_x    = bar_rel_x[4:0];

assign bar_env  = env[bar_idx];
assign bar_peak = peak[bar_idx];

// The bar area is exactly 64 px tall, so env 0..127 maps onto it with a shift
// and no multiplier.
assign bar_h    = bar_env[7:1];
assign peak_row = bar_peak[7:1];

assign in_bar_col = (cell_x >= BAR_OFF_LO) && (cell_x <= BAR_OFF_HI);

// The cap sits strictly ABOVE the bar, at rel_y in {pr, pr+1}. Drawing it at
// {pr-1, pr} instead overlaps the bar's own top row and hides one pixel of every
// bar -- a third of the signal on a 3 px bar. pr != 0 is required because at
// peak 0 the same predicate paints a two-row dash at the floor of all 16 bands.
// The difference form avoids an off-by-one at pr == 63, where pr+1 wraps to 0
// and the second row is clipped by the bar area instead of escaping the top.
assign peak_pixel = in_bars && in_bar_col && (peak_row != 6'd0) &&
                    (bar_rel_y >= peak_row) &&
                    ((bar_rel_y - peak_row) <= 6'd1);

assign bar_pixel = in_bars && in_bar_col && (bar_rel_y < bar_h);

assign meter_rel_x = x_pos - METER_X_W;
assign meter_l_w   = {2'd0, lvl_l} << 2;      // 0..127 -> 0..508 of 512 px
assign meter_r_w   = {2'd0, lvl_r} << 2;

assign meter_l_pixel = in_meter_l && (meter_rel_x < meter_l_w);
assign meter_r_pixel = in_meter_r && (meter_rel_x < meter_r_w);

// Legend: a solid chip in the left column, the same colour as its meter. The two
// meters are told apart by colour and by row, not by a glyph, so no font ROM is
// duplicated here.
assign chip_l = in_left && (y_pos >= METER_L_Y_W) && (y_pos <= METER_L_Y_LAST_W);
assign chip_r = in_left && (y_pos >= METER_R_Y_W) && (y_pos <= METER_R_Y_LAST_W);

assign db_tick = in_panel &&
                 ((y_pos == DB6_Y_W) || (y_pos == DB12_Y_W) || (y_pos == DB24_Y_W)) &&
                 (in_bars ||
                  ((x_pos >= LEFT_X_W) && (x_pos <= TICK_LEFT_HI_W)) ||
                  ((x_pos >= TICK_RIGHT_LO_W) && (x_pos <= RIGHT_X_LAST_W)));

// Background dimmed to 3/8 per channel by shift-add: (v >> 2) + (v >> 3).
assign bg_r = {2'b00, I_rgb[23:18]} + {3'b000, I_rgb[23:19]};
assign bg_g = {2'b00, I_rgb[15:10]} + {3'b000, I_rgb[15:11]};
assign bg_b = {2'b00, I_rgb[7:2]}   + {3'b000, I_rgb[7:3]};
assign panel_rgb = {bg_r, bg_g, bg_b};

assign bar_rgb = bar_rel_y[5] ? 24'hFFB84C :
                 bar_rel_y[4] ? 24'h6CFFB0 :
                                24'h40C8FF;

assign stage_rgb = peak_pixel    ? 24'hFFE060 :
                   bar_pixel     ? bar_rgb :
                   meter_l_pixel ? 24'hD8F4FF :
                   meter_r_pixel ? 24'hFFB0D8 :
                   chip_l        ? 24'hD8F4FF :
                   chip_r        ? 24'hFFB0D8 :
                   panel_border  ? 24'h70D0FF :
                   db_tick       ? 24'h5A7A88 :
                   grid_line     ? 24'h24343C :
                   in_panel      ? panel_rgb :
                                   I_rgb;

assign O_rgb = stage_rgb;

endmodule
