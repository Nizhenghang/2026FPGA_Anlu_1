// ---------------------------------------------------------------------------
// uart_screen_ctrl
//
// clk(50 MHz)-domain bridge that turns a TJC/Nextion serial screen into the
// board's control surface. It replaces key1/key2/key3 and SW1-4 but leaves them
// as a fallback: the merge policy lives in the top level, NOT here. This module
// is a pure "UART byte -> command effect" translator and owns no policy about
// how its effects combine with the physical controls.
//
// Protocol (see README "串口屏控制"):
//   - RX is 8N1 at BAUD (default 9600 == TJC factory default, so the screen
//     works out of the box without reconfiguring its project).
//   - A command is an ASCII frame: 4-char keyword + optional " <digit>",
//     terminated by the TJC convention of three 0xFF bytes. Payload bytes are
//     always ASCII (never 0xFF), so 0xFF unambiguously means terminator.
//   - Commands: NEXT, AUTO, BRUP (no arg); BRGT n, MODE n, MARQ n, IMGX n,
//     FILT n, FONT n, MUSC n.
//   - MODE and FILT arguments are ONE hex character, '0'-'9' then 'A'-'F' for
//     10..15, so both code spaces reach 0..15 while every framed command stays
//     exactly 9 bytes. Lowercase is rejected: the screen project has to send
//     uppercase. A two-digit argument is not a wider code space, it is a 10
//     byte frame that clen == 6 turns down.
//
// Every command effect is emitted in THIS clk domain:
//   - one-clock pulses: cmd_next_pulse / cmd_auto_pulse / cmd_bright_cycle_pulse
//   - value + one-clock set strobe: cmd_bright_set(+_v), cmd_mode(+_set),
//     cmd_marquee(+_set), cmd_img_sel(+_set), cmd_filt(+_set), cmd_font(+_set),
//     cmd_audio(+_set)
// The top level crosses the pulses into sd_card_clk (toggle-CDC) and the levels
// into video_clk (2FF), and merges them with the physical keys/switches.
//
// Reset polarity follows the project convention (see video_transition.v:73):
// rst is rst_all at the top, ACTIVE HIGH, sensitivity `posedge rst`, condition
// `if (rst)`. Do not flip it to `if (!rst)` against `posedge rst` -- the async
// reset would never fire and synthesis would infer a SET alongside the RESET.
//
// Stage 1: RX + parse only. uart_tx is held idle high; Stage 2 adds the TX
// status readback. Until then dbg_rx_toggle / dbg_rx_ff are the only window
// onto the link: they are observation-only (they feed nothing but LEDs) and
// are bring-up aids, not part of the protocol.
// ---------------------------------------------------------------------------

module uart_screen_ctrl #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD        = 9600
)(
    input  wire       clk,
    input  wire       rst,                    // active high (rst_all)
    input  wire       uart_rx,
    output wire       uart_tx,                // Stage 1: idle high

    // ---- command effects, all clk domain ----
    output reg        cmd_next_pulse,         // = key1 (next loaded picture)
    output reg        cmd_auto_pulse,         // = key2 (toggle auto carousel)
    output reg        cmd_bright_cycle_pulse, // = key3 (brightness +1, wrap)
    output reg  [2:0] cmd_bright_set,         // BRGT n: absolute brightness 0..4
    output reg        cmd_bright_set_v,
    output reg  [3:0] cmd_mode,               // MODE n: transition mode 0..15, hex digit
    output reg        cmd_mode_set,
    output reg        cmd_marquee,            // MARQ n: 1 show / 0 hide banner
    output reg        cmd_marquee_set,
    output reg  [1:0] cmd_img_sel,            // IMGX n: picture n-1, 0..3
    output reg        cmd_img_sel_set,
    output reg  [3:0] cmd_filt,               // FILT n: image algorithm 0..15, hex digit
    output reg        cmd_filt_set,
    output reg        cmd_font,               // FONT n: 1 extruded emboss / 0 flat
    output reg        cmd_font_set,
    output reg        cmd_audio,              // MUSC n: 1 TF-card WAV / 0 built-in test tone
    output reg        cmd_audio_set,

    // ---- link debug: uart_tx is idle-high in Stage 1, so there is no readback
    // and these two are the only way to see whether bytes reach the FPGA at all.
    output reg        dbg_rx_toggle,          // flips once per received byte
    output reg        dbg_rx_ff               // most recent byte was 0xFF
);

    // Stage 1 holds the line idle (high). Stage 2 drives it from a TX engine.
    assign uart_tx = 1'b1;

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;   // 5208 @ 50M/9600

    // ------------------------------------------------------------------
    // UART RX: 8N1, mid-bit sampling, LSB first.
    // ------------------------------------------------------------------
    localparam [1:0] RX_IDLE  = 2'd0,
                     RX_START = 2'd1,
                     RX_DATA  = 2'd2,
                     RX_STOP  = 2'd3;

    reg [1:0]  rx_sync;          // 2FF synchronizer on the async uart_rx pin
    reg [1:0]  rx_state;
    reg [15:0] rx_cnt;           // CLKS_PER_BIT-1 = 5207 fits 16 bits
    reg [2:0]  rx_bit;
    reg [7:0]  rx_shift;
    reg [7:0]  rx_byte;
    reg        rx_valid;         // one-clock pulse: rx_byte holds a new byte

    wire       rx_in = rx_sync[1];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_sync  <= 2'b11;
            rx_state <= RX_IDLE;
            rx_cnt   <= 16'd0;
            rx_bit   <= 3'd0;
            rx_shift <= 8'd0;
            rx_byte  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_sync  <= {rx_sync[0], uart_rx};
            rx_valid <= 1'b0;                 // default: pulse for one clock only
            case (rx_state)
            RX_IDLE: begin
                rx_cnt <= 16'd0;
                rx_bit <= 3'd0;
                if (rx_in == 1'b0)            // falling edge -> candidate start
                    rx_state <= RX_START;
            end
            RX_START: begin
                if (rx_cnt == (CLKS_PER_BIT-1)/2) begin
                    if (rx_in == 1'b0) begin  // still low at mid-bit: real start
                        rx_cnt   <= 16'd0;
                        rx_state <= RX_DATA;
                    end else begin
                        rx_state <= RX_IDLE;  // glitch, abort
                    end
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            RX_DATA: begin
                if (rx_cnt == CLKS_PER_BIT-1) begin
                    rx_cnt   <= 16'd0;
                    rx_shift <= {rx_in, rx_shift[7:1]};  // LSB first
                    rx_bit   <= rx_bit + 3'd1;
                    if (rx_bit == 3'd7)
                        rx_state <= RX_STOP;
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            RX_STOP: begin
                if (rx_cnt == CLKS_PER_BIT-1) begin
                    rx_cnt   <= 16'd0;
                    rx_state <= RX_IDLE;
                    if (rx_in == 1'b1) begin  // valid stop bit -> publish byte
                        rx_byte  <= rx_shift;
                        rx_valid <= 1'b1;
                    end
                end else begin
                    rx_cnt <= rx_cnt + 16'd1;
                end
            end
            default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Command parser: accumulate ASCII, dispatch on the third 0xFF.
    //
    // Only the first six bytes are captured (c0..c5); clen keeps counting up to
    // 15 so an over-long frame can never equal the exact lengths the dispatcher
    // matches (4 for keyword-only, 6 for "KEYW d"). That rejects trailing
    // garbage like "NEXTX" without a separate length check.
    // ------------------------------------------------------------------
    reg [7:0] c0, c1, c2, c3, c4, c5;
    reg [3:0] clen;
    reg [1:0] ffc;               // consecutive 0xFF count

    // Single uppercase hex digit decode, shared by the MODE and FILT arguments.
    // c5 is a flop that only ever loads on a payload byte, so both of these are
    // stable for the whole dispatch cycle. 'a'-'f' sit deliberately outside the
    // accepted range: one spelling per value keeps the screen project's button
    // strings unambiguous and keeps the lowercase negative control in
    // tools/sim_uart_ctrl.py meaningful.
    wire       c5_is_hex = ((c5 >= "0") && (c5 <= "9")) ||
                           ((c5 >= "A") && (c5 <= "F"));
    wire [3:0] c5_hex    = (c5 <= "9") ? (c5 - 8'h30)
                                       : ((c5 - 8'h41) + 4'd10);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            c0 <= 8'd0; c1 <= 8'd0; c2 <= 8'd0;
            c3 <= 8'd0; c4 <= 8'd0; c5 <= 8'd0;
            clen <= 4'd0;
            ffc  <= 2'd0;
            cmd_next_pulse         <= 1'b0;
            cmd_auto_pulse         <= 1'b0;
            cmd_bright_cycle_pulse <= 1'b0;
            cmd_bright_set         <= 3'd0;
            cmd_bright_set_v       <= 1'b0;
            cmd_mode               <= 4'd0;
            cmd_mode_set           <= 1'b0;
            cmd_marquee            <= 1'b0;
            cmd_marquee_set        <= 1'b0;
            cmd_img_sel            <= 2'd0;
            cmd_img_sel_set        <= 1'b0;
            cmd_filt               <= 4'd0;
            cmd_filt_set           <= 1'b0;
            cmd_font               <= 1'b0;
            cmd_font_set           <= 1'b0;
            cmd_audio              <= 1'b0;
            cmd_audio_set          <= 1'b0;
            dbg_rx_toggle          <= 1'b0;
            dbg_rx_ff              <= 1'b0;
        end else begin
            // default: every strobe/pulse is high for one clock only
            cmd_next_pulse         <= 1'b0;
            cmd_auto_pulse         <= 1'b0;
            cmd_bright_cycle_pulse <= 1'b0;
            cmd_bright_set_v       <= 1'b0;
            cmd_mode_set           <= 1'b0;
            cmd_marquee_set        <= 1'b0;
            cmd_img_sel_set        <= 1'b0;
            cmd_filt_set           <= 1'b0;
            cmd_font_set           <= 1'b0;
            cmd_audio_set          <= 1'b0;

            if (rx_valid) begin
                dbg_rx_toggle <= ~dbg_rx_toggle;
                dbg_rx_ff     <= (rx_byte == 8'hFF);
                if (rx_byte == 8'hFF) begin
                    if (ffc == 2'd2) begin
                        // ---- third 0xFF: frame complete, dispatch ----
                        // MODE and FILT take the whole hex range, not just the
                        // codes that mean something today. The reserved ones
                        // fall through to a defined harmless behaviour in the
                        // consumer (MODE F is fade, FILT D/E/F are passthrough),
                        // so a mistyped button lands somewhere safe instead of
                        // being dropped with no feedback on a screen that never
                        // reads back.
                        ffc  <= 2'd0;
                        clen <= 4'd0;
                        case ({c0, c1, c2, c3})
                        "NEXT": if (clen == 4'd4) cmd_next_pulse <= 1'b1;
                        "AUTO": if (clen == 4'd4) cmd_auto_pulse <= 1'b1;
                        "BRUP": if (clen == 4'd4) cmd_bright_cycle_pulse <= 1'b1;
                        "BRGT": if (clen == 4'd6 && c4 == " " && c5 >= "0" && c5 <= "4") begin
                                    cmd_bright_set   <= c5 - 8'h30;
                                    cmd_bright_set_v <= 1'b1;
                                 end
                        "MODE": if (clen == 4'd6 && c4 == " " && c5_is_hex) begin
                                    cmd_mode     <= c5_hex;
                                    cmd_mode_set <= 1'b1;
                                 end
                        "MARQ": if (clen == 4'd6 && c4 == " " && (c5 == "0" || c5 == "1")) begin
                                    cmd_marquee     <= (c5 == "1");
                                    cmd_marquee_set <= 1'b1;
                                 end
                        "IMGX": if (clen == 4'd6 && c4 == " " && c5 >= "1" && c5 <= "4") begin
                                    cmd_img_sel     <= (c5 - 8'h30) - 2'd1;
                                    cmd_img_sel_set <= 1'b1;
                                 end
                        "FILT": if (clen == 4'd6 && c4 == " " && c5_is_hex) begin
                                    cmd_filt     <= c5_hex;
                                    cmd_filt_set <= 1'b1;
                                 end
                        "FONT": if (clen == 4'd6 && c4 == " " && (c5 == "0" || c5 == "1")) begin
                                    cmd_font     <= (c5 == "1");
                                    cmd_font_set <= 1'b1;
                                 end
                        "MUSC": if (clen == 4'd6 && c4 == " " && (c5 == "0" || c5 == "1")) begin
                                    cmd_audio     <= (c5 == "1");
                                    cmd_audio_set <= 1'b1;
                                 end
                        default: ;
                        endcase
                    end else begin
                        ffc <= ffc + 2'd1;
                    end
                end else begin
                    // ---- payload byte: capture and count ----
                    ffc <= 2'd0;
                    if (clen < 4'd6) begin
                        case (clen)
                        4'd0: c0 <= rx_byte;
                        4'd1: c1 <= rx_byte;
                        4'd2: c2 <= rx_byte;
                        4'd3: c3 <= rx_byte;
                        4'd4: c4 <= rx_byte;
                        4'd5: c5 <= rx_byte;
                        default: ;
                        endcase
                    end
                    if (clen < 4'd15)
                        clen <= clen + 4'd1;
                end
            end
        end
    end

endmodule
