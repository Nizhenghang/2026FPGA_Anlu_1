
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关
    input                       key3,           // 亮度档位循环
    input       [3:0]           sw,             // 拨码开关：sw[2:0] (SW1-3) 选转场特效，sw[3] (SW4) 屏蔽滚动字幕（ON=隐藏，与 SW1-3 极性相反）
    input                       uart_rx,        // 串口屏 -> FPGA，F12，经板载 CH340/Type-C（PULLUP）
    output                      uart_tx,        // FPGA -> 串口屏，D12；Stage 1 恒为空闲高

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
parameter BUF2_ADDR     = 24'd614400;
parameter BUF3_ADDR     = 24'd921600;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data_base;
wire [23:0] vout_data_bright;
wire [23:0] vout_data_fade;
wire [23:0] vout_data_audio;
wire [23:0] vout_data_osd;
wire [23:0] vout_data;
wire        marquee_en;
wire        display_valid;
wire        auto_play_enabled;
wire        key3_bright_press;
wire        video_frame_start;

// Stage 4/5 transition controller outputs, all video_clk domain. bot/top are the
// two buffer selectors frame_fifo_read turns into read base addresses; they are
// equal except while a band effect is revealing the new picture. trans_effect is
// the band code 1..6 that tells frame_fifo_read's select_top which sweep shape to
// draw, and is 0 when idle or fading.
wire [1:0]  trans_bot_idx;
wire [1:0]  trans_top_idx;
wire [2:0]  trans_effect;
wire [1:0]  trans_img_idx;
wire [3:0]  trans_fade_level;
// Transition mode select. The physical path inverts the active-low DIP switches
// (ON connects the pin to GND) to an intuitive ON=1 mode: 000 auto-cycle,
// 001..110 force one band effect, 111 force fade. The serial screen can override
// it through a mux at the assignment below; with no screen command the override
// is off and trans_mode is exactly ~sw_v1 as before. sw[3] (SW4) is the banner
// mask, handled separately. Assigned below, next to sw_v1's declaration:
// initializing it here made TD warn HDL-5373 (used before declaration) and risked
// binding a 1-bit implicit net.
wire [2:0]  trans_mode;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;
// Audio bring-up readout on the three 7-segment digits left of state_code.
// chain = {wav_found, audio_phase, streamer ever wrote, header magic rejected},
// wr = peak audio FIFO occupancy on the sd_card_clk side, rd = peak occupancy on
// the video_clk side. A non-zero wr with a zero rd pins the fault to the CDC.
wire [3:0]  dbg_audio_chain;
wire [3:0]  dbg_aud_wr_peak;
wire [3:0]  dbg_gate;
wire [3:0]  dbg_loaded_cnt;
wire [3:0]  dbg_fail;
wire [3:0]  dbg_found_cnt;
wire [3:0]  dbg_next_idx;
wire [6:0]  seg_data_aud_chain;
wire [6:0]  seg_data_gate;
wire [6:0]  seg_data_cnt;
wire [6:0]  seg_data_fail;
wire [6:0]  seg_data_found;
wire [6:0]  seg_data_next;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
// Write-side FIFO occupancy, routed from frame_read_write to the scaler inside
// sd_card_bmp so backpressure never has to leave the sd_card_clk domain.
wire [8:0]  sd_card_write_fifo_usedw;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;
reg  [1:0]  disp_buf_idx_v0;
reg  [1:0]  disp_buf_idx_v1;
reg  [3:0]  state_code_v0;
reg  [3:0]  state_code_v1;
reg         auto_play_v0;
reg         auto_play_v1;
reg         display_valid_v0;
reg         display_valid_v1;
reg  [2:0]  brightness_level;
reg  [2:0]  brightness_level_v0;
reg  [2:0]  brightness_level_v1;
reg  [2:0]  sw_v0;
reg  [2:0]  sw_v1;
reg         sw4_v0;
reg         sw4_v1;
reg         vs_d;

// ---- 串口屏控制：uart_screen_ctrl 的 clk 域命令效果 ----
wire        cmd_next_pulse;
wire        cmd_auto_pulse;
wire        cmd_bright_cycle_pulse;
wire [2:0]  cmd_bright_set;
wire        cmd_bright_set_v;
wire [2:0]  cmd_mode;
wire        cmd_mode_set;
wire        cmd_marquee;
wire        cmd_marquee_set;
wire [1:0]  cmd_img_sel;
wire        cmd_img_sel_set;

// mode/marquee 覆盖：clk 域锁存屏幕设定值，物理拨码一旦变动即清除覆盖
// （last-writer-wins 兜底，两端互为退路）。ovr_en=0 时下面的 trans_mode /
// marquee_en 退回已验证的物理项 ~sw_v1 / sw4_v1，逐位不变——没接屏幕时就是
// 改前的行为。
reg  [2:0]  mode_ovr_val;
reg         mode_ovr_en;
reg         marq_ovr_val;
reg         marq_ovr_en;
// 物理拨码同步到 clk 域并做变动检测（与 video_clk 域的 sw_v0/sw_v1、sw4_v0/sw4_v1
// 是各自独立的同步器，互不影响）
reg  [2:0]  sw_c0, sw_c1, sw_c2;
reg         sw4_c0, sw4_c1, sw4_c2;
// 覆盖状态再 2FF 同步进 video_clk，供 trans_mode / marquee_en 的 mux 使用
reg  [2:0]  mode_ovr_val_v0, mode_ovr_val_v1;
reg         mode_ovr_en_v0, mode_ovr_en_v1;
reg         marq_ovr_val_v0, marq_ovr_val_v1;
reg         marq_ovr_en_v0, marq_ovr_en_v1;

// next/auto/img 命令脉冲 clk -> sd_card_clk 的 toggle-CDC：clk 域每来一条命令翻转
// 一个 toggle，sd_card_clk 域 2FF 同步后用 s1^s2 还原成单周期脉冲。img 的 2-bit
// 目标值用 data+toggle 同步（数据准静态、人类速率，toggle 边沿到达时 img_sel_s1
// 已稳定 ≥2 拍）。
reg         next_tgl, auto_tgl, img_tgl;
reg  [1:0]  img_sel_lat;
reg         next_tgl_s0, next_tgl_s1, next_tgl_s2;
reg         auto_tgl_s0, auto_tgl_s1, auto_tgl_s2;
reg         img_tgl_s0, img_tgl_s1, img_tgl_s2;
reg  [1:0]  img_sel_s0, img_sel_s1;
wire        cmd_next_pulse_sd    = next_tgl_s1 ^ next_tgl_s2;
wire        cmd_auto_pulse_sd    = auto_tgl_s1 ^ auto_tgl_s2;
wire        cmd_img_sel_pulse_sd = img_tgl_s1  ^ img_tgl_s2;

// 转场模式：屏幕覆盖优先，否则退回已验证的物理项 ~sw_v1（一位未动）
assign trans_mode = mode_ovr_en_v1 ? mode_ovr_val_v1 : ~sw_v1;

// SW4 masks the scrolling slogan banner. Deliberately the inverse polarity of
// SW1-3: those are mode selectors where ON=1 picks something, this one is a
// kill switch, and the all-OFF power-up state must still show the banner so
// the board demos out of the box. Pin is active-low with a PULLUP, so the raw
// synced value is 1 when SW4 is OFF, hence no inversion here. The serial screen
// can override the mask (marq_ovr_en_v1); otherwise the verified physical term
// sw4_v1 wins, so with no screen command this is exactly the old behaviour.
assign marquee_en = marq_ovr_en_v1 ? marq_ovr_val_v1 : sw4_v1;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

// Music playback CDC. sd_audio_stream (inside sd_card_bmp, sd_card_clk domain)
// writes {R,L} stereo frames into the async FIFO; audio_pcm_player (video_clk
// domain) reads them at 48 kHz. wrusedw is the streamer's sector-boundary
// backpressure, rdusedw tells the pacer whether the show-ahead head is live.
wire        aud_fifo_we;
wire [31:0] aud_fifo_di;
wire [8:0]  aud_fifo_wrusedw;
wire        aud_fifo_re;
wire [31:0] aud_fifo_dout;
wire [8:0]  aud_fifo_rdusedw;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// video_frame_start fires on the vsync edge as seen AFTER video_delay's 20 tap
// shift register, while video_timing_data raises read_req on the same edge seen
// before it. So this pulse lands about 20 video clocks after the read request
// that fetches the current frame, which is why video_transition treats an index
// change made here as taking effect on the NEXT frame.
assign video_frame_start = vs_d & ~vs;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n | ~audio_pll_lock;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) u_key_brightness (
    .clk        (clk),
    .rst        (rst_all),
    .button_in  (key3),
    .press_pulse(key3_bright_press)
);

always @(posedge clk or posedge rst_all) begin
    if (rst_all)
        brightness_level <= 3'd2;
    else if (cmd_bright_set_v)                              // 屏幕 BRGT n：直接设档（优先）
        brightness_level <= cmd_bright_set;
    else if (key3_bright_press || cmd_bright_cycle_pulse)   // key3 或屏幕 BRUP：循环 +1
        brightness_level <= (brightness_level == 3'd4) ? 3'd0 : (brightness_level + 3'd1);
end

// 串口屏控制源：clk 域把 UART 字节翻译成命令效果。所有跨域注入都由下面的
// toggle-CDC / 2FF 同步完成，本实例只在 clk 域产生效果，不含合并策略。
uart_screen_ctrl #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD        (9600)
) u_uart_screen_ctrl (
    .clk                    (clk),
    .rst                    (rst_all),
    .uart_rx                (uart_rx),
    .uart_tx                (uart_tx),
    .cmd_next_pulse         (cmd_next_pulse),
    .cmd_auto_pulse         (cmd_auto_pulse),
    .cmd_bright_cycle_pulse (cmd_bright_cycle_pulse),
    .cmd_bright_set         (cmd_bright_set),
    .cmd_bright_set_v       (cmd_bright_set_v),
    .cmd_mode               (cmd_mode),
    .cmd_mode_set           (cmd_mode_set),
    .cmd_marquee            (cmd_marquee),
    .cmd_marquee_set        (cmd_marquee_set),
    .cmd_img_sel            (cmd_img_sel),
    .cmd_img_sel_set        (cmd_img_sel_set)
);

// mode/marquee 覆盖锁存 + 物理拨码变动检测（clk 域）。屏幕命令置 ovr_en 并锁值；
// 任一物理拨码变动清 ovr_en，物理路径立即重新接管。复位值匹配 PULLUP 空闲态
// （SW1-3 全 OFF = 111，SW4 OFF = 1），上电不会误判为"拨码变动"而清掉尚未置起的覆盖。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        sw_c0 <= 3'b111; sw_c1 <= 3'b111; sw_c2 <= 3'b111;
        sw4_c0 <= 1'b1;  sw4_c1 <= 1'b1;  sw4_c2 <= 1'b1;
        mode_ovr_val <= 3'd0; mode_ovr_en <= 1'b0;
        marq_ovr_val <= 1'b1; marq_ovr_en <= 1'b0;
    end else begin
        sw_c0 <= sw[2:0]; sw_c1 <= sw_c0; sw_c2 <= sw_c1;
        sw4_c0 <= sw[3];  sw4_c1 <= sw4_c0; sw4_c2 <= sw4_c1;

        if (cmd_mode_set) begin
            mode_ovr_val <= cmd_mode;
            mode_ovr_en  <= 1'b1;
        end else if (sw_c1 != sw_c2) begin
            mode_ovr_en  <= 1'b0;
        end

        if (cmd_marquee_set) begin
            marq_ovr_val <= cmd_marquee;
            marq_ovr_en  <= 1'b1;
        end else if (sw4_c1 != sw4_c2) begin
            marq_ovr_en  <= 1'b0;
        end
    end
end

// 命令脉冲 -> toggle（clk 域）。img 的目标值先锁存再翻转 toggle，保证 data+toggle
// 同步时数据先于 toggle 边沿稳定。
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        next_tgl <= 1'b0; auto_tgl <= 1'b0; img_tgl <= 1'b0;
        img_sel_lat <= 2'd0;
    end else begin
        if (cmd_next_pulse) next_tgl <= ~next_tgl;
        if (cmd_auto_pulse) auto_tgl <= ~auto_tgl;
        if (cmd_img_sel_set) begin
            img_sel_lat <= cmd_img_sel;
            img_tgl     <= ~img_tgl;
        end
    end
end

// sd_card_clk 域：2FF 同步 toggle，s1^s2 还原单周期脉冲；img 选图值同样 2FF 同步。
// 复位后 toggle 与同步链都为 0，不会冒出虚假脉冲；这些脉冲与 sd_card_bmp 内部
// 消抖出的 key_next_press / key_auto_press OR 合并，实体按键仍是兜底。
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        next_tgl_s0 <= 1'b0; next_tgl_s1 <= 1'b0; next_tgl_s2 <= 1'b0;
        auto_tgl_s0 <= 1'b0; auto_tgl_s1 <= 1'b0; auto_tgl_s2 <= 1'b0;
        img_tgl_s0  <= 1'b0; img_tgl_s1  <= 1'b0; img_tgl_s2  <= 1'b0;
        img_sel_s0  <= 2'd0; img_sel_s1  <= 2'd0;
    end else begin
        next_tgl_s0 <= next_tgl; next_tgl_s1 <= next_tgl_s0; next_tgl_s2 <= next_tgl_s1;
        auto_tgl_s0 <= auto_tgl; auto_tgl_s1 <= auto_tgl_s0; auto_tgl_s2 <= auto_tgl_s1;
        img_tgl_s0  <= img_tgl;  img_tgl_s1  <= img_tgl_s0;  img_tgl_s2  <= img_tgl_s1;
        img_sel_s0  <= img_sel_lat; img_sel_s1 <= img_sel_s0;
    end
end

// 将 SD 控制域的慢速状态同步到 video_clk 域，供 OSD 和黑屏门控使用
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        disp_buf_idx_v0  <= 2'd0;
        disp_buf_idx_v1  <= 2'd0;
        state_code_v0    <= 4'd0;
        state_code_v1    <= 4'd0;
        auto_play_v0     <= 1'b0;
        auto_play_v1     <= 1'b0;
        display_valid_v0 <= 1'b0;
        display_valid_v1 <= 1'b0;
        brightness_level_v0 <= 3'd2;
        brightness_level_v1 <= 3'd2;
        sw_v0 <= 3'b111;                    // ~3'b111 = 3'b000 = auto-cycle out of reset
        sw_v1 <= 3'b111;
        sw4_v0 <= 1'b1;                     // PULLUP: SW4 OFF = 1 = banner shown out of reset
        sw4_v1 <= 1'b1;
        mode_ovr_val_v0 <= 3'd0; mode_ovr_val_v1 <= 3'd0;
        mode_ovr_en_v0  <= 1'b0; mode_ovr_en_v1  <= 1'b0;   // 覆盖默认关：trans_mode 退回 ~sw_v1
        marq_ovr_val_v0 <= 1'b1; marq_ovr_val_v1 <= 1'b1;
        marq_ovr_en_v0  <= 1'b0; marq_ovr_en_v1  <= 1'b0;   // 覆盖默认关：marquee_en 退回 sw4_v1
        vs_d <= 1'b0;
    end else begin
        disp_buf_idx_v0  <= disp_buf_idx;
        disp_buf_idx_v1  <= disp_buf_idx_v0;
        state_code_v0    <= state_code;
        state_code_v1    <= state_code_v0;
        auto_play_v0     <= auto_play_enabled;
        auto_play_v1     <= auto_play_v0;
        display_valid_v0 <= display_valid;
        display_valid_v1 <= display_valid_v0;
        brightness_level_v0 <= brightness_level;
        brightness_level_v1 <= brightness_level_v0;
        sw_v0 <= sw[2:0];                   // synchronize raw active-low pins SW1-3; sw[3] is synced separately below
        sw_v1 <= sw_v0;
        sw4_v0 <= sw[3];                    // SW4 banner mask, own 2-FF chain, see marquee_en
        sw4_v1 <= sw4_v0;
        mode_ovr_val_v0 <= mode_ovr_val; mode_ovr_val_v1 <= mode_ovr_val_v0;  // clk -> video_clk 2FF
        mode_ovr_en_v0  <= mode_ovr_en;  mode_ovr_en_v1  <= mode_ovr_en_v0;
        marq_ovr_val_v0 <= marq_ovr_val; marq_ovr_val_v1 <= marq_ovr_val_v0;
        marq_ovr_en_v0  <= marq_ovr_en;  marq_ovr_en_v1  <= marq_ovr_en_v0;
        vs_d <= vs;
    end
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key1),
    .key_auto          (key2),
    .cmd_next_pulse    (cmd_next_pulse_sd),
    .cmd_auto_pulse    (cmd_auto_pulse_sd),
    .cmd_img_sel       (img_sel_s1),
    .cmd_img_sel_pulse (cmd_img_sel_pulse_sd),
    .state_code        (state_code),
    .display_valid     (display_valid),
    .auto_play_enabled (auto_play_enabled),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_fifo_usedw  (sd_card_write_fifo_usedw),
    .aud_fifo_we       (aud_fifo_we),
    .aud_fifo_di       (aud_fifo_di),
    .aud_fifo_wrusedw  (aud_fifo_wrusedw),
    .dbg_audio_chain   (dbg_audio_chain),
    .dbg_aud_wr_peak   (dbg_aud_wr_peak),
    .dbg_gate          (dbg_gate),
    .dbg_loaded_cnt    (dbg_loaded_cnt),
    .dbg_fail          (dbg_fail),
    .dbg_found_cnt     (dbg_found_cnt),
    .dbg_next_idx      (dbg_next_idx),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

// Audio bring-up readout; all six digits are used. Left to right the panel
// reads [loaded count][fail cause][scan found count][next load index][chain]
// [state]. With chain == 8 and count < 4, found vs next separates "the scan
// never saw a fourth BMP" (found < 4) from "the fourth load was abandoned"
// (found == 4, next == 4), and fail says which watchdog did it: bit3 the 1 s
// no-progress stall, bit2 a rejected header, bits1:0 the retry counter.
seg_decoder seg_decoder_aud_cnt(
    .bin_data          (dbg_loaded_cnt),
    .seg_data          (seg_data_cnt)
);

seg_decoder seg_decoder_aud_fail(
    .bin_data          (dbg_fail),
    .seg_data          (seg_data_fail)
);

seg_decoder seg_decoder_aud_found(
    .bin_data          (dbg_found_cnt),
    .seg_data          (seg_data_found)
);

seg_decoder seg_decoder_aud_next(
    .bin_data          (dbg_next_idx),
    .seg_data          (seg_data_next)
);

seg_decoder seg_decoder_aud_chain(
    .bin_data          (dbg_audio_chain),
    .seg_data          (seg_data_aud_chain)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,seg_data_cnt}),
    .seg_data_1        ({1'b1,seg_data_fail}),
    .seg_data_2        ({1'b1,seg_data_found}),
    .seg_data_3        ({1'b1,seg_data_next}),
    .seg_data_4        ({1'b1,seg_data_aud_chain}),
    .seg_data_5        ({1'b1,seg_data_0})
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data_base = display_valid_v1 ? vout_data_raw : 24'd0;

video_brightness u_video_brightness (
    .I_rgb   (vout_data_base),
    .I_level (display_valid_v1 ? brightness_level_v1 : 3'd2),
    .O_rgb   (vout_data_bright)
);

// Stage 4 transition controller. Decides when the panel is allowed to see a
// buffer switch and how. sd_card_bmp still owns which picture is current; this
// only gates the handover, so the SD side and the write side are untouched.
video_transition #(
    .FADE_MAX    (4'd8),
    // Frames the two selectors are held apart. Must exceed the ramp length
    // WIPE_GRP_MAX / WIPE_GRP_STEP = 240 / 8 = 30 frames in frame_read_write,
    // plus margin for the one frame offset between I_frame_start and the read
    // request that precedes it. 36 leaves 6 frames of saturated full new
    // picture before the selectors are equalised again, and 38 frames end to
    // end is 0.63s at 60Hz, inside the 1s auto play interval in sd_card_bmp,
    // so a wipe always finishes before the picture is allowed to advance again.
    .WIPE_HOLD   (6'd36),
    .WIPE_SETTLE (6'd2)
) u_video_transition (
    .I_clk           (video_clk),
    .I_rst           (rst_all),
    .I_frame_start   (video_frame_start),
    .I_display_valid (display_valid_v1),
    .I_disp_idx      (disp_buf_idx_v1),
    .I_mode          (trans_mode),
    .O_bot_idx       (trans_bot_idx),
    .O_top_idx       (trans_top_idx),
    .O_effect        (trans_effect),
    .O_img_idx       (trans_img_idx),
    .O_fade_level    (trans_fade_level)
);

video_fade u_video_fade (
    .I_display_valid (display_valid_v1),
    .I_level         (trans_fade_level),
    .I_rgb           (vout_data_bright),
    .O_rgb           (vout_data_fade)
);

audio_visualizer #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_audio_visualizer (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_de          (de),
    .I_frame_start (video_frame_start),
    .I_rgb         (vout_data_fade),
    .I_audio_valid (audio_valid),
    .I_audio_left  (audio_left_data),
    .I_audio_right (audio_right_data),
    .O_rgb         (vout_data_audio)
);

osd_overlay #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_osd_overlay (
    .I_clk           (video_clk),
    .I_rst           (rst_all),
    .I_de            (de),
    .I_rgb           (vout_data_audio),
    .I_display_valid (display_valid_v1),
    .I_image_index   (trans_img_idx),
    .I_auto_play     (auto_play_v1),
    .I_brightness    (brightness_level_v1),
    .I_state_code    (state_code_v1),
    .O_rgb           (vout_data_osd)
);

marquee_overlay #(
    .H_ACTIVE (640),
    .V_ACTIVE (480)
) u_marquee_overlay (
    .I_clk (video_clk),
    .I_rst (rst_all),
    .I_de  (de),
    .I_rgb (vout_data_osd),
    .I_en  (marquee_en),
    .O_rgb (vout_data)
);

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (BUF2_ADDR),
    .read_addr_3       (BUF3_ADDR),
    .read_addr_index   (trans_bot_idx),
    .read_addr_index_top (trans_top_idx),
    .read_effect       (trans_effect),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (BUF3_ADDR),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_fifo_usedw  (sd_card_write_fifo_usedw)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：TF 卡 WAV 流式播放 =====================
// sd_audio_stream (inside sd_card_bmp, sd_card_clk domain) streams PCM frames
// off the card into this async FIFO, which crosses them into video_clk. The
// pacer below emits the continuous 48 kHz valid/sample stream the HDMI audio
// core and audio_arc_calculate need -- including silence on underrun, so the
// ACR reference never gaps.
//
// The old synthetic-tone path (hdmi_audio_tone_i2s_64fs + I2S_receiver) is
// removed from the build; both module files stay in the repo so the tone can be
// re-hung for debugging. audio_mclk is still driven by PLL_HDMI_AUDIO (kept
// because rst_all gates on audio_pll_lock -- the reset tree is unchanged) but
// is now unused; audio_i2s_* are unused dangling wires.
wfifo_32_32_512 u_audio_fifo (
    .rst        (rst_all),
    .clkw       (sd_card_clk),
    .clkr       (video_clk),
    .we         (aud_fifo_we),
    .di         (aud_fifo_di),
    .re         (aud_fifo_re),
    .dout       (aud_fifo_dout),
    .valid      (),
    .full_flag  (),
    .empty_flag (),
    .afull      (),
    .aempty     (),
    .wrusedw    (aud_fifo_wrusedw),
    .rdusedw    (aud_fifo_rdusedw)
);

audio_pcm_player #(
    .CLK_FREQ_HZ    (25_000_000),
    .SAMPLE_RATE_HZ (48_000)
) u_audio_pcm_player (
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .fifo_re            (aud_fifo_re),
    .fifo_dout          (aud_fifo_dout),
    .fifo_rdusedw       (aud_fifo_rdusedw),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs),
    .I_de          (de),
    .I_rgb         (vout_data),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
