
// ============================================================================
// 文件：startup_pulse.v
// 功能：上电后产生一个单周期脉冲(O_pulse)，用于"触发一次 EDID 读取"
// 原理：上电/复位后，计数器从 0 数到 CNT_MAX-1，到达瞬间拉高 O_pulse 一拍，
//       之后 S_done=1 不再触发，保证整个运行期只产生一次脉冲。
// 参数：CNT_MAX — 计数上限（默认 100000，配合 video_clk=25MHz 约 4ms 后触发）
// ============================================================================
module startup_pulse #(
    parameter [19:0] CNT_MAX = 20'd100000
)(
    input  wire I_clk,
    input  wire I_rst,
    output reg  O_pulse
);

reg [19:0] S_cnt;   // 上电计数：0 -> CNT_MAX-1
reg        S_done;  // 脉冲已产生标志，置位后不再触发

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_cnt   <= 20'd0;
        S_done  <= 1'b0;
        O_pulse <= 1'b0;
    end
    else begin
        O_pulse <= 1'b0;   // 默认每拍拉低，仅到达瞬间拉高一拍

        if (!S_done) begin
            // 计数到 CNT_MAX-1 瞬间：拉高 O_pulse 一拍，并锁定 S_done
            if (S_cnt == CNT_MAX - 1'b1) begin
                S_cnt   <= S_cnt;    // 计数停在此刻
                S_done  <= 1'b1;     // 此后不再触发
                O_pulse <= 1'b1;
            end
            else begin
                S_cnt <= S_cnt + 1'b1;  // 继续计数
            end
        end
    end
end

endmodule
