
// ============================================================================
// 文件：SD/spi_master.v
// 功能：SPI 主设备字节收发（最底层物理层）
// 协议：CPOL=1, CPHA=1（SD 卡 SPI 模式要求），MSB 先发送
// 接口：wr_req 拉高请求发送 1 字节(data_in)；wr_ack 拉高表示本字节收发完成；
//       data_out 返回收到的 1 字节；nCS_ctrl 由上层控制片选
// 状态机：IDLE -> DCLK_IDLE(等半个 SPI 周期) -> DCLK_EDGE(产生时钟边沿, 共16边沿)
//         -> LAST_HALF_CYCLE -> ACK -> ACK_WAIT -> IDLE
// 时钟分频：clk_div 决定 1 个 SPI bit 占用多少个 sys_clk 周期(值越大 SPI 越慢)
// ============================================================================
module spi_master
(
	input                       sys_clk,
	input                       rst,
	output                      nCS,       //chip select (SPI mode)
	output                      DCLK,      //spi clock
	output                      MOSI,      //spi data output
	input                       MISO,      //spi input
	input                       CPOL,
	input                       CPHA,
	input                       nCS_ctrl,
	input[15:0]                 clk_div,
	input                       wr_req,
	output                      wr_ack,
	input[7:0]                  data_in,
	output[7:0]                 data_out
);
localparam                   IDLE            = 0;
localparam                   DCLK_EDGE       = 1;
localparam                   DCLK_IDLE       = 2;
localparam                   ACK             = 3;
localparam                   LAST_HALF_CYCLE = 4;
localparam                   ACK_WAIT        = 5;
reg                          DCLK_reg;
reg[7:0]                     MOSI_shift;
reg[7:0]                     MISO_shift;
reg[2:0]                     state;
reg[2:0]                     next_state;
reg [15:0]                   clk_cnt;
reg[4:0]                     clk_edge_cnt;
assign MOSI = MOSI_shift[7];
assign DCLK = DCLK_reg;
assign data_out = MISO_shift;
assign wr_ack = (state == ACK);
assign nCS = nCS_ctrl;
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		state <= IDLE;
	else
		state <= next_state;
end

always@(*)
begin
	case(state)
		IDLE:
			if(wr_req == 1'b1)
				next_state <= DCLK_IDLE;
			else
				next_state <= IDLE;
		DCLK_IDLE:
			//half a SPI clock cycle produces a clock edge
			if(clk_cnt == clk_div)
				next_state <= DCLK_EDGE;
			else
				next_state <= DCLK_IDLE;
		DCLK_EDGE:
			//a SPI byte with a total of 16 clock edges
			if(clk_edge_cnt == 5'd15)
				next_state <= LAST_HALF_CYCLE;
			else
				next_state <= DCLK_IDLE;
		//this is the last data edge
		LAST_HALF_CYCLE:
			if(clk_cnt == clk_div)
				next_state <= ACK;
			else
				next_state <= LAST_HALF_CYCLE;
		//send one byte complete
		ACK:
			next_state <= ACK_WAIT;
		//wait for one clock cycle, to ensure that the cancel request signal
		ACK_WAIT:
			next_state <= IDLE;
		default:
			next_state <= IDLE;
	endcase
end

always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		DCLK_reg <= 1'b0;
	else if(state == IDLE)
		DCLK_reg <= CPOL;
        else if(state == DCLK_EDGE)
            DCLK_reg <= ~DCLK_reg;//SPI clock edge：在边沿状态翻转一次，形成 SPI 时钟
end
//SPI clock wait counter
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_cnt <= 16'd0;
	else if(state == DCLK_IDLE || state == LAST_HALF_CYCLE)
		clk_cnt <= clk_cnt + 16'd1;
	else
		clk_cnt <= 16'd0;
end
//SPI clock edge counter
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_edge_cnt <= 5'd0;
	else if(state == DCLK_EDGE)
		clk_edge_cnt <= clk_edge_cnt + 5'd1;
	else if(state == IDLE)
		clk_edge_cnt <= 5'd0;
end
//SPI data output
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MOSI_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MOSI_shift <= data_in;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b1)
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
		else if(CPHA == 1'b1 && (clk_edge_cnt != 5'd0 && clk_edge_cnt[0] == 1'b0))
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
end
//SPI data input
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MISO_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MISO_shift <= 8'h00;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b0)
			MISO_shift <= {MISO_shift[6:0],MISO};
		else if(CPHA == 1'b1 && (clk_edge_cnt[0] == 1'b1))
			MISO_shift <= {MISO_shift[6:0],MISO};
end
endmodule