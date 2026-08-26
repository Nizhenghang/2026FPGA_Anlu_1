// ============================================================================
// 文件：SD/video_timing_data.v
// 功能：产生视频时序(hs/vs/de) 并在每帧开始时发起"读一帧"请求(read_req)
// 结构：例化 color_bar 产生基准时序(hs/vs/de)，本模块再打 2 拍延迟后输出；
//       在 VS 下降沿(video_vs_d0 & ~video_vs)拉高 read_req 一拍(通知 frame_read_write 读新帧)
// 说明：read_req 的 ack 来自 frame_read_write(read_req_ack)，形成"每帧读一次"的节拍
// ============================================================================
module video_timing_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	//output                      read_en,            // Read data enable
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	output                      de                  // video valid
	//output[DATA_WIDTH - 1:0]    vout_data           // video data
);
wire                   video_hs;
wire                   video_vs;
wire                   video_de;
//delay video_hs video_vs  video_de 2 clock cycles
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;		
	end
end
/*
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(video_vs_d0 & ~video_vs) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_req_ack)
		read_req <= 1'b0;
end

color_bar color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),
	.rgb_g(),
	.rgb_b()
);
endmodule 