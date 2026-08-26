
// ============================================================================
// 文件：SD/seg_decoder.v
// 功能：4bit 二进制 -> 7 段数码管段码译码(组合逻辑)
// 段码格式：seg_data[6:0] = {g,f,e,d,c,b,a}(a=最低位，对应段 a~g)
// 用法：输入 bin_data(0~F) 输出对应 0~9/A~F 的段码；全 1(7'b111_1111)表示熄灭
// 本工程用于把 SD 卡状态码 state_code 显示到数码管
// ============================================================================
module seg_decoder
(
	input[3:0]      bin_data,     // bin data input
	output reg[6:0] seg_data      // seven segments LED output
);

always@(*)
begin
	case(bin_data)
		4'd0:seg_data <= 7'b100_0000;
		4'd1:seg_data <= 7'b111_1001;
		4'd2:seg_data <= 7'b010_0100;
		4'd3:seg_data <= 7'b011_0000;
		4'd4:seg_data <= 7'b001_1001;
		4'd5:seg_data <= 7'b001_0010;
		4'd6:seg_data <= 7'b000_0010;
		4'd7:seg_data <= 7'b111_1000;
		4'd8:seg_data <= 7'b000_0000;
		4'd9:seg_data <= 7'b001_0000;
		4'ha:seg_data <= 7'b000_1000;
		4'hb:seg_data <= 7'b000_0011;
		4'hc:seg_data <= 7'b100_0110;
		4'hd:seg_data <= 7'b010_0001;
		4'he:seg_data <= 7'b000_0110;
		4'hf:seg_data <= 7'b000_1110;
		default:seg_data <= 7'b111_1111;
	endcase
end
endmodule