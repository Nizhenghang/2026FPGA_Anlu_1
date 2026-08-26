/*
creat time: 2020/12/3 15:50

engineer:   xiong guo qiang

description:
对按键操作进行消抖操作，每次按下产生一个时钟周期的高脉冲

*/


module key_remove_shakes( 
	input wire      I_clk,    
	input wire      I_rst_n,
	
	input wire      I_key_in,        
	
	output reg      O_key_trig_out   
);

	///同步到处理时钟域, 连打 4 拍(1d~4d)消除按键异步输入的亚稳态, 取 3d/4d 做边沿检测
	reg       S_key_in_sync_1d;      
	reg       S_key_in_sync_2d;      
	reg       S_key_in_sync_3d;      
	reg	      S_key_in_sync_4d;
	wire      S_key_in_n_edge;       
			  
	reg       S_delay_en;            
	reg       S_delay_en_1d;         
	wire      S_delay_en_n_edge;     
	reg[16:0] S_delay_cnt;           
	
	
	always @ (posedge I_clk) begin
		S_key_in_sync_1d <= I_key_in;
		S_key_in_sync_2d <= S_key_in_sync_1d;
		S_key_in_sync_3d <= S_key_in_sync_2d;
		S_key_in_sync_4d <= S_key_in_sync_3d;
	end
	

	assign S_key_in_n_edge = ~S_key_in_sync_3d & S_key_in_sync_4d;
	
	
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_en <= 1'b0;
		else
			if(S_key_in_n_edge)
				S_delay_en <= 1'b1;
			else if(S_delay_cnt == 'd100000)   // 计数满 100000(约 2ms @50MHz) -> 消抖窗口结束
				S_delay_en <= 1'b0;
			else
				S_delay_en <= S_delay_en;
	end
	
	
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_cnt <= 'd0;
		else
			if(S_delay_en)
				S_delay_cnt <= S_delay_cnt + 1'b1;
			else
				S_delay_cnt <= 'd0;
	end
	
	
	always @ (posedge I_clk) begin
		S_delay_en_1d <= S_delay_en;
	end
	
	
	assign S_delay_en_n_edge = ~S_delay_en & S_delay_en_1d;
	
	
	// 消抖结束后, 在按键释放边沿(S_delay_en 下降沿 且 按键已松开)输出一个时钟周期高脉冲
	always @ (posedge I_clk) begin
		if(S_delay_en_n_edge && !S_key_in_sync_4d)
			O_key_trig_out <= 1'b1;
		else
			O_key_trig_out <= 1'b0;
	end
	

endmodule
