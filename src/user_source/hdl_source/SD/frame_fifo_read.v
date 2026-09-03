`timescale 1ns/1ps
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter FIFO_DEPTH             = 512,
	parameter BURST_SIZE             = 128,
	//Stage 4 vertical wipe geometry. The frame is read as read_len words of one
	//pixel each, so a 640 pixel line is 640 words and a burst of 256 words is
	//0.4 of a line: burst boundaries only land on a line boundary every five
	//bursts, i.e. every 5 * 256 = 1280 words = 2 lines. That two line pair is
	//the coarsest address redirection granularity that cannot split a burst
	//across two buffers, and 480 / 2 = 240 of them make a frame. WIPE_GRP_STEP
	//is how many groups the boundary advances per frame read, so the ramp runs
	//for WIPE_GRP_MAX / WIPE_GRP_STEP = 30 frames.
	parameter [8:0] WIPE_GRP_MAX     = 9'd240,
	parameter [8:0] WIPE_GRP_STEP    = 9'd8
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
    input							 Sdr_rd_en,
	input							 App_wr_busy,
    output							 O_rd_busy,
	/*
    output reg                       rd_burst_req,               // to external memory controller,send out a burst read request  
	output reg[BURST_BITS - 1:0]     rd_burst_len,               // to external memory controller,data length of the burst read request, not bytes 
	output reg[ADDR_BITS - 1:0]      rd_burst_addr,              // to external memory controller,base address of the burst read request
	input                            rd_burst_data_valid,        // from external memory controller,read request data valid    
	input                            rd_burst_finish,            // from external memory controller,burst read finish
	*/
	output 							 App_rd_en,
	output  [ADDR_BITS - 1:0]		 App_rd_addr,
	
	input                            read_req,                   // data read module read request,keep '1' until read_req_ack = '1'
	output reg                       read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input[ADDR_BITS - 1:0]           read_addr_0,                // data read module read request base address 0, used when read_addr_index = 0
	input[ADDR_BITS - 1:0]           read_addr_1,                // data read module read request base address 1, used when read_addr_index = 1
	input[ADDR_BITS - 1:0]           read_addr_2,                // data read module read request base address 1, used when read_addr_index = 2
	input[ADDR_BITS - 1:0]           read_addr_3,                // data read module read request base address 1, used when read_addr_index = 3
	input[1:0]                       read_addr_index,            // select valid base address from read_addr_0 read_addr_1 read_addr_2 read_addr_3
	input[1:0]                       read_addr_index_top,        // stage 4: selector for the region ABOVE the wipe boundary. Drive it equal to read_addr_index when no wipe is running and this module behaves exactly as it did before.
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[9:0]                      wrusedw                     // from fifo write used words

);
localparam ONE                       = 256'd1;                   //256 bit '1'   you can use ONE[n-1:0] for n bit '1'
localparam ZERO                      = 256'd0;                   //256 bit '0'
//read state machine code
localparam S_IDLE                    = 0;                        //idle state,waiting for frame read
localparam S_ACK                     = 1;                        //read request response
localparam S_CHECK_FIFO              = 2;                        //check the FIFO status, ensure that there is enough space to burst read
localparam S_READ_BURST              = 3;                        //begin a burst read
localparam S_READ_BURST_END          = 4;                        //a burst read complete
localparam S_END                     = 5;                        //a frame of data is read to complete

reg                                  read_req_d0;                //asynchronous read request, synchronize to 'mem_clk' clock domain,first beat
reg                                  read_req_d1;                //second
reg                                  read_req_d2;                //third,Why do you need 3 ? Here's the design habit
reg[ADDR_BITS - 1:0]                 read_len_d0;                //asynchronous read_len(read data length), synchronize to 'mem_clk' clock domain first
reg[ADDR_BITS - 1:0]                 read_len_d1;                //second
reg[ADDR_BITS - 1:0]                 read_len_latch;             //lock read data length
reg[ADDR_BITS - 1:0]                 read_cnt;                   //read data counter
reg[3:0]                             state;                      //state machine
reg[1:0]                             read_addr_index_d0;         //synchronize to 'mem_clk' clock domain first
reg[1:0]                             read_addr_index_d1;         //synchronize to 'mem_clk' clock domain second
reg[1:0]                             read_addr_index_top_d0;     //stage 4, same two beat synchroniser for the wipe selector
reg[1:0]                             read_addr_index_top_d1;
reg[8:0]                             wipe_pos;                   //two line groups taken from the top buffer, advances once per frame read and is never touched inside a frame
reg[8:0]                             grp_to_cross;               //the same number, but counts down as the frame is read so the crossing is a compare against 1
reg[2:0]                             burst_in_grp;               //0..4, five bursts make one 1280 word = 2 line group
reg[ADDR_BITS - 1:0]                 wipe_delta;                 //base_bottom - base_top, added to the address once, at the crossing
reg [ADDR_BITS - 1:0]	 App_rd_addr_r;

reg [BURST_BITS - 1:0]				burst_cnt;
wire								rd_burst_finish;
reg App_rd_en_r;
reg App_rd_en_d0;


wire rd_vld;
reg [3:0] rd_delay;

assign App_rd_addr = {App_rd_addr_r[ADDR_BITS - 1:0]};
assign rd_vld = (state == S_READ_BURST && burst_cnt >= BURST_SIZE);

assign O_rd_busy = (state == S_READ_BURST);//读指令期间
//burst_cnt代表发送的读指令
//但rd_burst_finish需要在发送完十个时钟后拉高，此时数据全部读出
assign rd_burst_finish = (rd_vld && rd_delay == 4'd10);
assign read_finish = (state == S_END) ? 1'b1 : 1'b0;             //read finish at state 'S_END'
assign App_rd_en = App_rd_en_d0;

wire                                 wipe_sel_diff;              //the two selectors disagree, i.e. a wipe is running
wire                                 s_ack_first;                //first cycle of S_ACK, which lasts as long as read_req is held
wire                                 grp_cross;                  //a group aligned burst boundary, and the one the buffer switches at
wire [8:0]                           wipe_pos_next;              //the wipe ramp advanced by one frame
wire [ADDR_BITS - 1:0]               base_top;
wire [ADDR_BITS - 1:0]               base_bot;
wire [ADDR_BITS - 1:0]               wipe_start_base;

//read_addr_0..3 are tied to constants at the top level, so this collapses to a
//4 way mux of literals.
function [ADDR_BITS - 1:0] base_sel;
	input [1:0] idx;
	begin
		case (idx)
			2'd0: base_sel = read_addr_0;
			2'd1: base_sel = read_addr_1;
			2'd2: base_sel = read_addr_2;
			default: base_sel = read_addr_3;
		endcase
	end
endfunction

assign base_top      = base_sel(read_addr_index_top_d1);
assign base_bot      = base_sel(read_addr_index_d1);
assign wipe_sel_diff = (read_addr_index_top_d1 != read_addr_index_d1);
//Above the boundary comes from the top buffer, so a frame whose boundary has
//not moved off group 0 must still start at the bottom one. When the two
//selectors are equal wipe_start_base is base_bot, which is the same 4 way mux
//on read_addr_index_d1 that this module always latched, so the no wipe case is
//bit for bit the original behaviour.
assign wipe_start_base = wipe_sel_diff ? base_top : base_bot;
//read_req_ack is registered in the state machine below and is guaranteed low on
//entry to S_ACK, so this marks the first of the several cycles S_ACK spends
//waiting for read_req to fall. Only the non idempotent wipe bookkeeping is
//gated by it; the address latch itself keeps loading on every S_ACK cycle as it
//always did, and the two agree because the selectors are held stable for a
//whole frame by video_transition.
assign s_ack_first = (state == S_ACK) && (read_req_ack == 1'b0);
//rd_burst_finish needs burst_cnt >= BURST_SIZE, which forces App_rd_en_d0 low,
//so this can never compete with the per word increment in the same cycle.
assign grp_cross = rd_burst_finish && (burst_in_grp == 3'd4) && (grp_to_cross == 9'd1);
//Saturating one step of the wipe ramp. WIPE_GRP_STEP divides WIPE_GRP_MAX
//exactly at the default settings, the clamp is here so another step size
//cannot overshoot past the bottom of the panel.
assign wipe_pos_next = (wipe_pos >= (WIPE_GRP_MAX - WIPE_GRP_STEP)) ? WIPE_GRP_MAX
                                                                   : (wipe_pos + WIPE_GRP_STEP);
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d0 <= 0;
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d1 <= 0;
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
		read_addr_index_top_d0 <= 2'b00;
		read_addr_index_top_d1 <= 2'b00;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;     
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0; 
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
		read_addr_index_top_d0 <= read_addr_index_top;
		read_addr_index_top_d1 <= read_addr_index_top_d0;
		
	end 
end

always @(posedge mem_clk or posedge rst)
begin
	if(rst || App_rd_en)begin
        rd_delay <= 4'd0;
    end
    else if(rd_delay < 4'd10)begin
    	rd_delay <= rd_delay + 1'b1;
    end
end
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_rd_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_rd_en_d0 <= 1'b0;
	end
	else begin
	
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_rd_en)
			burst_cnt <= burst_cnt + 1'b1;
		else
			burst_cnt <= burst_cnt;
		//
		//Stage 4: the base address latch is now wipe_start_base instead of an
		//inline 4 way mux on read_addr_index_d1. With the two selectors equal
		//that is the identical expression, so nothing changes when no wipe is
		//running.
		if(state == S_ACK)
			App_rd_addr_r <= wipe_start_base;
		//One shot redirect at a group aligned burst boundary, out of the top
		//buffer region and down into the bottom one. Fires at most once per
		//frame, because grp_to_cross is decremented on every group boundary and
		//only compares equal to 1 on one of them, the Gth, at word G * 1280.
		else if(grp_cross)
			App_rd_addr_r <= App_rd_addr_r + wipe_delta;
		else if(App_rd_en)
			App_rd_addr_r <= App_rd_addr_r + 1'b1;
		else
			App_rd_addr_r <= App_rd_addr_r;
		//
		if(App_rd_en_r && burst_cnt + App_rd_en < BURST_SIZE)
			App_rd_en_d0 <= 1'b1;
		else
			App_rd_en_d0 <= 1'b0;
	
	end		
end
// ---------------------------------------------------------------------------
// Stage 4 vertical wipe bookkeeping.
//
// Two counters hold the same number and they have to be separate registers,
// which is not obvious and is exactly the mistake the reference model in
// tools/sim_transition.py caught first time round:
//
//   wipe_pos      how many two line groups at the top of the frame come from
//                 read_addr_index_top. Loaded once per frame read and left
//                 alone for the rest of the frame, so it accumulates across
//                 frames and the boundary walks down the panel.
//   grp_to_cross  the group index at which the address has to move back to
//                 read_addr_index. Loaded from wipe_pos_next at the same
//                 moment, then decremented at every group boundary, which
//                 turns the crossing test into a compare against 1 instead of
//                 a compare against a second counter. grp_cross then fires on
//                 the boundary entering group G, exactly the first group that
//                 belongs to the bottom buffer.
//
// Sharing one register for both roles loses the accumulated position, because
// the in frame countdown reaches zero long before the next frame read starts.
//
// Both updates are gated on single cycle events, s_ack_first and
// rd_burst_finish. S_ACK itself lasts as long as read_req is held, several
// mem_clk cycles, and an ungated increment there would advance the boundary
// many times per frame.
//
// Nothing else about the frame read is touched: same burst count, same burst
// length, same read_cnt, same FIFO control, same rd_delay, same App_rd_en
// pattern. Only the value loaded into the address register differs, and only
// at a word offset that is an exact multiple of two lines, so no burst ever
// straddles two buffers and the read FIFO sees an identical stream shape.
// ---------------------------------------------------------------------------
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		wipe_pos     <= 9'd0;
		grp_to_cross <= 9'd0;
		burst_in_grp <= 3'd0;
		wipe_delta   <= ZERO[ADDR_BITS - 1:0];
	end
	else
	begin
		if(s_ack_first)
		begin
			burst_in_grp <= 3'd0;
			wipe_delta   <= base_bot - base_top;
			if(wipe_sel_diff)
			begin
				wipe_pos     <= wipe_pos_next;
				grp_to_cross <= wipe_pos_next;
			end
			else
			begin
				wipe_pos     <= 9'd0;
				grp_to_cross <= 9'd0;
			end
		end
		else if(rd_burst_finish)
		begin
			if(burst_in_grp == 3'd4)
			begin
				burst_in_grp <= 3'd0;
				if(grp_to_cross != 9'd0)
					grp_to_cross <= grp_to_cross - 9'd1;
			end
			else
				burst_in_grp <= burst_in_grp + 3'd1;
		end
	end
end

always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		
		//rd_burst_addr <= ZERO[ADDR_BITS - 1:0];
		//rd_burst_req <= 1'b0;
		App_rd_en_r <= 1'b0;
		
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		//rd_burst_len <= ZERO[BURST_BITS - 1:0];
		read_req_ack <= 1'b0;
	end
	else
		case(state)
			//idle state,waiting for read, read_req_d2 == '1' goto the 'S_ACK'
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end
			//'S_ACK' state completes the read request response, the FIFO reset, the address latch, and the data length latch
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					//read request response
					read_req_ack <= 1'b1;
					//FIFO reset
					fifo_aclr <= 1'b1;
					//select valid base address from read_addr_0 read_addr_1 read_addr_2 read_addr_3
					/*
					if(read_addr_index_d1 == 2'd0)
						App_rd_addr <= read_addr_0;
					else if(read_addr_index_d1 == 2'd1)
						App_rd_addr <= read_addr_1;
					else if(read_addr_index_d1 == 2'd2)
						App_rd_addr <= read_addr_2;
					else if(read_addr_index_d1 == 2'd3)
						App_rd_addr <= read_addr_3;
					*/
					//latch data length
					read_len_latch <= read_len_d1;
				end
				//read data counter reset, read_cnt <= 0;
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_CHECK_FIFO:
			begin
				//if there is a read request at this time, enter the 'S_ACK' state
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the FIFO space is a burst read request, goto burst read state
				else if(wrusedw < (FIFO_DEPTH - BURST_SIZE) && ~App_wr_busy)
				begin
					state <= S_READ_BURST;
					//rd_burst_len <= BURST_SIZE[BURST_BITS - 1:0];
					//rd_burst_req <= 1'b1;
					App_rd_en_r <= 1'b1;
				end
			end
			
			S_READ_BURST:
			begin
				//burst finish  
				if(rd_burst_finish == 1'b1)
				begin
					App_rd_en_r <= 1'b0;
					state <= S_READ_BURST_END;
					//read counter + burst length
					read_cnt <= read_cnt + BURST_SIZE[ADDR_BITS - 1:0];
					//the next burst read address is generated
					//rd_burst_addr <= rd_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end     
			end
			S_READ_BURST_END:
			begin
				//if there is a read request at this time, enter the 'S_ACK' state
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the read counter value is less than the frame length, continue read,
				//otherwise the read is complete
				else if(read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule
