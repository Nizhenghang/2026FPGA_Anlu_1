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
	input[2:0]                       effect,                     // stage 5: band effect code 1..6 (wipe down/up, blinds, split, random bars, comb); 0 and 7 mean no band redirect. From video_transition, synchronised into mem_clk exactly like the two index selectors.
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
reg[2:0]                             effect_d0;                  //stage 5, same two beat synchroniser for the band effect code
reg[2:0]                             effect_d1;
reg[8:0]                             progress;                   //band ramp position in two line groups: advances once per frame read, frozen for the whole frame so every group evaluates select_top against the same value
reg[8:0]                             g;                          //group index within the frame, 0..240, advances at every group aligned burst boundary while the selectors disagree
reg[8:0]                             g_plus1;                    //g + 1, registered so next_sel_comb reads a flop instead of an adder; the 1 cycle lag is invisible because next_sel_r is only consumed at group boundaries ~1280 cycles after g last moved
reg                                  cur_sel;                    //buffer the current group reads from, always select_top(g, progress, effect); 1 is the top buffer
reg                                  next_sel_r;                 //select_top(g+1, progress, effect), registered every cycle so the address mux only ever sees a flop output
reg[2:0]                             burst_in_grp;               //0..4, five bursts make one 1280 word = 2 line group
reg[ADDR_BITS - 1:0]                 wipe_delta;                 //base_bottom - base_top, added to the address when a boundary steps back down into the bottom buffer
reg[ADDR_BITS - 1:0]                 neg_wipe_delta;             //base_top - base_bottom, precomputed so entering the top buffer is a 2:1 mux, not a subtractor, on the ext_mem_clk address path
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

wire                                 wipe_sel_diff;              //the two selectors disagree, i.e. a band effect is running
wire                                 s_ack_first;                //first cycle of S_ACK, which lasts as long as read_req is held
wire                                 grp_boundary;               //a group aligned burst boundary: five bursts = 1280 words = two lines
wire                                 do_redirect;                //a group boundary at which the buffer selection flips, so the address jumps by +/- the buffer delta
wire                                 first_sel;                  //buffer for group 0 of the frame about to be read, latched into cur_sel at s_ack_first
wire                                 next_sel_comb;              //buffer for group g+1, registered into next_sel_r every cycle
wire [8:0]                           progress_next;              //the band ramp advanced by one frame, saturating at WIPE_GRP_MAX
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

// ---------------------------------------------------------------------------
// select_top: the band geometry. For a two line group g, at the frozen ramp
// position prog, under effect code eff, return 1 if that group reads from the
// top buffer (read_addr_index_top) and 0 if it reads from the bottom one
// (read_addr_index). This one function is the generalization that turns the old
// single crossing wipe into a family of vertical sweeps; effect 1 IS that old
// wipe, bit for bit.
//
// Every branch is compares, add/sub, constant shifts, a bit mask and a bit
// reverse, so there is no divider and no DSP. The prog >= WIPE_GRP_MAX guard on
// blinds/split/random is what reveals the last slat / outermost group / last bar
// at the saturated end of the ramp; without it those three effects leave holes in
// the panel, which is exactly what the negative control in Pass G of
// tools/sim_transition.py checks for. WIPE_GRP_MAX is 240 here, so the blinds
// slat is 16 groups (15 slats), the split centre is 120, and bitrev8 spreads the
// 240 group indices over 0..255.
// ---------------------------------------------------------------------------
function select_top;
	input [8:0] gi;
	input [8:0] prog;
	input [2:0] eff;
	reg [8:0] half;
	reg [7:0] rank;
	reg [8:0] scaled;
	begin
		case (eff)
			3'd1: select_top = (gi < prog);                                         //wipe down: new picture sweeps in from the top
			3'd2: select_top = (gi >= (WIPE_GRP_MAX - prog));                       //wipe up: new picture sweeps in from the bottom
			3'd3: select_top = ((gi[3:0] < (prog >> 4)) || (prog >= WIPE_GRP_MAX)); //blinds: 15 slats of 16 groups fill together
			3'd4: begin                                                            //split: reveals outwards from the centre line
				half = WIPE_GRP_MAX >> 1;
				//|gi - half| < (prog>>1) rewritten as two PARALLEL compares against
				//prog derived bounds, so gi no longer feeds a subtractor -> mux ->
				//compare series on the ext_mem_clk path. Equivalent for all gi and
				//K = prog>>1 >= 0, including gi == half and K == 0.
				select_top = ((gi > (half - (prog >> 1))) && (gi < (half + (prog >> 1)))) || (prog >= WIPE_GRP_MAX);
			end
			3'd5: begin                                                            //random bars: bit reversed group index fills in scrambled order
				rank   = {gi[0], gi[1], gi[2], gi[3], gi[4], gi[5], gi[6], gi[7]};
				scaled = prog + (prog >> 3);
				select_top = ((rank < scaled) || (prog >= WIPE_GRP_MAX));
			end
			3'd6: select_top = gi[0] ? (gi >= (WIPE_GRP_MAX - prog)) : (gi < prog);   //comb: even groups wipe down, odd groups wipe up
			default: select_top = 1'b0;                                            //0 and 7 are fade / idle: no band redirect
		endcase
	end
endfunction

assign base_top      = base_sel(read_addr_index_top_d1);
assign base_bot      = base_sel(read_addr_index_d1);
assign wipe_sel_diff = (read_addr_index_top_d1 != read_addr_index_d1);
//read_req_ack is registered in the state machine below and is guaranteed low on
//entry to S_ACK, so this marks the first of the several cycles S_ACK spends
//waiting for read_req to fall. Only the non idempotent band bookkeeping is gated
//by it; the address latch itself keeps loading on every S_ACK cycle as it always
//did.
assign s_ack_first = (state == S_ACK) && (read_req_ack == 1'b0);
//rd_burst_finish needs burst_cnt >= BURST_SIZE, which forces App_rd_en_d0 low, so
//a group boundary can never compete with the per word increment in the same
//cycle. burst_in_grp == 4 marks the fifth burst, i.e. the end of a 1280 word =
//two line group.
assign grp_boundary = rd_burst_finish && (burst_in_grp == 3'd4);
//Saturating one step of the band ramp. WIPE_GRP_STEP divides WIPE_GRP_MAX exactly
//at the default settings; the clamp is here so another step size cannot overshoot
//past the bottom of the panel.
assign progress_next = (progress >= (WIPE_GRP_MAX - WIPE_GRP_STEP)) ? WIPE_GRP_MAX
                                                                    : (progress + WIPE_GRP_STEP);
//Group 0's buffer for the frame about to be read, from the ramp value progress is
//frozen to at s_ack_first. cur_sel latches this; the address itself is driven from
//the registered cur_sel below, so select_top never enters the 21 bit address
//register's D path.
assign first_sel     = wipe_sel_diff ? select_top(9'd0, progress_next, effect_d1) : 1'b0;
//Group g+1's buffer, registered into next_sel_r every cycle. g_plus1 is the
//registered g+1, so this reads a flop output rather than an adder, keeping the
//leading incrementer off the ext_mem_clk path into next_sel_r.
assign next_sel_comb = select_top(g_plus1, progress, effect_d1);
//The start base comes from the REGISTERED cur_sel, not first_sel, so select_top
//stays off the ext_mem_clk address path. At s_ack_first cur_sel is still the old
//value, but S_ACK reloads the address on each of its several cycles, cur_sel holds
//first_sel from the second cycle on, and App_rd_en is low throughout S_ACK, so the
//first word emitted in S_READ_BURST sees the correct group-0 base. With the two
//selectors equal cur_sel is 0 and this is base_bot, the same 4 way mux on
//read_addr_index_d1 the module always latched: the no band case is bit for bit the
//original behaviour.
assign wipe_start_base = cur_sel ? base_top : base_bot;
//Redirect the address at a group boundary where the selection flips. next_sel_r is
//registered, so the +/- delta mux and the 21 bit incrementer that follow are the
//only logic on this path, exactly as the single crossing wipe was.
assign do_redirect = wipe_sel_diff && grp_boundary && (next_sel_r != cur_sel);
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
		effect_d0 <= 3'b000;
		effect_d1 <= 3'b000;
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
		effect_d0 <= effect;
		effect_d1 <= effect_d0;
		
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
		//Band redirect at a group aligned burst boundary where the buffer
		//selection flips. next_sel_r is registered, so only this 2:1 delta mux and
		//the 21 bit incrementer that follow sit on the ext_mem_clk address path,
		//exactly as the single crossing wipe did. Entering the top buffer adds
		//neg_wipe_delta, entering the bottom buffer adds wipe_delta; the net offset
		//after any number of flips is always a multiple of the buffer delta, so
		//group g reads from base_sel(g) + g * 1280 + w throughout the frame.
		else if(do_redirect)
			App_rd_addr_r <= App_rd_addr_r + (next_sel_r ? neg_wipe_delta : wipe_delta);
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
// Stage 5 band effect bookkeeping.
//
// This generalizes the old single crossing wipe. Where the wipe crossed the
// buffer boundary exactly once per frame, a band effect can flip the selection at
// any number of group boundaries, and select_top decides which side each group
// comes from. Six registers, all held stable across a frame read:
//
//   progress        the ramp position in two line groups. Loaded once per frame
//                   read at s_ack_first, frozen for the whole frame so every
//                   group evaluates select_top against the same value, and it
//                   accumulates across frames so the sweep walks down the panel.
//   g               the group index within the frame, 0..239. Advances at every
//                   group aligned burst boundary while the selectors disagree.
//   cur_sel         the buffer the CURRENT group reads from; always equals
//                   select_top(g, progress, effect). Latched from first_sel at
//                   s_ack_first and from next_sel_r at each flip.
//   next_sel_r      select_top(g+1, progress, effect), registered EVERY cycle so
//                   the address mux and the redirect test only ever see a flop
//                   output. This is what keeps select_top's combinational depth
//                   off the ext_mem_clk address register D path.
//   burst_in_grp    0..4; five 256 word bursts make one 1280 word = two line
//                   group. Free runs on every burst boundary, harmless when no
//                   band is running because do_redirect is gated by the
//                   selectors disagreeing.
//   wipe_delta      base_bottom - base_top, added when a flip steps down into the
//   neg_wipe_delta  base_top - base_bottom, added when a flip steps up. Both are
//                   precomputed at s_ack_first so the redirect is a 2:1 mux, not
//                   a subtractor, on the timing critical address path.
//
// next_sel_r is registered unconditionally; progress, g and cur_sel update only
// on the single cycle events s_ack_first and a group aligned rd_burst_finish.
// S_ACK itself lasts as long as read_req is held, several mem_clk cycles, so the
// non idempotent loads are gated by s_ack_first. When the two selectors are equal
// (fade or idle) progress and cur_sel are held at zero, g never advances, and
// next_sel_r stays select_top(1, 0, effect) = 0, so the whole engine is inert and
// the module reads exactly as it did before any wipe existed.
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
		progress       <= 9'd0;
		g              <= 9'd0;
		g_plus1        <= 9'd1;
		cur_sel        <= 1'b0;
		next_sel_r     <= 1'b0;
		burst_in_grp   <= 3'd0;
		wipe_delta     <= ZERO[ADDR_BITS - 1:0];
		neg_wipe_delta <= ZERO[ADDR_BITS - 1:0];
	end
	else
	begin
		//registered every cycle so the address mux only ever sees a flop
		next_sel_r <= next_sel_comb;
		//g_plus1 tracks g+1 as a flop so next_sel_comb reads a register, not an
		//adder. It lags by one cycle only in the single cycle after g advances at a
		//group boundary, and next_sel_r is consumed only at the next boundary ~1280
		//cycles later, by which time g_plus1 == g+1 has long settled.
		g_plus1 <= g + 9'd1;
		if(s_ack_first)
		begin
			burst_in_grp   <= 3'd0;
			g              <= 9'd0;
			wipe_delta     <= base_bot - base_top;
			neg_wipe_delta <= base_top - base_bot;
			if(wipe_sel_diff)
			begin
				progress <= progress_next;
				cur_sel  <= first_sel;
			end
			else
			begin
				progress <= 9'd0;
				cur_sel  <= 1'b0;
			end
		end
		else if(rd_burst_finish)
		begin
			if(burst_in_grp == 3'd4)
			begin
				burst_in_grp <= 3'd0;
				if(wipe_sel_diff)
				begin
					g       <= g + 9'd1;
					cur_sel <= next_sel_r;
				end
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
