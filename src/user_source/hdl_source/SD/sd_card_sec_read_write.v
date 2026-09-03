
module sd_card_sec_read_write
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // spi clk speed = clk speed /((SPI_LOW_SPEED_DIV + 2) * 2 )
	parameter  SPI_HIGH_SPEED_DIV = 0           // spi clk speed = clk speed /((SPI_HIGH_SPEED_DIV + 2) * 2 )
	                                            // 0 -> 25MHz, 4 -> 8.3MHz at a 100MHz clk. Live knob: S_WAIT_READ_WRITE
	                                            // loads it into spi_clk_div, so changing it here actually changes SCK.
)
(
	input            clk,
	input            rst,
	output reg       sd_init_done,
	input            sd_sec_read,
	input[31:0]      sd_sec_read_addr,
	output[7:0]      sd_sec_read_data,
	output           sd_sec_read_data_valid,
	output           sd_sec_read_end,
	input            sd_sec_write,
	input[31:0]      sd_sec_write_addr,
	input[7:0]       sd_sec_write_data,
	output           sd_sec_write_data_req,
	output           sd_sec_write_end,

	output reg[15:0] spi_clk_div,
	output reg       cmd_req,
	input            cmd_req_ack,
	input            cmd_req_error,
	output reg[47:0] cmd,
	output reg[7:0]  cmd_r1,
	output reg[15:0] cmd_data_len,
	output reg       block_read_req,
	input            block_read_valid,
	input[7:0]       block_read_data,
	input            block_read_req_ack,
	output reg       block_write_req,
	output[7:0]      block_write_data,
	input            block_write_data_rd,
	input            block_write_req_ack
);
reg[7:0] read_data;
reg[31:0] timer;
// A missed 0xFE start token is a bit error on MISO, not a dead card: no payload
// byte was delivered, so re-issuing CMD17 for the same sec_addr recovers with
// nothing lost and nothing duplicated. Retry before falling back to skipping the
// sector. Skipping is survivable but wrong -- ST_LOAD_DATA ends on
// bmp_len_cnt >= file_len rather than on a sector count, so a dropped sector
// shifts every later pixel by 512 bytes, and if it happens during ST_LOAD_HDR the
// headerless sector fails header_match_r, raises load_failed, and sd_card_bmp
// drops that image permanently because next_load_idx has already advanced.
// Budget: 3 attempts x 100ms + 2 drain gaps x 82us = 300.2ms worst case per
// sector, inside the 1s load_stall_cnt watchdog in sd_card_bmp.v with 3.3x to
// spare.
reg[1:0]                       rd_retry;
localparam [1:0]               RD_RETRY_MAX = 2'd2;
// One SPI byte costs 17*div + 36 sys_clk (IDLE 1, 16 x (DCLK_IDLE div+1 plus
// DCLK_EDGE 1), LAST_HALF_CYCLE div+1, ACK 1, ACK_WAIT 1) and the master needs
// two more to walk ACK -> ACK_WAIT -> IDLE. A retry has to wait that long before
// re-issuing CMD17. If it does not, sd_card_cmd reaches S_CMD_PRE while the byte
// that was in flight when the timeout fired is still shifting, sees that byte's
// spi_wr_ack on its very first S_CMD_PRE cycle, takes the branch that skips
// transmitting, and the CS-high deselect byte never goes out. CS then stays high
// only for the tail of the leftover byte -- about 6 SCK periods at div=0 -- which
// is under the 8 clocks the SD spec lets a card keep driving the line after
// deselect. Waiting a full byte period makes the deselect unambiguous.
// Sized off SPI_LOW_SPEED_DIV so it holds on any error path including one taken
// during init at the slow divider: 2^13 = 8192 >= 17*248 + 38 = 4254. Widen
// GAP_AW if SPI_LOW_SPEED_DIV ever goes above 479.
localparam integer             GAP_AW = 14;
reg[GAP_AW-1:0]                rd_gap;

localparam S_IDLE               = 0;
localparam S_CMD0               = 1;
localparam S_CMD8               = 2;
localparam S_CMD55              = 3;
localparam S_CMD41              = 4;
localparam S_CMD17              = 5;
localparam S_READ               = 6;
localparam S_CMD24              = 7;
localparam S_WRITE              = 8;
localparam S_ERR                = 14;
localparam S_WRITE_END          = 15;
localparam S_READ_END           = 16;
localparam S_WAIT_READ_WRITE    = 17;
localparam S_CMD16              = 18;
localparam S_RETRY_GAP          = 19;

reg[4:0]                       state;
reg[31:0]                      sec_addr;
assign sd_sec_read_data_valid = (state == S_READ) && block_read_valid;
assign sd_sec_read_data = block_read_data;
assign sd_sec_read_end = (state == S_READ_END);
assign sd_sec_write_data_req = (state == S_WRITE) && block_write_data_rd;
assign block_write_data = sd_sec_write_data;
assign sd_sec_write_end = (state == S_WRITE_END);

always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		cmd_req <= 1'b0;
		cmd_data_len <= 16'd0;
		cmd_r1 <= 8'd0;
		cmd <= 48'd0;
		spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
		block_write_req <= 1'b0;
		block_read_req <= 1'b0;
		sec_addr <= 32'd0;
		sd_init_done <= 1'b0;
		rd_retry <= 2'd0;
		rd_gap <= {GAP_AW{1'b0}};
	end
	else
		case(state)
			S_IDLE:
			begin
				state <= S_CMD0;
				sd_init_done <= 1'b0;
				spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
			end
			S_CMD0:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD8;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
				end
			end
			S_CMD8:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD55;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd4;
					cmd_r1 <= 8'h01;
					cmd <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
				end
			end
			S_CMD55:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD41;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD41:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD16;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD16:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WAIT_READ_WRITE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd16,32'd512,8'hff};
				end
			end			
			S_WAIT_READ_WRITE:
			begin
				if(sd_sec_write ==  1'b1)
				begin
					state <= S_CMD24;
					sec_addr <= sd_sec_write_addr;
				end
				else if(sd_sec_read == 1'b1)
				begin
					state <= S_CMD17;
					sec_addr <= sd_sec_read_addr;
					rd_retry <= RD_RETRY_MAX;
				end

				//Must be the parameter, never a literal. Hard coding 16'd0 here overrides
				//the SPI_HIGH_SPEED_DIV that S_CMD41/S_CMD16 loaded one state earlier and
				//turns the top level knob into a no-op, which is how a 25MHz SPI with no
				//read timeout ever got shipped once already. Drop it to 4 (8.3MHz) if a
				//card misses tokens often enough for the retries to visibly slow the load.
				spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
			end
			S_CMD24:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WRITE;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd24,sec_addr,8'hff};

				end
			end
			S_WRITE:
			begin
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req <= 1'b0;
					state <= S_WRITE_END;
				end
				else
					block_write_req <= 1'b1;
			end
			S_CMD17:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_READ;
					cmd_req <= 1'b0;
				end
				else if(cmd_req_ack)
				begin
					//ack with error: the card rejected CMD17. sec_addr is untouched, so
					//re-arming through S_RETRY_GAP re-issues the same command for the same
					//sector once the SPI has drained.
					cmd_req <= 1'b0;
					block_read_req <= 1'b0;
					if(rd_retry != 2'd0)
					begin
						rd_retry <= rd_retry - 2'd1;
						rd_gap <= {GAP_AW{1'b0}};
						state <= S_RETRY_GAP;
					end
					else
						state <= S_READ_END;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd17,sec_addr,8'hff};
				end
			end
			S_READ:
			begin
				if(block_read_req_ack)
				begin
					state <= S_READ_END;
					block_read_req <= 1'b0;
				end
				else if(cmd_req_error)
				begin
					//sd_card_cmd timed out. Drop block_read_req in this same cycle: it is
					//on its way back to S_WAIT, and if the request is still asserted when
					//it arrives it re-enters S_READ_WAIT and burns another whole timeout
					//without this layer ever regaining control.
					block_read_req <= 1'b0;
					if(rd_retry != 2'd0)
					begin
						rd_retry <= rd_retry - 2'd1;
						rd_gap <= {GAP_AW{1'b0}};
						state <= S_RETRY_GAP;
					end
					else
						state <= S_READ_END;
				end
				else
				begin
					block_read_req <= 1'b1;
				end
			end
			S_RETRY_GAP:
			begin
				//Idle with both requests low until the byte that was in flight when the
				//error fired has completed and spi_master is back in IDLE, so that the
				//re-issued command gets a real CS-high deselect byte in front of it.
				cmd_req <= 1'b0;
				block_read_req <= 1'b0;
				//Saturating, so the done test stays a single LUT input on the top bit
				//instead of a 14 bit compare in the sd_card_clk domain.
				if(rd_gap[GAP_AW-1])
					state <= S_CMD17;
				else
					rd_gap <= rd_gap + {{(GAP_AW-1){1'b0}}, 1'b1};
			end
			S_WRITE_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			S_READ_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule