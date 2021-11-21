`timescale 1ns/1ns
`include "const.v"

`define CHACHESETSIZE 159 // {16*4 + 1(dirty) + 1(valid) + 13(tag) =79}*2 + 1(LRU) = 159

module Instruction_Cache(
	clk, reset_n, read_IC, instruction_address, data1,
	cached_data, readM1, address1, IC_missed
);
	input clk;
	input reset_n;
	input read_IC;
	input [`WORD_SIZE-1:0] instruction_address;
	input [`WORD_SIZE-1:0] data1;

	output [`WORD_SIZE-1:0] cached_data;
	output readM1;
	output [`WORD_SIZE-1:0] address1;
	output IC_missed;
	
	reg [`WORD_SIZE-1:0] cached_data;
	reg readM1;
	reg [`WORD_SIZE-1:0] address1;
	reg IC_missed;

	reg [`WORD_SIZE-1:0] read_word_1;
	reg [`WORD_SIZE-1:0] read_word_2;
	reg [`WORD_SIZE-1:0] read_word_3;
	reg [`WORD_SIZE-1:0] read_word_4;
	reg [`WORD_SIZE-1:0] address_1;
	reg [`WORD_SIZE-1:0] address_2;
	reg [`WORD_SIZE-1:0] address_3;
	reg [`WORD_SIZE-1:0] address_4;

	reg [2:0] count;
	reg [`WORD_SIZE-1:0] h_count;
	reg [`WORD_SIZE-1:0] m_count;
	wire [`WORD_SIZE-1:0] t_count;
	assign t_count = h_count + m_count;
	reg buffered_idx;
	reg [12:0] buffered_tag;

	wire [12:0] tag;
	wire idx;
	wire [1:0] offset;
	assign tag = instruction_address[15:3];
	assign idx = instruction_address[2];
	assign offset = instruction_address[1:0];

	reg [`CHACHESETSIZE-1:0] I_cache [1:0]; //[158]LRU [157:145]tag [144]valid [143]dirty [142:79]data//[78:66]tag [65]valid [64]dirty [63:0]data

	wire hit;
	wire hit1;
	wire hit2;
	assign hit1 = ((tag == I_cache[idx][157:145]) && I_cache[idx][144]) ? 1 : 0; 
	assign hit2 = ((tag == I_cache[idx][78:66]) && I_cache[idx][65]) ? 1 : 0;
	assign hit = hit1 || hit2;

	always @(posedge(clk)) begin
		//reset
		if (!reset_n) begin
			I_cache[0] <= `CHACHESETSIZE'b0;
			I_cache[1] <= `CHACHESETSIZE'b0;
			cached_data <= 0;
			readM1 <= 0;
			address1 <= 0;
			IC_missed <= 0;
			read_word_1 <= 0;
			read_word_2 <= 0;
			read_word_3 <= 0;
			read_word_4 <= 0;
			address_1 <= 0;
			address_2 <= 0;
			address_3 <= 0;
			address_4 <= 0;
			count <= 0;
			h_count <= 0;
			m_count <= 0;
			buffered_idx <= 0;
			buffered_tag <= 0;
		end
		//when missed
		else if (count == 3'd5) begin
			address1 <= address_1;
			count <= count - 1;
			readM1 <= 1;
		end
		else if (count == 3'd4) begin
			read_word_1 <= data1;
			address1 <= address_2;
			count <= count - 1;
			readM1 <= 1;
		end
		else if (count == 3'd3) begin
			read_word_2 <= data1;
			address1 <= address_3;
			count <= count - 1;
			readM1 <= 1;
		end
		else if (count == 3'd2) begin
			read_word_3 <= data1;
			address1 <= address_4;
			count <= count - 1;
			readM1 <= 1;
		end
		else if (count == 3'd1) begin
			read_word_4 <= data1;
			count <= count - 1;
		end
		else if (count == 3'd0) begin
			//update cache
			if (!I_cache[buffered_idx][144]) begin //if line1 is invalid
				I_cache[buffered_idx][157:145] <= buffered_tag;
				I_cache[buffered_idx][144] <= 1;
				I_cache[buffered_idx][143] <= 0;
				I_cache[buffered_idx][142:127] <= read_word_1;
				I_cache[buffered_idx][126:111] <= read_word_2;
				I_cache[buffered_idx][110:95] <= read_word_3;
				I_cache[buffered_idx][94:79] <= read_word_4;
			end
			else if (!I_cache[buffered_idx][65])begin //if line0 is invalid
				I_cache[buffered_idx][78:66] <= buffered_tag;
				I_cache[buffered_idx][65] <= 1;
				I_cache[buffered_idx][64] <= 0;
				I_cache[buffered_idx][63:48] <= read_word_1;
				I_cache[buffered_idx][47:32] <= read_word_2;
				I_cache[buffered_idx][31:16] <= read_word_3;
				I_cache[buffered_idx][15:0] <= read_word_4;
			end
			// LRU
			else if (!I_cache[buffered_idx][158]) begin //line0 is recently used, so replace 1
				I_cache[buffered_idx][157:145] <= buffered_tag;
				I_cache[buffered_idx][144] <= 1;
				I_cache[buffered_idx][143] <= 0;
				I_cache[buffered_idx][142:127] <= read_word_1;
				I_cache[buffered_idx][126:111] <= read_word_2;
				I_cache[buffered_idx][110:95] <= read_word_3;
				I_cache[buffered_idx][94:79] <= read_word_4;
			end
			else begin
				I_cache[buffered_idx][78:66] <= buffered_tag;
				I_cache[buffered_idx][65] <= 1;
				I_cache[buffered_idx][64] <= 0;
				I_cache[buffered_idx][63:48] <= read_word_1;
				I_cache[buffered_idx][47:32] <= read_word_2;
				I_cache[buffered_idx][31:16] <= read_word_3;
				I_cache[buffered_idx][15:0] <= read_word_4;
			end
			IC_missed <= 0;
			count <= count - 1;
		end
	end

	always @(negedge(clk)) begin
		if(read_IC && !IC_missed) begin
			//hit
			if (hit) begin
				if (hit1) begin
					case(offset)
						2'b00: cached_data <= I_cache[idx][142:127];
						2'b01: cached_data <= I_cache[idx][126:111];
						2'b10: cached_data <= I_cache[idx][110:95];
						2'b11: cached_data <= I_cache[idx][94:79];
					endcase
					I_cache[idx][158] <= 1;
				end
				else begin
					case(offset)
						2'b00: cached_data <= I_cache[idx][63:48];
						2'b01: cached_data <= I_cache[idx][47:32];
						2'b10: cached_data <= I_cache[idx][31:16];
						2'b11: cached_data <= I_cache[idx][15:0];
					endcase
					I_cache[idx][158] <= 0;
				end
				h_count <= h_count + 1;
			end

			//miss
			else begin
				address_1[15:2] <= instruction_address[15:2];
				address_2[15:2] <= instruction_address[15:2];
				address_3[15:2] <= instruction_address[15:2];
				address_4[15:2] <= instruction_address[15:2];
				address_1[1:0] <= 2'b00;
				address_2[1:0] <= 2'b01;
				address_3[1:0] <= 2'b10;
				address_4[1:0] <= 2'b11;
				IC_missed <= 1;
				count <= 3'd5;
				buffered_idx <= idx;
				buffered_tag <= tag;
				m_count <= m_count + 1;
			end
		end
	end
endmodule


module Data_Cache(
	clk, reset_n, read_DC, write_DC, mem_address, write_data, data2,
	cached_data, readM2, writeM2, address2, DC_missed
);
	input clk;
	input reset_n;
	input read_DC;
	input write_DC;
	input [`WORD_SIZE-1:0] mem_address;
	input [`WORD_SIZE-1:0] write_data;

	inout [`WORD_SIZE-1:0] data2;

	output [`WORD_SIZE-1:0] cached_data;
	output readM2;
	output writeM2;
	output [`WORD_SIZE-1:0] address2;
	output DC_missed;
	
	reg [`WORD_SIZE-1:0] cached_data;
	reg readM2;
	reg writeM2;
	reg [`WORD_SIZE-1:0] address2;
	reg DC_missed;

	reg [`WORD_SIZE-1:0] read_word_1;
	reg [`WORD_SIZE-1:0] read_word_2;
	reg [`WORD_SIZE-1:0] read_word_3;
	reg [`WORD_SIZE-1:0] read_word_4;
	reg [`WORD_SIZE-1:0] address_1;
	reg [`WORD_SIZE-1:0] address_2;
	reg [`WORD_SIZE-1:0] address_3;
	reg [`WORD_SIZE-1:0] address_4;

	reg [2:0] count;
	reg [`WORD_SIZE-1:0] h_count;
	reg [`WORD_SIZE-1:0] m_count;
	wire [`WORD_SIZE-1:0] t_count;
	assign t_count = h_count + m_count;

	wire [12:0] tag;
	wire idx;
	wire [1:0] offset;
	assign tag = mem_address[15:3];
	assign idx = mem_address[2];
	assign offset = mem_address[1:0];

	reg buffered_idx;
	reg [12:0] buffered_tag;

	reg [`CHACHESETSIZE-1:0] D_cache [1:0]; //[158]LRU [157:145]tag [144]valid [143]dirty [142:79]data//[78:66]tag [65]valid [64]dirty [63:0]data

	reg DC_W;

	assign data2 = (writeM2) ? write_data : 16'bz;

	wire hit;
	wire hit1;
	wire hit2;
	assign hit1 = (tag == D_cache[idx][157:145] && D_cache[idx][144]) ? 1 : 0; 
	assign hit2 = (tag == D_cache[idx][78:66] && D_cache[idx][65]) ? 1 : 0;
	assign hit = hit1 || hit2;

	reg missed;

	always @(posedge(clk)) begin
		//reset
		if (!reset_n) begin
			D_cache[0] <= `CHACHESETSIZE'b0;
			D_cache[1] <= `CHACHESETSIZE'b0;
			cached_data <= 0;
			readM2 <= 0;
			writeM2 <= 0;
			address2 <= 0;
			DC_missed <= 0;
			read_word_1 <= 0;
			read_word_2 <= 0;
			read_word_3 <= 0;
			read_word_4 <= 0;
			address_1 <= 0;
			address_2 <= 0;
			address_3 <= 0;
			address_4 <= 0;
			count <= 0;
			h_count <= 0;
			m_count <= 0;
			DC_W <= 0;
			missed <= 0;
			buffered_idx <= 0;
			buffered_tag <= 0;
		end
		else if (count == 3'd7) begin
			missed <= 0;
		end
		//when missed
		else if (count == 3'd5) begin
			if (!DC_W) begin
				address2 <= address_1;
				readM2 <= 1;
			end
			count <= count - 1;
		end
		else if (count == 3'd4) begin
			if (!DC_W) begin
				read_word_1 <= data2;
				address2 <= address_2;
				readM2 <= 1;
			end
			count <= count - 1;
		end
		else if (count == 3'd3) begin
			if (!DC_W) begin
				read_word_2 <= data2;
				address2 <= address_3;
				readM2 <= 1;
			end
			count <= count - 1;
		end
		else if (count == 3'd2) begin
			if (!DC_W) begin
				read_word_3 <= data2;
				address2 <= address_4;
				readM2 <= 1;
			end
			count <= count - 1;
		end
		else if (count == 3'd1) begin
			readM2 <= 0;
			if (!DC_W) begin
				read_word_4 <= data2;
			end
			else begin
				address2 <= mem_address;
				writeM2 <= 1;
			end
			count <= count - 1;
		end
		else if (count == 3'd0) begin
			if (!DC_W && DC_missed) begin
				//update cache
				if (!D_cache[buffered_idx][144]) begin //if line1 is invalid
					D_cache[buffered_idx][157:145] <= buffered_tag;
					D_cache[buffered_idx][144] <= 1;
					D_cache[buffered_idx][143] <= 0;
					D_cache[buffered_idx][142:127] <= read_word_1;
					D_cache[buffered_idx][126:111] <= read_word_2;
					D_cache[buffered_idx][110:95] <= read_word_3;
					D_cache[buffered_idx][94:79] <= read_word_4;
				end
				else if (!D_cache[buffered_idx][65])begin //if line0 is invalid
					D_cache[buffered_idx][78:66] <= buffered_tag;
					D_cache[buffered_idx][65] <= 1;
					D_cache[buffered_idx][64] <= 0;
					D_cache[buffered_idx][63:48] <= read_word_1;
					D_cache[buffered_idx][47:32] <= read_word_2;
					D_cache[buffered_idx][31:16] <= read_word_3;
					D_cache[buffered_idx][15:0] <= read_word_4;
				end
				// LRU
				else if (!D_cache[buffered_idx][158]) begin //line0 is recently used, so replace 1
					D_cache[buffered_idx][157:145] <= buffered_tag;
					D_cache[buffered_idx][144] <= 1;
					D_cache[buffered_idx][143] <= 0;
					D_cache[buffered_idx][142:127] <= read_word_1;
					D_cache[buffered_idx][126:111] <= read_word_2;
					D_cache[buffered_idx][110:95] <= read_word_3;
					D_cache[buffered_idx][94:79] <= read_word_4;
				end
				else begin
					D_cache[buffered_idx][78:66] <= buffered_tag;
					D_cache[buffered_idx][65] <= 1;
					D_cache[buffered_idx][64] <= 0;
					D_cache[buffered_idx][63:48] <= read_word_1;
					D_cache[buffered_idx][47:32] <= read_word_2;
					D_cache[buffered_idx][31:16] <= read_word_3;
					D_cache[buffered_idx][15:0] <= read_word_4;
				end
			end
			writeM2 <= 0;
			DC_missed <= 0;
			count <= count - 1;
			DC_W <= 0;
		end
	end

	always @(negedge(clk)) begin
		//read
		if(read_DC && !DC_missed) begin
			//hit
			if (hit) begin
				if (hit1) begin
					case(offset)
						2'b00: cached_data <= D_cache[idx][142:127];
						2'b01: cached_data <= D_cache[idx][126:111];
						2'b10: cached_data <= D_cache[idx][110:95];
						2'b11: cached_data <= D_cache[idx][94:79];
					endcase
					D_cache[idx][158] <= 1;
				end
				else begin
					case(offset)
						2'b00: cached_data <= D_cache[idx][63:48];
						2'b01: cached_data <= D_cache[idx][47:32];
						2'b10: cached_data <= D_cache[idx][31:16];
						2'b11: cached_data <= D_cache[idx][15:0];
					endcase
					D_cache[idx][158] <= 0;
				end
				h_count <= h_count + 1;
			end
			//miss
			else begin
				address_1[15:2] <= mem_address[15:2];
				address_2[15:2] <= mem_address[15:2];
				address_3[15:2] <= mem_address[15:2];
				address_4[15:2] <= mem_address[15:2];
				address_1[1:0] <= 2'b00;
				address_2[1:0] <= 2'b01;
				address_3[1:0] <= 2'b10;
				address_4[1:0] <= 2'b11;
				DC_missed <= 1;
				missed <= 1;
				count <= 3'd5;
				m_count <= m_count + 1;
				buffered_idx <= idx;
				buffered_tag <= tag;
			end
		end
		//write
		else if(write_DC && !DC_missed && !missed) begin
			//write cache
			if (hit) begin
				if (hit1) begin
					case(offset)
						2'b00: D_cache[idx][142:127] <= write_data;
						2'b01: D_cache[idx][126:111] <= write_data;
						2'b10: D_cache[idx][110:95] <= write_data;
						2'b11: D_cache[idx][94:79] <= write_data;
					endcase
					D_cache[idx][158] <= 1;
				end
				else begin
					case(offset)
						2'b00: D_cache[idx][63:48] <= write_data;
						2'b01: D_cache[idx][47:32] <= write_data;
						2'b10: D_cache[idx][31:16] <= write_data;
						2'b11: D_cache[idx][15:0] <= write_data;
					endcase
					D_cache[idx][158] <= 0;
				end
			end
			//write mem
			DC_missed <= 1;
			missed <= 1;
			DC_W <= 1;
			count <= 3'd5;
			buffered_idx <= idx;
			buffered_tag <= tag;
		end
	end
endmodule


module cache(
	clk, reset_n, read_IC, instruction_address, data1, read_DC, write_DC, mem_address, write_data, data2,
	IC_cached_data, readM1, address1, IC_missed, DC_cached_data, readM2, writeM2, address2, DC_missed
);
	input clk;
	input reset_n;
	input read_IC;
	input [`WORD_SIZE-1:0] instruction_address;
	input [`WORD_SIZE-1:0] data1;
	input read_DC;
	input write_DC;
	input [`WORD_SIZE-1:0] mem_address;
	input [`WORD_SIZE-1:0] write_data;
	inout [`WORD_SIZE-1:0] data2;

	output [`WORD_SIZE-1:0] IC_cached_data;
	output readM1;
	output [`WORD_SIZE-1:0] address1;
	output IC_missed;
	output [`WORD_SIZE-1:0] DC_cached_data;
	output readM2;
	output writeM2;
	output [`WORD_SIZE-1:0] address2;
	output DC_missed;

	Instruction_Cache I_cache(clk, reset_n, read_IC, instruction_address, data1, IC_cached_data, readM1, address1, IC_missed);
	Data_Cache D_cache(clk, reset_n, read_DC, write_DC, mem_address, write_data, data2, DC_cached_data, readM2, writeM2, address2, DC_missed);
endmodule // cache
