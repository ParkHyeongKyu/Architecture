`timescale 1ns/1ns
`include "const.v"

module DMAC(
	clk, length, address, data, BG,
	BR, interrupt, writeMem, use_bus, state, address2Wr
);

	input clk;
	input length;
	input [`WORD_SIZE-1:0] address;
	input [`WORD_SIZE-1:0] data;
	input BG;
	
	output BR;
	output interrupt;
	output writeMem;
	output use_bus;
	output [3:0] state;
	output [`WORD_SIZE-1:0] address2Wr;

	reg BR;
	reg interrupt;
	reg writeMem;
	reg use_bus;
	reg [3:0] state;
	reg [`WORD_SIZE-1:0] address2Wr;

	initial begin
		BR <= 0;
		interrupt <= 0;
		writeMem <= 0;
		use_bus <= 0;
		state <= 4'd13;
		address2Wr <= 16'hFFFF;
	end

	always @(posedge(clk)) begin
		if (length) begin
			BR <= 1;
			state <= 0;
			address2Wr <= address;
		end
	end

	always @(posedge(clk)) begin
		if (BG) begin
			if (state < 4'd12) begin
				use_bus <= 1;
				writeMem <= 1;
				state <= state + 1;
				address2Wr <= address2Wr + 1;
			end
			if (state == 4'd12) begin
				BR <= 0;
				use_bus <= 0;
				writeMem <= 0;
				state <= state + 1;
				interrupt <= 1;
			end
			
		end
		else begin
			interrupt <= 0;
		end
	end
endmodule // DMAC