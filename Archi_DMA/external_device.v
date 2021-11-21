`timescale 1ns/1ns
`include "const.v"

module external_device(
	clk, use_bus, idx,
	data,
	dma_interrupt
);

	input clk;
	input use_bus;
	input [3:0] idx;
	inout [`WORD_SIZE-1:0] data;
	output dma_interrupt;

	reg dma_interrupt;
	reg [`WORD_SIZE-1:0] device_data [11:0];

	initial begin
		dma_interrupt <= 0;
		device_data[0] <= 16'h0000;
		device_data[1] <= 16'h1111;
		device_data[2] <= 16'h2222;
		device_data[3] <= 16'h3333;
		device_data[4] <= 16'h4444;
		device_data[5] <= 16'h5555;
		device_data[6] <= 16'h6666;
		device_data[7] <= 16'h7777;
		device_data[8] <= 16'h8888;
		device_data[9] <= 16'h9999;
		device_data[10] <= 16'haaaa;
		device_data[11] <= 16'hbbbb;
	end

	assign data = (use_bus) ? device_data[idx] : 16'bz;

	reg [`WORD_SIZE-1:0] count_clk;

	initial begin
		count_clk <= 0;
	end
	always @(posedge(clk)) begin
		count_clk <= count_clk + 1;
		if (count_clk == 16'd10) begin
			dma_interrupt <= 1;
		end
		else begin
			dma_interrupt <= 0;
		end
	end
endmodule // external_device