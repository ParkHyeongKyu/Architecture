`timescale 1ns/1ns
`include "const.v"


module ALU(
	A, B, ALU_func,
	result
);
	input [`WORD_SIZE-1:0] A;
	input [`WORD_SIZE-1:0] B;
	input [2:0] ALU_func;

	output [`WORD_SIZE-1:0] result;

	reg [`WORD_SIZE-1:0] result;
	reg signed [`WORD_SIZE-1:0] AS;

	always @(*) begin
		case (ALU_func)
			`ALU_FUNC_ADD: begin
				result = A + B;
			end
			 `ALU_FUNC_SUB: begin
				result = A - B;
			end
			`ALU_FUNC_AND: begin
				result = A & B;
			end
			`ALU_FUNC_ORR: begin
				result = A | B;
			end
			`ALU_FUNC_NOT: begin
				result = ~A;
			end
			`ALU_FUNC_TCP: begin
				result = ~A + 1;
			end
			`ALU_FUNC_SHL: begin
				result = A <<< 1;
			end
			`ALU_FUNC_SHR: begin
				AS = A;
				result = AS >>> 1;
			end
		endcase
	end
endmodule // ALU


module ADDER(
	A, B,
	result
);
	input [`WORD_SIZE-1:0] A;
	input [`WORD_SIZE-1:0] B;

	output [`WORD_SIZE-1:0] result;

	wire [`WORD_SIZE-1:0] result;

	assign result = A + B;
endmodule // ADDER

module imm_gen(
	opcode, imm_offset, current_pc, target,
	imm_16
);
	input [3:0] opcode;
	input [7:0] imm_offset;
	input [`WORD_SIZE-1:0] current_pc;
	input [11:0] target;

	output [`WORD_SIZE-1:0] imm_16;

	reg [`WORD_SIZE-1:0] imm_16;

	integer i;

	always @(*) begin
		case (opcode)
			//Rtype: don't use
			`ALU_OP: begin
				imm_16 = 16'b0;
			end
			//Itype:
			`ADI_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`ORI_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = 0;
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`LHI_OP: begin
				for(i=0; i<8; i=i+1) begin
				imm_16[i] = 0;
				end
				imm_16[15:8] = imm_offset[7:0];
			end
			`LWD_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`SWD_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`BNE_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`BEQ_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`BGZ_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			`BLZ_OP: begin
				for(i=15; i>7; i=i-1) begin
				imm_16[i] = imm_offset[7];
				end
				imm_16[7:0] = imm_offset[7:0];
			end
			//Jtype: 
			`JMP_OP: begin
				for(i=15; i>11; i=i-1) begin
				imm_16[i] = current_pc[i];
				end
				imm_16[11:0] = target[11:0];
			end
			`JAL_OP: begin
				for(i=15; i>11; i=i-1) begin
				imm_16[i] = current_pc[i];
				end
				imm_16[11:0] = target[11:0];
			end 
		endcase
	end
endmodule // imm_gen


module GPR(
	clk, reset_n, RegWrite, read_reg_1, read_reg_2, wrtie_reg, write_data, wb_is_real,
	read_data_1, read_data_2
);
	input clk;
	input reset_n;
	input RegWrite;
	input [1:0] read_reg_1;
	input [1:0] read_reg_2;
	input [1:0] wrtie_reg;
	input [`WORD_SIZE-1:0] write_data;
	input wb_is_real;

	output [`WORD_SIZE-1:0] read_data_1;
	output [`WORD_SIZE-1:0] read_data_2;

	reg [`WORD_SIZE-1:0] REGISTER [`NUM_REGS-1:0];

	assign read_data_1 = REGISTER[read_reg_1];
	assign read_data_2 = REGISTER[read_reg_2];

	integer i;
	// general
	always @(negedge(clk)) begin
		if (!reset_n) begin
			for (i = 0; i < `NUM_REGS; i = i + 1) begin
				REGISTER[i] <= 16'b0;
			end
		end
		else begin
			if (RegWrite == 1 && wb_is_real == 1) begin
				REGISTER[wrtie_reg] <= write_data;
			end
		end
	end
endmodule // GPR


module program_counter(
	clk, reset_n, pc_update, next_pc,
	current_pc
);
	input clk;
	input reset_n;
	input pc_update;
	input [`WORD_SIZE-1:0] next_pc;

	output [`WORD_SIZE-1:0] current_pc;

	reg [`WORD_SIZE-1:0] current_pc;

	// general
	always @(posedge(clk)) begin
		if (!reset_n) begin
			current_pc <= 0;
		end
		else begin
			if (pc_update == 1) begin
				current_pc <= next_pc;
			end
		end
	end
endmodule // program_counter

module MUX2_2(
	in0, in1, flag,
	out
);
	input [1:0] in0;
	input [1:0] in1;
	input flag;

	output [1:0] out;

	wire [1:0] out;

	assign out = (flag == 0) ? in0 : in1;
endmodule // MUX2_2


module MUX16_2(
	in0, in1, flag,
	out
);
	input [`WORD_SIZE-1:0] in0;
	input [`WORD_SIZE-1:0] in1;
	input flag;

	output [`WORD_SIZE-1:0] out;

	wire [`WORD_SIZE-1:0] out;

	assign out = (flag == 0) ? in0 : in1;
endmodule // MUX16_2


module MUX16_4(
	in0, in1, in2, in3, flag,
	out
);
	input [`WORD_SIZE-1:0] in0;
	input [`WORD_SIZE-1:0] in1;
	input [`WORD_SIZE-1:0] in2;
	input [`WORD_SIZE-1:0] in3;
	input [1:0] flag;

	output [`WORD_SIZE-1:0] out;

	wire [`WORD_SIZE-1:0] out;

	assign out = (flag == 2'b00) ? in0 :
		(flag == 2'b01) ? in1 :
		(flag == 2'b10) ? in2 : in3;
endmodule // MUX16_4

