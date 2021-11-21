`timescale 1ns/1ns
`include "const.v"

module IFandID(
	clk, reset_n, IFIDWrite, IF_PC_plus_one, IF_instr, IF_current_pc,
	ID_PC_plus_one, ID_opcode, ID_rs, ID_rt, ID_rd, ID_funct, ID_imm_offset, ID_target, ID_current_pc
);
	input clk;
	input reset_n;
	input IFIDWrite;
	input [`WORD_SIZE-1:0] IF_PC_plus_one;
	input [`WORD_SIZE-1:0] IF_instr;
	input [`WORD_SIZE-1:0] IF_current_pc;

	output [`WORD_SIZE-1:0] ID_PC_plus_one;
	output [3:0] ID_opcode;
	output [1:0] ID_rs;
	output [1:0] ID_rt;
	output [1:0] ID_rd;
	output [5:0] ID_funct;
	output [7:0] ID_imm_offset;
	output [11:0] ID_target;
	output [`WORD_SIZE-1:0] ID_current_pc;

	reg [`WORD_SIZE-1:0] ID_PC_plus_one;
	reg [`WORD_SIZE-1:0] inst;
	reg [`WORD_SIZE-1:0] ID_current_pc;
	reg ID_valid;

	assign ID_opcode = inst[15:12];
	assign ID_rs = inst[11:10];
	assign ID_rt = inst[9:8];
	assign ID_rd = inst[7:6];
	assign ID_funct = inst[5:0];
	assign ID_imm_offset = inst[7:0];
	assign ID_target = inst[11:0];

	always @(posedge(clk)) begin
		if (!reset_n) begin
			ID_PC_plus_one <= 0;
			inst <= 0;
			ID_current_pc <= 0;
			ID_valid <= 0;
		end
		else begin
			if (IFIDWrite == 1 && ID_valid) begin
				ID_PC_plus_one <= IF_PC_plus_one;
				inst <= IF_instr;
				ID_current_pc <= IF_current_pc;
			end
			else if (!ID_valid) begin //??? ?? ???
				ID_valid <= 1;
			end
		end
	end
endmodule

module IDandEX(
	clk, reset_n, is_halt, is_real, ID_PC_plus_one, ID_read_rs, ID_read_rt, ID_opcode, ID_funct, ID_imm16, ID_rs, ID_rt, ID_write_reg, ID_EX_ALU_src_A, ID_EX_ALU_src_B, ID_MEM_readM, ID_MEM_writeM, ID_WB_memtoreg, ID_WB_reg_write, ID_use_rs, ID_use_rt, setZero, ID_is_J_type, ID_is_JR_type,
	EX_is_halt, is_empty, EX_is_real, EX_PC_plus_one, EX_read_rs, EX_read_rt, EX_opcode, EX_funct, EX_imm16, EX_rs, EX_rt, EX_write_reg, EX_EX_ALU_src_A, EX_EX_ALU_src_B, EX_MEM_readM, EX_MEM_writeM, EX_WB_memtoreg, EX_WB_reg_write, EX_use_rs, EX_use_rt,  EX_is_J_type, EX_is_JR_type
);
	input clk;
	input reset_n;
	input is_halt;
	input is_real;
	input [`WORD_SIZE-1:0] ID_PC_plus_one;
	input [`WORD_SIZE-1:0] ID_read_rs;
	input [`WORD_SIZE-1:0] ID_read_rt;
	input [3:0] ID_opcode;
	input [5:0] ID_funct;
	input [`WORD_SIZE-1:0] ID_imm16;
	input [1:0] ID_rs;
	input [1:0] ID_rt;
	input [1:0] ID_write_reg;
	input ID_EX_ALU_src_A;
	input ID_EX_ALU_src_B;
	input ID_MEM_readM;
	input ID_MEM_writeM;
	input [1:0] ID_WB_memtoreg;
	input ID_WB_reg_write;
	input ID_use_rs;
	input ID_use_rt;
	input setZero;
	input ID_is_J_type;
	input ID_is_JR_type;

	output EX_is_halt;
	output is_empty;
	wire is_empty;
	output EX_is_real;
	output [`WORD_SIZE-1:0] EX_PC_plus_one;
	output [`WORD_SIZE-1:0] EX_read_rs;
	output [`WORD_SIZE-1:0] EX_read_rt;
	output [3:0] EX_opcode;
	output [5:0] EX_funct;
	output [`WORD_SIZE-1:0] EX_imm16;
	output [1:0] EX_rs;
	output [1:0] EX_rt;
	output [1:0] EX_write_reg;
	output EX_EX_ALU_src_A;
	output EX_EX_ALU_src_B;
	output EX_MEM_readM;
	output EX_MEM_writeM;
	output [1:0] EX_WB_memtoreg;
	output EX_WB_reg_write;
	output EX_use_rs;
	output EX_use_rt;
	output EX_is_J_type;
	output EX_is_JR_type;

	reg EX_is_halt;
	reg [1:0] temp;
	assign is_empty = temp[0] || temp[1];
	reg EX_is_real;
	reg [`WORD_SIZE-1:0] EX_PC_plus_one;
	reg [`WORD_SIZE-1:0] EX_read_rs;
	reg [`WORD_SIZE-1:0] EX_read_rt;
	reg [3:0] EX_opcode;
	reg [5:0] EX_funct;
	reg [`WORD_SIZE-1:0] EX_imm16;
	reg [1:0] EX_rs;
	reg [1:0] EX_rt;
	reg [1:0] EX_write_reg;
	reg EX_EX_ALU_src_A;
	reg EX_EX_ALU_src_B;
	reg EX_MEM_readM;
	reg EX_MEM_writeM;
	reg [1:0] EX_WB_memtoreg;
	reg EX_WB_reg_write;
	reg EX_use_rs;
	reg EX_use_rt;
	reg EX_is_J_type;
	reg EX_is_JR_type;

	always @(posedge(clk)) begin
		temp <= temp >> 1;
	end

	always @(posedge(clk)) begin
		if (!reset_n) begin
			EX_PC_plus_one <= 0;
			EX_read_rs <= 0;
			EX_read_rt <= 0;
			EX_opcode <= 0;
			EX_funct <= 0;
			EX_imm16 <= 0;
			EX_rs <= 0;
			EX_rt <= 0;
			EX_write_reg <= 0;
			EX_EX_ALU_src_A <= 0;
			EX_EX_ALU_src_B <= 0;
			EX_MEM_readM <= 0;
			EX_MEM_writeM <= 0;
			EX_WB_memtoreg <= 0;
			EX_WB_reg_write <= 0;
			EX_use_rs <= 0;
			EX_use_rt <= 0;
			EX_is_J_type <= 0;
			EX_is_JR_type <= 0;
			EX_is_real <= 0;
			temp <= 2'b10;
			EX_is_halt <= 0;
		end
		else begin
			EX_PC_plus_one <= ID_PC_plus_one;
			EX_read_rs <= ID_read_rs;
			EX_read_rt <= ID_read_rt;
			EX_opcode <= ID_opcode;
			EX_funct <= ID_funct;
			EX_imm16 <= ID_imm16;
			EX_rs <= ID_rs;
			EX_rt <= ID_rt;
			EX_write_reg <= ID_write_reg;
			EX_EX_ALU_src_A <= ID_EX_ALU_src_A;
			EX_EX_ALU_src_B <= ID_EX_ALU_src_B;
			EX_WB_memtoreg <= ID_WB_memtoreg;
			EX_use_rs <= ID_use_rs;
			EX_use_rt <= ID_use_rt;
			EX_is_J_type <= ID_is_J_type;
			EX_is_JR_type <= ID_is_JR_type;
			EX_is_real <= is_real;
			EX_is_halt <= is_halt;
			if (setZero) begin
				EX_MEM_readM <= 0;
				EX_MEM_writeM <= 0;
				EX_WB_reg_write <= 0;
			end
			else begin
				EX_MEM_readM <= ID_MEM_readM;
				EX_MEM_writeM <= ID_MEM_writeM;
				EX_WB_reg_write <= ID_WB_reg_write;	
			end
		end
	end	
endmodule // IDandEX

module EXandMEM(
	clk, reset_n, EX_is_real, EX_PC_plus_one, EX_ALU_result, EX_write_data, EX_write_reg, EX_MEM_readM, EX_MEM_writeM, EX_WB_memtoreg, EX_WB_reg_write,
	MEM_is_real, MEM_PC_plus_one, MEM_ALU_result, MEM_write_data, MEM_write_reg, MEM_MEM_readM, MEM_MEM_writeM, MEM_WB_memtoreg, MEM_WB_reg_write
);
	input clk;
	input reset_n;
	input EX_is_real;
	input [`WORD_SIZE-1:0] EX_PC_plus_one;
	input [`WORD_SIZE-1:0] EX_ALU_result;
	input [`WORD_SIZE-1:0] EX_write_data;
	input [1:0] EX_write_reg;
	input EX_MEM_readM;
	input EX_MEM_writeM;
	input [1:0] EX_WB_memtoreg;
	input EX_WB_reg_write;

	output MEM_is_real;
	output [`WORD_SIZE-1:0] MEM_PC_plus_one;
	output [`WORD_SIZE-1:0] MEM_ALU_result;
	output [`WORD_SIZE-1:0] MEM_write_data;
	output [1:0] MEM_write_reg;
	output MEM_MEM_readM;
	output MEM_MEM_writeM;
	output [1:0] MEM_WB_memtoreg;
	output MEM_WB_reg_write;

	reg MEM_is_real;
	reg [`WORD_SIZE-1:0] MEM_PC_plus_one;
	reg [`WORD_SIZE-1:0] MEM_ALU_result;
	reg [`WORD_SIZE-1:0] MEM_write_data;
	reg [1:0] MEM_write_reg;
	reg MEM_MEM_readM;
	reg MEM_MEM_writeM;
	reg [1:0] MEM_WB_memtoreg;
	reg MEM_WB_reg_write;

	always @(posedge(clk)) begin
		if (!reset_n) begin
			MEM_PC_plus_one <= 0;
			MEM_ALU_result <= 0;
			MEM_write_data <= 0;
			MEM_write_reg <= 0;
			MEM_MEM_readM <= 0;
			MEM_MEM_writeM <= 0;
			MEM_WB_memtoreg <= 0;
			MEM_WB_reg_write <= 0;
			MEM_is_real <= 0;
		end
		else begin
			MEM_PC_plus_one <= EX_PC_plus_one;
			MEM_ALU_result <= EX_ALU_result;
			MEM_write_data <= EX_write_data;
			MEM_write_reg <= EX_write_reg;
			MEM_MEM_readM <= EX_MEM_readM;
			MEM_MEM_writeM <= EX_MEM_writeM;
			MEM_WB_memtoreg <= EX_WB_memtoreg;
			MEM_WB_reg_write <= EX_WB_reg_write;
			MEM_is_real <= EX_is_real;
		end
	end	
endmodule // EXandMEM


module MEMandWB(
	clk, reset_n, MEM_is_real, MEM_PC_plus_one, MEM_ALU_result, MEM_read_data, MEM_write_reg, MEM_WB_memtoreg, MEM_WB_reg_write,
	WB_is_real, WB_PC_plus_one, WB_ALU_result, WB_read_data, WB_write_reg, WB_WB_memtoreg, WB_WB_reg_write
);
	input clk;
	input reset_n;
	input MEM_is_real;
	input [`WORD_SIZE-1:0] MEM_PC_plus_one;
	input [`WORD_SIZE-1:0] MEM_ALU_result;
	input [`WORD_SIZE-1:0] MEM_read_data;
	input [1:0] MEM_write_reg;
	input [1:0] MEM_WB_memtoreg;
	input MEM_WB_reg_write;

	output WB_is_real;
	output [`WORD_SIZE-1:0] WB_PC_plus_one;
	output [`WORD_SIZE-1:0] WB_ALU_result;
	output [`WORD_SIZE-1:0] WB_read_data;
	output [1:0] WB_write_reg;
	output [1:0] WB_WB_memtoreg;
	output WB_WB_reg_write;

	reg WB_is_real;
	reg [`WORD_SIZE-1:0] WB_PC_plus_one;
	reg [`WORD_SIZE-1:0] WB_ALU_result;
	reg [`WORD_SIZE-1:0] WB_read_data;
	reg [1:0] WB_write_reg;
	reg [1:0] WB_WB_memtoreg;
	reg WB_WB_reg_write;
	reg WB_valid;

	always @(posedge(clk)) begin
		if (!reset_n) begin
			WB_PC_plus_one <= 0;
			WB_ALU_result <= 0;
			WB_read_data <= 0;
			WB_write_reg <= 0;
			WB_WB_memtoreg <= 0;
			WB_WB_reg_write <= 0;
			WB_is_real <= 0;
			WB_valid <= 0;
		end
		else begin
			if(WB_valid) begin
				WB_PC_plus_one <= MEM_PC_plus_one;
				WB_ALU_result <= MEM_ALU_result;
				WB_read_data <= MEM_read_data;
				WB_write_reg <= MEM_write_reg;
				WB_WB_memtoreg <= MEM_WB_memtoreg;
				WB_WB_reg_write <= MEM_WB_reg_write;
				WB_is_real <= MEM_is_real;
			end
			else begin
				WB_valid <= 1;
			end
		end
	end	
endmodule // MEMandWB