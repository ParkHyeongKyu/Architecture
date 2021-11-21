`timescale 1ns/1ns
`include "const.v"

module cpu(clk, reset_n, readM, writeM, address, data, num_inst, output_port, is_halted);
	input clk;
	input reset_n;

	output readM;
	output writeM;
	output [`WORD_SIZE-1:0] address;

	inout [`WORD_SIZE-1:0] data;

	output [`WORD_SIZE-1:0] num_inst;		// number of instruction during execution (for debuging & testing purpose)
	output [`WORD_SIZE-1:0] output_port;	// this will be used for a "WWD" instruction
	output is_halted;

	// TODO : Implement your multi-cycle CPU!
	reg [`WORD_SIZE-1:0] num_inst;
	reg [`WORD_SIZE-1:0] output_port;
	reg is_halted;

	reg [`ST_SIZE-1:0] current_state;
	wire PVSWriteEn;


	// ALUcontrol
	wire [2:0] ALU_func;
	// control
	wire pc_write;
	wire IorD;
	wire readM;
	wire writeM;
	wire IR_write;
	wire [1:0] Reg_src;
	wire Reg_write;
	wire [1:0] ALU_src_A;
	wire [1:0] ALU_src_B;
	wire PC_src;
	wire is_R_type;
	wire is_JL_type;
	wire is_JR_type;
	wire is_halt;
	wire [`ST_SIZE-1:0] next_state;
	// PC
	wire [`WORD_SIZE-1:0] current_pc;
	// ALU
	wire [`WORD_SIZE-1:0] alu_out;
	// IR
	wire [3:0] opcode;
	wire [1:0] rs;
	wire [1:0] rt;
	wire [1:0] rd;
	wire [5:0] inst_func;
	wire [7:0] imm_offset;
	wire [11:0] target;
	// IMMGEN
	wire [`WORD_SIZE-1:0] imm_16;
	// gpr
	wire [`WORD_SIZE-1:0] read_data_1;
	wire [`WORD_SIZE-1:0] read_data_2;
	// buffer_A
	wire [`WORD_SIZE-1:0] buffered_read_data_1;
	// buffer_B
	wire [`WORD_SIZE-1:0] buffered_read_data_2;
	// ALUout
	wire [`WORD_SIZE-1:0] buffered_alu_out;
	// Memory data register MDR
	wire [`WORD_SIZE-1:0] buffered_MDR;
	// MUXs
	wire [`WORD_SIZE-1:0] address;
	wire [`WORD_SIZE-1:0] next_pc_alu;
	wire [`WORD_SIZE-1:0] next_pc;
	wire [1:0] wrtie_reg;
	wire [`WORD_SIZE-1:0] write_data;
	wire [`WORD_SIZE-1:0] alu_A;
	wire [`WORD_SIZE-1:0] alu_B;

	//bcond
	wire [3:0] bcond;
	assign bcond[0] = ((read_data_1 - read_data_2) != 0) ? 1 : 0;
	assign bcond[1] = ((read_data_1 - read_data_2) == 0) ? 1 : 0;
	assign bcond[2] = ((read_data_1 < 16'h8000) && (read_data_1 > 16'h0000)) ? 1 : 0;
	assign bcond[3] = (read_data_1 > 16'h7fff) ? 1 : 0;

	program_counter PC(clk, reset_n, PVSWriteEn, pc_write, next_pc, current_pc);
	ALU alu(alu_A, alu_B, ALU_func, alu_out);
	instruction_register IR(clk, reset_n, IR_write, data, opcode, rs, rt, rd, inst_func, imm_offset, target);
	imm_gen IMMGEN(opcode, imm_offset, current_pc, target, imm_16);
	GPR gpr(clk, reset_n, PVSWriteEn, Reg_write, rs, rt, wrtie_reg, write_data, is_JL_type, read_data_1, read_data_2);

	ALU_control ALUCONTROL(inst_func, opcode, current_state, ALU_func);
	main_control MAINCONTROL(clk, reset_n, opcode, inst_func, bcond, current_state, pc_write, IorD, readM, writeM, IR_write, Reg_src, Reg_write, ALU_src_A, ALU_src_B, PC_src, is_R_type, is_JL_type, is_JR_type, is_halt, next_state);

	buffer buffer_A(clk, reset_n, read_data_1, buffered_read_data_1);
	buffer buffer_B(clk, reset_n, read_data_2, buffered_read_data_2);
	alu_out_buffer ALUout(clk, reset_n, alu_out, current_state, buffered_alu_out);
	buffer MDR(clk, reset_n, data, buffered_MDR);

	MUX16_2 mux_MEMaddress(current_pc, alu_out, IorD, address);
	MUX16_2 mux_PC1(buffered_alu_out, alu_out, PC_src, next_pc_alu);
	MUX16_2 mux_PC2(next_pc_alu, buffered_read_data_1, is_JR_type, next_pc);
	MUX2_2 mux_rtorrd(rt, rd, is_R_type, wrtie_reg);
	MUX16_4 mux_write_data(alu_out, imm_16, buffered_MDR, buffered_alu_out, Reg_src, write_data);
	MUX16_4 mux_alu_A(buffered_read_data_1, current_pc, 16'b0, 16'b0, ALU_src_A, alu_A);
	MUX16_4 mux_alu_B(buffered_read_data_2, 16'b1, imm_16, 16'b1, ALU_src_B, alu_B);
	
	assign data = (writeM) ? buffered_read_data_2 : 16'bz;

	assign PVSWriteEn = (next_state == `ST_IF1) ? 1 : 0;

	// initial
	initial begin
		num_inst <= 0;
		output_port <= 0;
		is_halted <= 0;
		current_state <= `ST_IF1;
	end
	// general
	always @(posedge(clk)) begin
		if (!reset_n) begin
			num_inst <= 0;
			output_port <= 0;
			is_halted <= 0;
			current_state <= `ST_IF1;	
		end
		else begin
			current_state <= next_state;
			if(next_state == `ST_IF1) begin
				num_inst <= num_inst + 1;
			end
		end
	end

	// when halt
	always @(*) begin
		if (current_state == `ST_HLT) begin
			is_halted <= 1'b1;
		end
	end
	// WWD
	always @(posedge(clk)) begin
		if(current_state == `ST_EX2 && opcode == `WWD_OP && inst_func == `INST_FUNC_WWD) begin
			output_port <= read_data_1;
		end
	end
endmodule
