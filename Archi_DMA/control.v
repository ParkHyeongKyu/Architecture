`timescale 1ns/1ns
`include "const.v"

module ALU_control(
	inst_func, opcode,
	ALU_func
);
	input [5:0] inst_func;
	input [3:0] opcode;

	output [2:0] ALU_func;

	reg [2:0] ALU_func;

	always @(*) begin
		case (opcode)
			`ALU_OP: begin
				case(inst_func)
					`INST_FUNC_ADD: ALU_func <= `ALU_FUNC_ADD;
					`INST_FUNC_SUB: ALU_func <= `ALU_FUNC_SUB;
					`INST_FUNC_AND: ALU_func <= `ALU_FUNC_AND;
					`INST_FUNC_ORR: ALU_func <= `ALU_FUNC_ORR;
					`INST_FUNC_NOT: ALU_func <= `ALU_FUNC_NOT;
					`INST_FUNC_TCP: ALU_func <= `ALU_FUNC_TCP;
					`INST_FUNC_SHL: ALU_func <= `ALU_FUNC_SHL;
					`INST_FUNC_SHR: ALU_func <= `ALU_FUNC_SHR;
					`INST_FUNC_JPR: ALU_func <= `ALU_FUNC_ADD; //default
					`INST_FUNC_JRL: ALU_func <= `ALU_FUNC_ADD;
				endcase
			end
			`ADI_OP: ALU_func <= `ALU_FUNC_ADD;
			`ORI_OP: ALU_func <= `ALU_FUNC_ORR;
			`LHI_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`LWD_OP: ALU_func <= `ALU_FUNC_ADD;
			`SWD_OP: ALU_func <= `ALU_FUNC_ADD;
			`BNE_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`BEQ_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`BGZ_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`BLZ_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`JMP_OP: ALU_func <= `ALU_FUNC_ADD; //default
			`JAL_OP: ALU_func <= `ALU_FUNC_ADD;
		endcase
	end
endmodule // ALU_control


module main_control(
	opcode, funct,
	readM, writeM, Reg_write, is_JMPJALLHI, ALU_src_B, is_halt,  storeRT, is_JL_type, is_J_type, is_JR_type, use_rs, use_rt, m2r
);
	input [3:0] opcode;
	input [5:0] funct;

	output readM;
	output writeM;
	output Reg_write;
	output is_JMPJALLHI;
	output ALU_src_B;
	output is_halt;
	output storeRT;
	output is_JL_type;
	output is_J_type;
	output is_JR_type;
	output use_rs;
	output use_rt;
	output [1:0] m2r;

	// combinational logic
	assign readM = (opcode == `LWD_OP);
	assign writeM = (opcode == `SWD_OP); 
	assign Reg_write = (opcode != `SWD_OP) && ((opcode != `BNE_OP) && (opcode != `BEQ_OP) && (opcode != `BGZ_OP) && (opcode != `BLZ_OP)) && (opcode != `JMP_OP) && ((opcode != `JPR_OP) || (funct != `INST_FUNC_JPR)) && (funct != `INST_FUNC_WWD);
	assign is_JMPJALLHI = ((opcode == `JMP_OP) || (opcode == `JAL_OP)) || (opcode == `LHI_OP);
	assign ALU_src_B = (opcode != `ALU_OP) && ((opcode != `BNE_OP) && (opcode != `BEQ_OP) && (opcode != `BGZ_OP) && (opcode != `BLZ_OP));
	assign is_J_type = (opcode == `JAL_OP) || (opcode == `JMP_OP) || (opcode == `JPR_OP && funct == `INST_FUNC_JPR) || (opcode == `JRL_OP && funct == `INST_FUNC_JRL);
	assign is_JL_type = (opcode == `JAL_OP) || ((opcode == `JRL_OP) && (funct ==`INST_FUNC_JRL));
	assign is_halt = (opcode == `HLT_OP) && (funct == `INST_FUNC_HLT);
	assign storeRT = (opcode == `ADI_OP) || (opcode == `ORI_OP) || (opcode == `LHI_OP) || (opcode == `LWD_OP);
	assign is_JR_type = ((opcode == `JRL_OP) && (funct == `INST_FUNC_JRL)) || ((opcode == `JPR_OP) && (funct == `INST_FUNC_JPR));
	assign use_rs =	!is_JMPJALLHI;
	assign use_rt = (opcode == `SWD_OP) || (opcode == `BNE_OP) || (opcode == `BEQ_OP) || ((opcode == `ALU_OP) && ((funct == `INST_FUNC_ADD) || (funct == `INST_FUNC_SUB) || (funct == `INST_FUNC_AND) || (funct == `INST_FUNC_ORR)));
	assign m2r[0] = readM;
	assign m2r[1] = is_JL_type;
endmodule // control