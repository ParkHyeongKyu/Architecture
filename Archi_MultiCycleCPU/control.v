`timescale 1ns/1ns
`include "const.v"

module ALU_control(
	inst_func, opcode, current_state,
	ALU_func
);
	input [5:0] inst_func;
	input [3:0] opcode;
	input [`ST_SIZE-1:0] current_state;

	output [2:0] ALU_func;

	reg [2:0] ALU_func;

	always @(*) begin
		if (current_state == `ST_EX1) begin
			ALU_func = `ALU_FUNC_ADD; // pc = pc + 1;
		end
		else begin
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
	end
endmodule // ALU_control


module main_control(
	clk, reset_n, opcode, funct, bcond, current_state,
	pc_write, IorD, readM, writeM, IR_write, Reg_src, Reg_write, ALU_src_A, ALU_src_B, PC_src, is_R_type, is_JL_type, is_JR_type, is_halt, next_state
);
	input clk;
	input reset_n;
	input [3:0] opcode;
	input [5:0] funct;
	input [3:0] bcond;
	input [`ST_SIZE-1:0] current_state;

	output pc_write;
	output IorD;
	output readM;
	output writeM;
	output IR_write;
	output [1:0] Reg_src;
	output Reg_write;
	output [1:0] ALU_src_A;
	output [1:0] ALU_src_B;
	output PC_src;
	output is_R_type;
	output is_JL_type;
	output is_JR_type;
	output is_halt;
	output [`ST_SIZE-1:0] next_state;

	reg [`ST_SIZE-1:0] next_state;

	// combinational logic
	assign pc_write = (next_state == `ST_IF1) ? 1 : 0;
	assign IorD = ((current_state == `ST_MEM1) || (current_state == `ST_MEM2) || (current_state == `ST_MEM3) || (current_state == `ST_MEM4)) && ((opcode == `LWD_OP) || (opcode == `SWD_OP));
	assign readM = ((current_state == `ST_IF1) || (current_state == `ST_IF2) || (current_state == `ST_IF3) || (current_state == `ST_IF4)) || ((current_state == `ST_MEM1) || (current_state == `ST_MEM2) || (current_state == `ST_MEM3) || (current_state == `ST_MEM4)) && (opcode == `LWD_OP);
	assign writeM = ((current_state == `ST_MEM1) || (current_state == `ST_MEM2) || (current_state == `ST_MEM3) || (current_state == `ST_MEM4)) && (opcode == `SWD_OP);
	assign IR_write = (current_state == `ST_IF4);
	assign Reg_src = (opcode == `LHI_OP) ? 2'b01 :
					(opcode == `LWD_OP) ? 2'b10 :
					((opcode == `JAL_OP) || (opcode == `JRL_OP && funct == `INST_FUNC_JRL)) ? 2'b11 : 2'b00;
	assign Reg_write = (current_state == `ST_WB);
	assign ALU_src_A[0] = (current_state == `ST_EX1) || ((opcode == `BNE_OP) || (opcode == `BEQ_OP) || (opcode == `BGZ_OP) || (opcode == `BLZ_OP));
	assign ALU_src_A[1] = (current_state != `ST_EX1) && ((opcode == `LHI_OP) || (opcode == `JMP_OP) || (opcode == `JAL_OP));
	assign ALU_src_B[0] = (current_state == `ST_EX1);
	assign ALU_src_B[1] = (opcode != `ALU_OP);
	assign PC_src = (bcond[0] && (opcode == `BNE_OP)) || (bcond[1] && (opcode == `BEQ_OP)) || (bcond[2] && (opcode == `BGZ_OP)) || (bcond[3] && (opcode == `BLZ_OP)) || (opcode == `JMP_OP) || (opcode == `JAL_OP);
	assign is_R_type = (opcode == `ALU_OP);
	assign is_JL_type = (opcode == `JAL_OP) || ((opcode == `JRL_OP) && (funct ==`INST_FUNC_JRL));
	assign is_JR_type = ((opcode == `JRL_OP) && (funct == `INST_FUNC_JRL)) || ((opcode == `JPR_OP) && (funct == `INST_FUNC_JPR));
	assign is_halt = (opcode == `HLT_OP) && (funct == `INST_FUNC_HLT);

	always @(*) begin
		case (current_state)
			`ST_IF1: next_state <= `ST_IF2;
			`ST_IF2: next_state <= `ST_IF3;
			`ST_IF3: next_state <= `ST_IF4;
			`ST_IF4: next_state <= `ST_ID;
			`ST_ID: begin
				if(opcode == `HLT_OP && funct == `INST_FUNC_HLT)
					next_state <= `ST_HLT;
				else
					next_state <= `ST_EX1;
			end
			`ST_EX1: next_state <= `ST_EX2;
			`ST_EX2: begin
				if(opcode == `BNE_OP || opcode == `BEQ_OP || opcode == `BGZ_OP || opcode == `BLZ_OP || opcode == `JMP_OP|| (opcode == `JPR_OP && funct == `INST_FUNC_JPR) || (opcode == `WWD_OP && funct == `INST_FUNC_WWD))
					next_state <= `ST_IF1;
				else if(opcode == `LWD_OP || opcode == `SWD_OP)
					next_state <= `ST_MEM1;
				else
					next_state <= `ST_WB;
			end
			`ST_MEM1: next_state <= `ST_MEM2;
			`ST_MEM2: next_state <= `ST_MEM3;
			`ST_MEM3: next_state <= `ST_MEM4;
			`ST_MEM4: begin
				if(opcode == `LWD_OP)
					next_state <= `ST_WB;
				else
					next_state <= `ST_IF1;
			end
			`ST_WB: begin
				next_state <= `ST_IF1;
			end
		endcase
	end
endmodule // control