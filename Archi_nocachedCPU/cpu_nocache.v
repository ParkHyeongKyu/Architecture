`timescale 1ns/1ns
`include "opcodes.v"
`define INST_FUNC_WWD 6'd28
`define INST_FUNC_HLT 6'd29
`define INST_FUNC_ENI 6'd30
`define INST_FUNC_DSI 6'd31

//ALU
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

//Instruction decoder(Instruction memory)
module InstructionDecoder(
	data, 
	opcode, rs, rt, rd, funct, immediate, target
);
	input [`WORD_SIZE-1:0] data;
	output [3:0] opcode;
	output [1:0] rs;
	output [1:0] rt;
	output [1:0] rd;
	output [5:0] funct;
	output [7:0] immediate;
	output [11:0] target;

	assign opcode = data[15:12];
	assign rs = data[11:10];
	assign rt = data[9:8];
	assign rd = data[7:6];
	assign funct = data[5:0];
	assign immediate = data[7:0];
	assign target = data[11:0];

endmodule

//Registers
module GPR(
	clk, readReg1, readReg2, WriteReg, writeData, RegWrite, 
	readData1, readData2
);
	input clk;	
	input [1:0] readReg1;
	input [1:0] readReg2;
	input [1:0] WriteReg;
	input [`WORD_SIZE-1:0] writeData;
	input RegWrite;      //if 1, then write writeData into WriteReg
	// input isJLtype;

	output [`WORD_SIZE-1:0] readData1;
	output [`WORD_SIZE-1:0] readData2;

	//have to implement register by ourserlf
	reg [`WORD_SIZE-1:0] registers [`NUM_REGS-1:0];
	integer i;

	initial begin
		for(i = 0; i < `NUM_REGS; i = i+1) begin
			registers[i] <= 16'h0;
		end
	end

	always @(negedge(clk)) begin
		if (RegWrite) begin
			registers[WriteReg] <= writeData;
		end
	end

	assign readData1 = registers[readReg1];
	assign readData2 = registers[readReg2];

endmodule

//ALU control
module ALU_Control(
	InstFunc, opcode, 
	ALUFunc
);
	input [5:0] InstFunc;
	input [3:0] opcode;
	output [2:0] ALUFunc;

	reg [2:0] ALUFunc;
	always @(InstFunc, opcode) begin
		case (opcode)
			`ALU_OP: begin
				case(InstFunc)
					`INST_FUNC_ADD: ALUFunc = `ALU_FUNC_ADD;
					`INST_FUNC_SUB: ALUFunc = `ALU_FUNC_SUB;
					`INST_FUNC_AND: ALUFunc = `ALU_FUNC_AND;
					`INST_FUNC_ORR: ALUFunc = `ALU_FUNC_ORR;
					`INST_FUNC_NOT: ALUFunc = `ALU_FUNC_NOT;
					`INST_FUNC_TCP: ALUFunc = `ALU_FUNC_TCP;
					`INST_FUNC_SHL: ALUFunc = `ALU_FUNC_SHL;
					`INST_FUNC_SHR: ALUFunc = `ALU_FUNC_SHR;
					`INST_FUNC_JPR: ALUFunc = `ALU_FUNC_ADD; //default
					`INST_FUNC_JRL: ALUFunc = `ALU_FUNC_ADD; //default
				endcase
			end
			`ADI_OP: ALUFunc = `ALU_FUNC_ADD;
			`ORI_OP: ALUFunc = `ALU_FUNC_ORR;
			`LHI_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`LWD_OP: ALUFunc = `ALU_FUNC_ADD;
			`SWD_OP: ALUFunc = `ALU_FUNC_ADD;
			`BNE_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`BEQ_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`BGZ_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`BLZ_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`JMP_OP: ALUFunc = `ALU_FUNC_ADD; //default
			`JAL_OP: ALUFunc = `ALU_FUNC_ADD; //default
		endcase
	end

endmodule

//16bit mux 4 choices
module MUX16_pc(
	in0, in1, in2, in3, current, flag, IFstate, 
	result
);
	input [`WORD_SIZE-1:0] in0;
	input [`WORD_SIZE-1:0] in1;
	input [`WORD_SIZE-1:0] in2;
	input [`WORD_SIZE-1:0] in3;
	input [`WORD_SIZE-1:0] current;
	input [1:0] flag;
	input IFstate;
	output [`WORD_SIZE-1:0] result;
	reg [`WORD_SIZE-1:0] result;

	initial begin
		result = 16'b0;
	end

	always @(*) begin
		if(IFstate) begin
			if(flag == 2'b00) begin
				result = in0;
			end
			else if(flag == 2'b01) begin
				result = in1;
			end
			else if(flag == 2'b10) begin
				result = in2;
			end
			else begin
				result = in3;
			end
		end
		else begin
			result = current;
		end
	end

endmodule

//16bit mux 4 choices
module MUX16_4(
	in0, in1, in2, in3, flag,
	result
);
	input [`WORD_SIZE-1:0] in0;
	input [`WORD_SIZE-1:0] in1;
	input [`WORD_SIZE-1:0] in2;
	input [`WORD_SIZE-1:0] in3;
	input [1:0] flag;
	output [`WORD_SIZE-1:0] result;
	reg [`WORD_SIZE-1:0] result;

	initial begin
		result = 16'b0;
	end

	always @(*) begin
		if(flag == 2'b00) begin
			result = in0;
		end
		else if(flag == 2'b01) begin
			result = in1;
		end
		else if(flag == 2'b10) begin
			result = in2;
		end
		else begin
			result = in3;
		end
	end

endmodule

//16bit mux
module Mux16_2(
	in0, in1, flag, 
	result
);
	input [`WORD_SIZE-1:0] in0;
	input [`WORD_SIZE-1:0] in1;
	input flag;
	output [`WORD_SIZE-1:0] result;
	reg [`WORD_SIZE-1:0] result;

	always @(*) begin
		if(flag == 1'b0) begin
			result = in0;
		end
		else begin
			result = in1;
		end
	end

endmodule

//2bit mux
module Mux2_2(
	in0, in1, flag, 
	result
);
	input [1:0] in0;
	input [1:0] in1;
	input flag;
	output [1:0] result;
	reg [1:0] result;

	always @(*) begin
		if(flag == 1'b0) begin
			result = in0;
		end
		else begin
			result = in1;
		end
	end

endmodule

//immgenerater
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

module InstructionMemory(
	clk, Reset_N, pc, data1, 
	readM1, address1, instruction
);
	input clk;
	input Reset_N;
	input [`WORD_SIZE-1:0] pc;
	input [`WORD_SIZE-1:0] data1;

	output readM1;
	output [`WORD_SIZE-1:0] address1;
	output [`WORD_SIZE-1:0] instruction;

	reg readM1;
	reg [`WORD_SIZE-1:0] address1;
	reg [`WORD_SIZE-1:0] instruction;

	always @(posedge(clk)) begin
		if (!Reset_N) begin
			readM1 <= 0;
			instruction <= 0;
		end
	end

	always @(*) begin
		readM1 = 1;
		address1 = pc;
	end

	always @(*) begin
		instruction = data1;
		readM1 = 0;
	end
endmodule

module DataMemory(
	clk, Reset_N, Address, WriteData, data2, MemWrite, MemRead, 
	writeM2, readM2, address2, ReadData
);
	input clk;
	input Reset_N;
	input [`WORD_SIZE-1:0] Address;
	input [`WORD_SIZE-1:0] WriteData;
	input [`WORD_SIZE-1:0] data2;
	input MemWrite;
	input MemRead;

	output writeM2;
	output readM2;
	output [`WORD_SIZE-1:0] address2;
	output [`WORD_SIZE-1:0] ReadData;

	reg writeM2;
	reg readM2;
	reg [`WORD_SIZE-1:0] address2;
	assign ReadData = data2;

	always @(posedge(clk)) begin
		if (!Reset_N) begin
			writeM2 <= 0;
			readM2 <= 0;
		end
	end

	always @(*) begin
		if (MemWrite) begin
			writeM2 = 1;
			address2 = Address;
		end
		else begin
			writeM2 = 0;
		end

		if (MemRead) begin
			readM2 = 1;
			address2 = Address;
		end
		else begin
			readM2 = 0;
		end
	end
endmodule

module HazardDetectionUnit(
	IDisBranch, EXMemRead, IDrs, IDrt, IDopcode, IDpc, IDbcond, EXrt, EXrd, EXisJtype, EXRegWrite, EXisRtype, 
	PCWrite, IFIDWrite, IDisStall, IDEXWrite
);
	input IDisBranch;
	input EXMemRead;
	input [1:0] IDrs;
	input [1:0] IDrt;
	input [3:0] IDopcode;
	input [`WORD_SIZE-1:0] IDpc;
	input [3:0] IDbcond;
	input [1:0] EXrt;
	input [1:0] EXrd;
	input EXisJtype;
	input EXRegWrite;
	input EXisRtype;

	output PCWrite;
	output IFIDWrite;
	output IDisStall;
	output IDEXWrite;
	reg IFIDWrite;
	reg IDEXWrite;

	wire users;
	wire usert;
	
	assign users = (IDopcode == `ALU_OP) || (IDopcode == `ADI_OP) || (IDopcode == `ORI_OP) || (IDopcode == `LHI_OP) ||
					(IDopcode == `SWD_OP) || (IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP) || (IDopcode == `BGZ_OP) || (IDopcode == `BLZ_OP);
	assign usert = (IDopcode == `ALU_OP) || (IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP);
	assign IDisStall = (EXisRtype && EXRegWrite && ((IDrs == EXrd) && users) && EXMemRead) ||
					(EXisRtype && EXRegWrite && ((IDrs == EXrd) && usert) && EXMemRead) ||
					(!EXisRtype && EXRegWrite && ((IDrs == EXrt) && users) && EXMemRead) ||
					(!EXisRtype && EXRegWrite && ((IDrs == EXrt) && usert) && EXMemRead);
	assign PCWrite = !IDisStall;
	initial begin
		IFIDWrite = 1;
		IDEXWrite = 1;
	end
	always @(*) begin
		IFIDWrite = ((!EXisJtype) && (!IDisStall) && (!IDisBranch || (!IDbcond[IDopcode])));
		IDEXWrite = (!EXisJtype);
	end
endmodule

module ForwardingUnit(
	MEMRegWrite, MEMWriteReg, WBRegWrite, WBWriteReg, EXrs, EXrt, EXopcode, EXWriteReg, EXRegWrite, IDrs, IDrt, IDopcode, 
	ForwardA, ForwardB, IDForwardA, IDForwardB
);
	input MEMRegWrite;
	input [1:0] MEMWriteReg;
	input WBRegWrite;
	input [1:0] WBWriteReg;
	input [1:0] EXrs;
	input [1:0] EXrt;
	input [3:0] EXopcode;
	input [1:0] EXWriteReg;
	input EXRegWrite;
	input [1:0] IDrs;
	input [1:0] IDrt;
	input [3:0] IDopcode;

	output [1:0] ForwardA;
	output [1:0] ForwardB;
	output [1:0] IDForwardA;
	output [1:0] IDForwardB;

	assign ForwardA = ((EXopcode != `JMP_OP) && (EXopcode != `JAL_OP) && (EXopcode != `LHI_OP) && (EXrs == MEMWriteReg) && MEMRegWrite) ? 2'b10 : 
					((EXopcode != `JMP_OP) && (EXopcode != `JAL_OP) && (EXopcode != `LHI_OP) && (EXrs == WBWriteReg) && WBRegWrite) ? 2'b01 : 2'b00;
	assign ForwardB = ((EXopcode == 4'd15) && (EXrt == MEMWriteReg) && MEMRegWrite) ? 2'b10 :
					((EXopcode == 4'd15) && (EXrt == WBWriteReg) && WBRegWrite) ? 2'b01 : 2'b00;

	assign IDForwardA = (((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP) || (IDopcode == `BGZ_OP) || (IDopcode == `BLZ_OP)) && (IDrs == EXWriteReg) && EXRegWrite) ? 2'b11 :
						(((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP) || (IDopcode == `BGZ_OP) || (IDopcode == `BLZ_OP)) && (IDrs == MEMWriteReg) && MEMRegWrite) ? 2'b10 :
						(((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP) || (IDopcode == `BGZ_OP) || (IDopcode == `BLZ_OP)) && (IDrs == WBWriteReg) && WBRegWrite) ? 2'b01 : 2'b00;
	assign IDForwardB = (((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP)) && (IDrt == EXWriteReg) && EXRegWrite) ? 2'b11 :
						(((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP)) && (IDrt == MEMWriteReg) && MEMRegWrite) ? 2'b10 :
						(((IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP)) && (IDrt == WBWriteReg) && WBRegWrite) ? 2'b01 : 2'b00;
endmodule

module Comparator(
	IDReadData1, IDReadData2, 
	IDbcond
);
	input [`WORD_SIZE-1:0] IDReadData1;
	input [`WORD_SIZE-1:0] IDReadData2;

	output [3:0] IDbcond;

	assign IDbcond[0] = (IDReadData1 != IDReadData2);
	assign IDbcond[1] = (IDReadData1 == IDReadData2);
	assign IDbcond[2] = (IDReadData1 < 16'h7000) && (IDReadData1 != 16'h0000);
	assign IDbcond[3] = IDReadData1[15];
endmodule

module PCSrcGenerator(
	clk, Reset_N, IDisBranch, IDbcond, IDopcode, EXisJRtype, EXisJtype, 
	PCSrc
);
	input clk;
	input Reset_N;
	input IDisBranch;
	input [3:0] IDbcond;
	input [3:0] IDopcode;
	input EXisJRtype;
	input EXisJtype;

	output [1:0] PCSrc;
	reg [1:0] PCSrc;

	always @(posedge(clk)) begin
		if (!Reset_N) begin
			PCSrc <= 2'b00;
		end
	end

	always @(*) begin
		PCSrc = (EXisJtype && EXisJRtype) ? 2'b11 : (EXisJtype) ? 2'b10 : (IDisBranch && IDbcond[IDopcode]) ? 2'b01 : 2'b00;
	end
endmodule

module Control(
	IDopcode, IDfunct, IDisStall, 
	IDMemRead, IDMemWrite, IDMemtoReg, IDALUSrc, IDRegWrite, IDisJtype, IDisRtype, IDisLHI, IDisJLtype, IDisJRtype, IDisJMPJALLHI, IDisHalt, IDisBranch, IDisWWD
);
	input [3:0] IDopcode;
	input [5:0] IDfunct;
	input IDisStall;
	
	output IDMemRead;
	output IDMemWrite;
	output IDMemtoReg;
	output IDALUSrc;
	output IDRegWrite;
	output IDisJtype;
	output IDisRtype;
	output IDisLHI;
	output IDisJLtype;
	output IDisJRtype;
	output IDisJMPJALLHI;
	output IDisHalt;
	output IDisBranch;
	output IDisWWD;
	
	assign IDMemRead = (IDopcode == `LWD_OP);
	assign IDMemWrite = (IDisStall == 0) && (IDopcode == `SWD_OP);
	assign IDMemtoReg = (IDopcode == `LWD_OP);
	assign IDALUSrc = (IDopcode != `ALU_OP) && ((IDopcode != `BNE_OP) && (IDopcode != `BEQ_OP) && (IDopcode != `BGZ_OP) && (IDopcode != `BLZ_OP));
	assign IDRegWrite = (IDisStall == 0) && (IDopcode != `SWD_OP) && ((IDopcode != `BNE_OP) && (IDopcode != `BEQ_OP) && (IDopcode != `BGZ_OP) && (IDopcode != `BLZ_OP)) && (IDopcode != `JMP_OP) && ((IDopcode != `JPR_OP) || (IDfunct != `INST_FUNC_JPR)) && (IDfunct != `INST_FUNC_WWD);
	assign IDisJtype = (IDopcode == `JAL_OP) || (IDopcode == `JMP_OP) || (IDopcode == `JPR_OP && IDfunct == `INST_FUNC_JPR) || (IDopcode == `JRL_OP && IDfunct == `INST_FUNC_JRL);
	assign IDisRtype = (IDopcode == `ALU_OP);
	assign IDisLHI = (IDopcode == `LHI_OP);
	assign IDisJLtype = (IDopcode == `JAL_OP) || ((IDopcode == `JRL_OP) && (IDfunct ==`INST_FUNC_JRL));
	assign IDisJRtype = ((IDopcode == `JRL_OP) && (IDfunct == `INST_FUNC_JRL)) || ((IDopcode == `JPR_OP) && (IDfunct == `INST_FUNC_JPR));
	assign IDisJMPJALLHI = ((IDopcode == `JMP_OP) || (IDopcode == `JAL_OP)) || (IDopcode == `LHI_OP);
	assign IDisHalt = (IDopcode == `ALU_OP) && (IDfunct == `INST_FUNC_HLT);
	assign IDisBranch = (IDopcode == `BNE_OP) || (IDopcode == `BEQ_OP) || (IDopcode == `BGZ_OP) || (IDopcode == `BLZ_OP);
	assign IDisWWD = (IDopcode == `ALU_OP) && (IDfunct == `INST_FUNC_WWD);
endmodule

module IFandID(Reset_N, clk, IFIDWrite, IFpc, IFinstruction, EXisJtype, IDisStall, IDisBranch, IDbcond, IDisWWD, IDpc, IDinstruction, IDisValidInstruction, IFstate);
	input Reset_N;
	input clk;
	input IFIDWrite;
	input [`WORD_SIZE-1:0] IFpc;
	input [`WORD_SIZE-1:0] IFinstruction;
	input EXisJtype;
	input IDisStall;
	input IDisBranch;
	input IDbcond;
    input IDisWWD;

	output [`WORD_SIZE-1:0] IDpc;
	output [`WORD_SIZE-1:0] IDinstruction;
	output IDisValidInstruction;
    output IFstate;

	reg [`WORD_SIZE-1:0] IDpc;
	reg [`WORD_SIZE-1:0] IDinstruction;
	reg IDisValidInstruction;
    reg IFstate;

	always @(posedge(clk)) begin //pb1
		if(!Reset_N) begin
				IDpc <= 0;
				IDinstruction <= 0;
				IFstate = 0;
				IDisValidInstruction <= 0;
		end
	end

	always @(posedge(clk)) begin
		if(IFIDWrite && IFstate) begin
			IDpc <= IFpc;
			IDinstruction <= IFinstruction;
			IDisValidInstruction <= 1;
			IFstate <= 0;
		end
		else begin
            if(IFstate) begin
                // if(IDinstruction == 0 && IDpc == 0) IDisValidInstruction = 0;
                //if stall occured in ID, then instruction in ID is invalid
                if(IDisStall) IDisValidInstruction <= 0;

                //if jump in EX, then instruction in ID and IF is invalid
                else if(EXisJtype) begin
                    IDisValidInstruction <= 0;
					IDpc <= 0;
					IDinstruction <= 0;
					IFstate <= 0;
                end

                //if branch taken in ID, then instruction in IF is invalid
                else if(IDisBranch && IDbcond) begin
                    IDisValidInstruction <= 0;
					IDpc <= 0;
					IDinstruction <= 0;
					IFstate <= 0;
                end
            end
            else begin
            	IFstate <= 1;
				IDisValidInstruction <= 0;
            end
		end
	end
endmodule

module IDandEx(Reset_N, clk, IDEXWrite, IDpc, IDReadData1, IDReadData2, IDopcode, IDfunct, IDimmediate, IDrs, IDrt, IDWriteReg,
				IDMemRead, IDMemWrite, IDMemtoReg, IDALUSrc, IDRegWrite, IDisJtype, IDisRtype, IDisLHI,
				IDisJLtype, IDisJRtype, IDisJMPJALLHI, IDisHalt, IDisBranch, IDisWWD, IDisStall, IDisValidInstruction,
				EXpc, EXReadData1, EXReadData2, EXopcode, EXfunct, EXimmediate, EXrs, EXrt, EXWriteReg,
				EXMemRead, EXMemWrite, EXMemtoReg, EXALUSrc, EXRegWrite, EXisJtype, EXisRtype, EXisLHI,
				EXisJLtype, EXisJRtype, EXisJMPJALLHI, EXisHalt, EXisBranch, EXisWWD, EXisValidInstruction);
	input Reset_N;
	input clk;
	input IDEXWrite;
	input [`WORD_SIZE-1:0] IDpc;
	input [`WORD_SIZE-1:0] IDReadData1;
	input [`WORD_SIZE-1:0] IDReadData2;
	input [3:0] IDopcode;
	input [5:0] IDfunct;
	input [`WORD_SIZE-1:0] IDimmediate;
	input [1:0] IDrs;
	input [1:0] IDrt;
	input [1:0] IDWriteReg;

	input IDMemRead;
	input IDMemWrite;
	input IDMemtoReg;
	input IDALUSrc;
	input IDRegWrite;
	input IDisJtype;
	input IDisRtype;
	input IDisLHI;
	input IDisJLtype;
	input IDisJRtype;
	input IDisJMPJALLHI;
	input IDisHalt;
	input IDisBranch;
	input IDisWWD;
	input IDisStall;
	input IDisValidInstruction;

	output [`WORD_SIZE-1:0] EXpc;
	output [`WORD_SIZE-1:0] EXReadData1;
	output [`WORD_SIZE-1:0] EXReadData2;
	output [3:0] EXopcode;
	output [5:0] EXfunct;
	output [`WORD_SIZE-1:0] EXimmediate;
	output [1:0] EXrs;
	output [1:0] EXrt;
	output [1:0] EXWriteReg;

	output EXMemRead;
	output EXMemWrite;
	output EXMemtoReg;
	output EXALUSrc;
	output EXRegWrite;
	output EXisJtype;
	output EXisRtype;
	output EXisLHI;
	output EXisJLtype;
	output EXisJRtype;
	output EXisJMPJALLHI;
	output EXisHalt;
	output EXisBranch;
	output EXisWWD;
	output EXisValidInstruction;

	reg [`WORD_SIZE-1:0] EXpc;
	reg [`WORD_SIZE-1:0] EXReadData1;
	reg [`WORD_SIZE-1:0] EXReadData2;
	reg [3:0] EXopcode;
	reg [5:0] EXfunct;
	reg [`WORD_SIZE-1:0] EXimmediate;
	reg [1:0] EXrs;
	reg [1:0] EXrt;
	reg [1:0] EXWriteReg;

	reg EXMemRead;
	reg EXMemWrite;
	reg EXMemtoReg;
	reg EXALUSrc;
	reg EXRegWrite;
	reg EXisJtype;
	reg EXisRtype;
	reg EXisLHI;
	reg EXisJLtype;
	reg EXisJRtype;
	reg EXisJMPJALLHI;
	reg EXisHalt;
	reg EXisBranch;
	reg EXisWWD;
	reg EXisValidInstruction;

	always @(posedge(clk)) begin
		if(!Reset_N) begin
			EXpc <= 0;
			EXReadData1 <= 0;
			EXReadData2 <= 0;
			EXopcode <= 0;
			EXfunct <= 0;
			EXimmediate <= 0;
			EXrs <= 0;
			EXrt <= 0;
			EXWriteReg <= 0;
			EXMemRead <= 0;
			EXMemWrite <= 0;
			EXMemtoReg <= 0;
			EXALUSrc <= 0;
			EXRegWrite <= 0;
			EXisJtype <= 0;
			EXisRtype <= 0;
			EXisLHI <= 0;
			EXisJLtype <= 0;
			EXisJRtype <= 0;
			EXisJMPJALLHI <= 0;
			EXisHalt <= 0;
			EXisBranch <= 0;
			EXisWWD <= 0;
			EXisValidInstruction <= 0;
		end
	end

	always @(posedge(clk)) begin
		if(IDEXWrite) begin
			EXpc <= IDpc;
			EXReadData1 <= IDReadData1;
			EXReadData2 <= IDReadData2;
			EXopcode <= IDopcode;
			EXfunct <= IDfunct;
			EXimmediate <= IDimmediate;
			EXrs <= IDrs;
			EXrt <= IDrt;
			EXWriteReg <= IDWriteReg;
			EXMemRead <= IDMemRead;
			EXMemWrite <= IDMemWrite;
			EXMemtoReg <= IDMemtoReg;
			EXALUSrc <= IDALUSrc;
			EXRegWrite <= IDRegWrite;
			EXisJtype <= IDisJtype;
			EXisRtype <= IDisRtype;
			EXisLHI <= IDisLHI;
			EXisJLtype <= IDisJLtype;
			EXisJRtype <= IDisJRtype;
			EXisJMPJALLHI <= IDisJMPJALLHI;
			EXisHalt <= IDisHalt;
			EXisBranch <= IDisBranch;
			EXisWWD <= IDisWWD;
			EXisValidInstruction <= IDisValidInstruction;
		end
		else begin
			EXpc <= 0;
			EXReadData1 <= 0;
			EXReadData2 <= 0;
			EXopcode <= 0;
			EXfunct <= 0;
			EXimmediate <= 0;
			EXrs <= 0;
			EXrt <= 0;
			EXWriteReg <= 0;
			EXMemRead <= 0;
			EXMemWrite <= 0;
			EXMemtoReg <= 0;
			EXALUSrc <= 0;
			EXRegWrite <= 0;
			EXisJtype <= 0;
			EXisRtype <= 0;
			EXisLHI <= 0;
			EXisJLtype <= 0;
			EXisJRtype <= 0;
			EXisJMPJALLHI <= 0;
			EXisHalt <= 0;
			EXisBranch <= 0;
			EXisWWD <= 0;
			EXisValidInstruction <= 0;
		end
	end
endmodule

module EXandMEM(Reset_N, clk, EXpc, EXALUresult, EXimmediate, EXWriteReg, EXALUInput2, EXReadData1, EXReadData2,
				EXMemRead, EXMemWrite, EXMemtoReg, EXRegWrite, EXisJLtype, EXisLHI, EXisHalt, EXisWWD, EXisValidInstruction,
				MEMpc, MEMALUresult, MEMimmediate, MEMWriteReg, MEMALUInput2, MEMReadData1,
				MEMMemRead, MEMMemWrite, MEMMemtoReg, MEMRegWrite, MEMisJLtype, MEMisLHI, MEMisHalt, MEMisWWD, MEMisValidInstruction);
	input Reset_N;
	input clk;
	input [`WORD_SIZE-1:0] EXpc;
	input [`WORD_SIZE-1:0] EXALUresult;
	input [`WORD_SIZE-1:0] EXimmediate;
	input [1:0] EXWriteReg;
	input [`WORD_SIZE-1:0] EXALUInput2;
	input [`WORD_SIZE-1:0] EXReadData1;
	input [`WORD_SIZE-1:0] EXReadData2;

	input EXMemRead;
	input EXMemWrite;
	input EXMemtoReg;
	input EXRegWrite;
	input EXisJLtype;
	input EXisLHI;
	input EXisHalt;
	input EXisWWD;
	input EXisValidInstruction;

	output [`WORD_SIZE-1:0] MEMpc;
	output [`WORD_SIZE-1:0] MEMALUresult;
	output [`WORD_SIZE-1:0] MEMimmediate;
	output [1:0] MEMWriteReg;
	output [`WORD_SIZE-1:0] MEMALUInput2;
	output [`WORD_SIZE-1:0] MEMReadData1;

	output MEMMemRead;
	output MEMMemWrite;
	output MEMMemtoReg;
	output MEMRegWrite;
	output MEMisJLtype;
	output MEMisLHI;
	output MEMisHalt;
	output MEMisWWD;
	output MEMisValidInstruction;

	reg [`WORD_SIZE-1:0] MEMpc;
	reg [`WORD_SIZE-1:0] MEMALUresult;
	reg [`WORD_SIZE-1:0] MEMimmediate;
	reg [1:0] MEMWriteReg;
	reg [`WORD_SIZE-1:0] MEMALUInput2;
	reg [`WORD_SIZE-1:0] MEMReadData1;

	reg MEMMemRead;
	reg MEMMemWrite;
	reg MEMMemtoReg;
	reg MEMRegWrite;
	reg MEMisJLtype;
	reg MEMisLHI;
	reg MEMisHalt;
	reg MEMisWWD;
	reg MEMisValidInstruction;

	always @(posedge(clk)) begin
		if(!Reset_N) begin
			MEMpc <= 0;
			MEMALUresult <= 0;
			MEMimmediate <= 0;
			MEMWriteReg <= 0;
			MEMALUInput2 <= 0;
			MEMMemRead <= 0;
			MEMMemWrite <= 0;
			MEMMemtoReg <= 0;
			MEMRegWrite <= 0;
			MEMisJLtype <= 0;
			MEMisLHI <= 0;
			MEMisHalt <= 0;
			MEMisWWD <= 0;
			MEMReadData1 <= 0;
			MEMisValidInstruction <= 0;
		end
	end


	always @(posedge(clk)) begin
		MEMpc <= EXpc;
		if (EXisValidInstruction) MEMALUresult = EXALUresult;
		MEMimmediate <= EXimmediate;
		MEMWriteReg <= EXWriteReg;
		MEMALUInput2 <= EXReadData2;
		MEMMemRead <= EXMemRead;
		MEMMemWrite <= EXMemWrite;
		MEMMemtoReg <= EXMemtoReg;
		MEMRegWrite <= EXRegWrite;
		MEMisJLtype <= EXisJLtype;
		MEMisLHI <= EXisLHI;
		MEMisHalt <= EXisHalt;
		MEMisWWD <= EXisWWD;
		MEMReadData1 <= EXReadData1;
		MEMisValidInstruction <= EXisValidInstruction;
	end
endmodule

module MEMandWB(Reset_N, clk, MEMpc, MEMReadData, MEMALUresult, MEMimmediate, MEMWriteReg, MEMReadData1,
				MEMRegWrite, MEMMemtoReg, MEMisLHI, MEMisJLtype, MEMisHalt, MEMisWWD, WBWriteData, MEMisValidInstruction, MEMMemRead, MEMMemWrite,
				WBpc, WBReadData, WBALUresult, WBimmediate, WBWriteReg, WBReadData1,
				WBRegWrite, WBMemtoReg, WBisLHI, WBisJLtype, WBisHalt, WBisWWD, WBisValidInstruction);
	input Reset_N;
	input clk;
	input [`WORD_SIZE-1:0] MEMpc;
	input [`WORD_SIZE-1:0] MEMReadData;
	input [`WORD_SIZE-1:0] MEMALUresult;
	input [`WORD_SIZE-1:0] MEMimmediate;
	input [1:0] MEMWriteReg;
	input [`WORD_SIZE-1:0] MEMReadData1;

	input MEMRegWrite;
	input MEMMemtoReg;
	input MEMisLHI;
	input MEMisJLtype;
	input MEMisHalt;
	input MEMisWWD;
	input [`WORD_SIZE-1:0] WBWriteData;
	input MEMisValidInstruction;
    input MEMMemRead;
    input MEMMemWrite;

	output [`WORD_SIZE-1:0] WBpc;
	output [`WORD_SIZE-1:0] WBReadData;
	output [`WORD_SIZE-1:0] WBALUresult;
	output [`WORD_SIZE-1:0] WBimmediate;
	output [1:0] WBWriteReg;
	output [`WORD_SIZE-1:0] WBReadData1;

	output WBRegWrite;
	output WBMemtoReg;
	output WBisLHI;
	output WBisJLtype;
	output WBisHalt;
	output WBisWWD;
	output WBisValidInstruction;

	reg [`WORD_SIZE-1:0] WBpc;
	reg [`WORD_SIZE-1:0] WBReadData;
	reg [`WORD_SIZE-1:0] WBALUresult;
	reg [`WORD_SIZE-1:0] WBimmediate;
	reg [1:0] WBWriteReg;
	reg [`WORD_SIZE-1:0] WBReadData1;

	reg WBRegWrite;
	reg WBMemtoReg;
	reg WBisLHI;
	reg WBisJLtype;
	reg WBisHalt;
	reg WBisWWD;
	reg WBisValidInstruction;

    reg MEMstate;
    reg lastValid;

	always @(posedge(clk)) begin
		if(!Reset_N) begin
			WBpc <= 0;
			WBReadData <= 0;
			WBALUresult <= 0;
			WBimmediate <= 0;
			WBWriteReg <= 0;
			WBRegWrite <= 0;
			WBMemtoReg <= 0;
			WBisLHI <= 0;
			WBisJLtype <= 0;
			WBisHalt <= 0;
			WBisWWD <= 0;
			WBReadData1 <= 0;
			WBisValidInstruction <= 0;
	 		MEMstate <= 0;
			lastValid <= 0;
		end
	end

	always @(posedge(clk)) begin
		if(!MEMstate && (MEMMemRead || MEMMemWrite)) begin
			MEMstate <= 1;
			lastValid <= MEMisValidInstruction;
			WBisValidInstruction <= 0;
		end
		else begin
			WBpc <= MEMpc;
			WBReadData <= MEMReadData;
			WBALUresult <= MEMALUresult;
			WBimmediate <= MEMimmediate;
			WBWriteReg <= MEMWriteReg;
			WBRegWrite <= MEMRegWrite;
			WBMemtoReg <= MEMMemtoReg;
			WBisLHI <= MEMisLHI;
			WBisJLtype <= MEMisJLtype;
			WBisHalt <= MEMisHalt;
			WBisWWD <= MEMisWWD;
			WBReadData1 <= MEMReadData1;
			if(MEMMemRead || MEMMemWrite) WBisValidInstruction <= lastValid;
			else WBisValidInstruction <= MEMisValidInstruction;
			MEMstate <= 0;
		end
	end
endmodule

module cpu(Clk, Reset_N, readM1, address1, data1, readM2, writeM2, address2, data2, num_inst, output_port, is_halted);
	input Clk;
	input Reset_N;

	output readM1;
	output [`WORD_SIZE-1:0] address1;
	output readM2;
	output writeM2;
	output [`WORD_SIZE-1:0] address2;

	input [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;

	output [`WORD_SIZE-1:0] num_inst;
	reg [`WORD_SIZE-1:0] num_inst;
	output [`WORD_SIZE-1:0] output_port;
	reg [`WORD_SIZE-1:0] output_port;
	output is_halted;
	// TODO : Implement your pipelined CPU!

	reg [`WORD_SIZE-1:0] pc;
	wire [`WORD_SIZE-1:0] nextpc;
	wire [1:0] PCSrc;
	wire [`WORD_SIZE-1:0] EXReadData1plus1;
	wire [`WORD_SIZE-1:0] IDpcplusIDimmediate;

	wire IDisStall;
	wire [1:0] IDForwardA;
	wire [1:0] IDForwardB;
	wire [1:0] ForwardA;
	wire [1:0] ForwardB;
	wire [`WORD_SIZE-1:0] CompareInput1;
	wire [`WORD_SIZE-1:0] CompareInput2;

	wire [`WORD_SIZE-1:0] IFpc;
	wire [`WORD_SIZE-1:0] IFinstruction;
	wire [`WORD_SIZE-1:0] IFnextpc;
    wire IFstate;
	
	wire [`WORD_SIZE-1:0] IDpc;
	wire [`WORD_SIZE-1:0] IDinstruction;
	wire [3:0] IDopcode;
	wire [1:0] IDrs;
	wire [1:0] IDrt;
	wire [1:0] IDrd;
	wire [5:0] IDfunct;
	wire [7:0] IDimmediate;
	wire [11:0] IDtarget;
	wire [1:0] IDWriteRegtemp;
	wire [1:0] IDWriteReg;
	wire [`WORD_SIZE-1:0] IDReadData1;
	wire [`WORD_SIZE-1:0] IDReadData2;
	wire [15:0] IDimmediate16;
	wire [3:0] IDbcond;
	
	wire IDMemRead;
	wire IDMemWrite;
	wire IDMemtoReg;
	wire IDALUSrc;
	wire IDRegWrite;
	wire IDisJtype;
	wire IDisRtype;
	wire IDisLHI;
	wire IDisJLtype;
	wire IDisJRtype;
	wire IDisJMPJALLHI;
	wire IDisHalt;
	wire IDisBranch;
	wire IDisWWD;
	wire IDisValidInstruction;

	wire [`WORD_SIZE-1:0] EXpc;
	wire [`WORD_SIZE-1:0] EXReadData1;
	wire [`WORD_SIZE-1:0] EXReadData2;
	wire [3:0] EXopcode;
	wire [5:0] EXfunct;
	wire [`WORD_SIZE-1:0] EXimmediate;
	wire [1:0] EXrs;
	wire [1:0] EXrt;
	wire [1:0] EXrd;
	wire [1:0] EXWriteReg;
	wire [`WORD_SIZE-1:0] EXALUInput1;
	wire [`WORD_SIZE-1:0] EXALUInput2;
	wire [`WORD_SIZE-1:0] EXALUresult;
	wire [`WORD_SIZE-1:0] EXALUInput1tmp;
	wire [`WORD_SIZE-1:0] EXALUInput2tmp;
	wire [2:0] EXALUFunc;

	wire EXMemRead;
	wire EXMemWrite;
	wire EXMemtoReg;
	wire EXALUSrc;
	wire EXRegWrite;
	wire EXisJtype;
	wire EXisRtype;
	wire EXisLHI;
	wire EXisJLtype;
	wire EXisJRtype;
	wire EXisHalt;
	wire EXisBranch;
	wire EXisWWD;
	wire EXisValidInstruction;

	wire [`WORD_SIZE-1:0] MEMpc;
	wire [`WORD_SIZE-1:0] MEMReadData;
	wire [`WORD_SIZE-1:0] MEMALUresult;
	wire [`WORD_SIZE-1:0] MEMimmediate;
	wire [1:0] MEMWriteReg;
	wire [`WORD_SIZE-1:0] MEMALUInput2;
	wire [`WORD_SIZE-1:0] MEMReadData1;

	wire MEMMemRead;
	wire MEMMemWrite;
	wire MEMMemtoReg;
	wire MEMRegWrite;
	wire MEMisJLtype;
	wire MEMisLHI;
	wire MEMisHalt;
	wire MEMisWWD;
	wire MEMisValidInstruction;

	wire [`WORD_SIZE-1:0] WBpc;
	wire [`WORD_SIZE-1:0] WBReadData;
	wire [`WORD_SIZE-1:0] WBALUresult;
	wire [`WORD_SIZE-1:0] WBimmediate;
	wire [1:0] WBWriteReg;
	wire [`WORD_SIZE-1:0] WBWriteData;
	wire [`WORD_SIZE-1:0] WBWriteDatatmp1;
	wire [`WORD_SIZE-1:0] WBWriteDatatmp2;
	wire [`WORD_SIZE-1:0] WBReadData1;
	wire [`WORD_SIZE-1:0] WBpcplus1 = WBpc+1;

	wire WBRegWrite;
	wire WBMemtoReg;
	wire WBisLHI;
	wire WBisJLtype;
	wire WBisHalt;
	wire WBisWWD;
	wire WBisValidInstruction;

	wire PCWrite;

	always @(posedge(Clk)) begin
		if(PCWrite) begin
			pc = nextpc;
		end
	end

    always @(WBisWWD) begin
        if(WBisWWD) output_port = MEMReadData1;
        else output_port = 16'bz;
    end

	assign IFpc = pc;
	assign IFnextpc = IFpc+1;
	assign WBpcplus1 = WBpc+1;
	assign IDpcplusIDimmediate = IDpc + IDimmediate16 + 1;

	always @(negedge(Clk)) begin
        	if(WBisValidInstruction) num_inst = num_inst+1;
    	end
	HazardDetectionUnit hazarddetectionunit(IDisBranch, EXMemRead, IDrs, IDrt, IDopcode, IDpc, IDbcond, EXrt, EXrd, 
											EXisJtype, EXRegWrite, EXisRtype, PCWrite, IFIDWrite, IDisStall, IDEXWrite);
	ForwardingUnit forwardingunit(MEMRegWrite, MEMWriteReg, WBRegWrite, WBWriteReg, EXrs, EXrt, EXopcode, EXWriteReg, EXRegWrite, IDrs, IDrt, IDopcode, ForwardA, ForwardB, IDForwardA, IDForwardB);
	PCSrcGenerator pcsrcgenerator(Clk, Reset_N, IDisBranch, IDbcond, IDopcode, EXisJRtype, EXisJtype, PCSrc);

	MUX16_pc pcMux(IFnextpc, IDpcplusIDimmediate, EXALUresult, EXALUInput1, IFpc, PCSrc, IFstate, nextpc);
	InstructionMemory instructionmemory(Clk, Reset_N, IFpc, data1, readM1, address1, IFinstruction);
	IFandID IFandID(Reset_N, Clk, IFIDWrite, IFpc, IFinstruction, EXisJtype, IDisStall, IDisBranch, IDbcond[IDopcode], IDisWWD, IDpc, IDinstruction, IDisValidInstruction, IFstate);
	InstructionDecoder instructiondecoder(IDinstruction, IDopcode, IDrs, IDrt, IDrd, IDfunct, IDimmediate, IDtarget);
	Control control(IDopcode, IDfunct, IDisStall, IDMemRead, IDMemWrite, IDMemtoReg, IDALUSrc, IDRegWrite, IDisJtype, IDisRtype, IDisLHI, IDisJLtype, IDisJRtype, IDisJMPJALLHI, IDisHalt, IDisBranch, IDisWWD);
	GPR pregisters(Clk, IDrs, IDrt, WBWriteReg, WBWriteData, WBRegWrite, IDReadData1, IDReadData2);
	MUX16_4 branchCompare1Mux(IDReadData1, WBWriteDatatmp1, MEMALUresult, EXALUresult, IDForwardA, CompareInput1);
	MUX16_4 branchCompare2Mux(IDReadData2, WBWriteDatatmp1, MEMALUresult, EXALUresult, IDForwardB, CompareInput2);
	Comparator comparator(CompareInput1, CompareInput2, IDbcond);
	imm_gen immgenerater(IDopcode, IDimmediate, IDpc, IDtarget, IDimmediate16);
	Mux2_2 WriteRegMux(IDrt, IDrd, IDisRtype, IDWriteRegtemp);
	Mux2_2 WriteRegMux2(IDWriteRegtemp, 2'd2, IDisJLtype, IDWriteReg);
	IDandEx IDandEx(Reset_N, Clk, IDEXWrite, IDpc, IDReadData1, IDReadData2, IDopcode, IDfunct, IDimmediate16, IDrs, IDrt, IDWriteReg,
						IDMemRead, IDMemWrite, IDMemtoReg, IDALUSrc, IDRegWrite, IDisJtype, IDisRtype, IDisLHI,
						IDisJLtype, IDisJRtype, IDisJMPJALLHI, IDisHalt, IDisBranch, IDisWWD, IDisStall, IDisValidInstruction,
						EXpc, EXReadData1, EXReadData2, EXopcode, EXfunct, EXimmediate, EXrs, EXrt, EXWriteReg,
						EXMemRead, EXMemWrite, EXMemtoReg, EXALUSrc, EXRegWrite, EXisJtype, EXisRtype, EXisLHI,
						EXisJLtype, EXisJRtype, EXisJMPJALLHI, EXisHalt, EXisBranch, EXisWWD, EXisValidInstruction);
	Mux16_2 ALUinput1Mux1(EXReadData1, 16'b0, EXisJMPJALLHI, EXALUInput1tmp);
	Mux16_2 ALUinput2Mux1(EXReadData2, EXimmediate, EXALUSrc, EXALUInput2tmp);
	MUX16_4 ALUinput1Mux2(EXALUInput1tmp, WBWriteDatatmp1, MEMALUresult, 16'b0, ForwardA, EXALUInput1);
	MUX16_4 ALUinput2Mux2(EXALUInput2tmp, WBWriteDatatmp1, MEMALUresult, 16'b0, ForwardB, EXALUInput2);
	ALU_Control alucontrol(EXfunct, EXopcode, EXALUFunc);
	ALU alu(EXALUInput1, EXALUInput2, EXALUFunc, EXALUresult);
	EXandMEM EXandMEM(Reset_N, Clk, EXpc, EXALUresult, EXimmediate, EXWriteReg, EXALUInput2, EXALUInput1, EXReadData2,
						EXMemRead, EXMemWrite, EXMemtoReg, EXRegWrite, EXisJLtype, EXisLHI, EXisHalt, EXisWWD, EXisValidInstruction,
						MEMpc, MEMALUresult, MEMimmediate, MEMWriteReg, MEMALUInput2, MEMReadData1,
						MEMMemRead, MEMMemWrite, MEMMemtoReg, MEMRegWrite, MEMisJLtype, MEMisLHI, MEMisHalt, MEMisWWD, MEMisValidInstruction);
	DataMemory datamemory(Clk, Reset_N, MEMALUresult, MEMALUInput2, data2, MEMMemWrite, MEMMemRead, writeM2, readM2, address2, MEMReadData);
	MEMandWB MEMandWB(Reset_N, Clk, MEMpc, MEMReadData, MEMALUresult, MEMimmediate, MEMWriteReg, MEMReadData1,
						MEMRegWrite, MEMMemtoReg, MEMisLHI, MEMisJLtype, MEMisHalt, MEMisWWD, WBWriteData, MEMisValidInstruction, MEMMemRead, MEMMemWrite,
						WBpc, WBReadData, WBALUresult, WBimmediate, WBWriteReg, WBReadData1,
						WBRegWrite, WBMemtoReg, WBisLHI, WBisJLtype, WBisHalt, WBisWWD, WBisValidInstruction);
	Mux16_2 WriteDataMux1(WBALUresult, WBReadData, WBMemtoReg, WBWriteDatatmp1);
	Mux16_2 WriteDataMux2(WBWriteDatatmp1, WBimmediate, WBisLHI, WBWriteDatatmp2);
	Mux16_2 WriteDataMux3(WBWriteDatatmp2, WBpcplus1, WBisJLtype, WBWriteData);

	assign data2 = (writeM2) ? MEMALUInput2 : 16'bz;
	assign is_halted = WBisHalt && WBisValidInstruction;

	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			pc <= 0;
			num_inst <= 0;
		end
	end
   

endmodule
