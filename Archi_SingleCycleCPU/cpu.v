`include "opcodes.v"


//ALU
module ALU (A, B, FuncCode, C, BranchCondition);
   input [`WORD_SIZE-1:0] A;
   input [`WORD_SIZE-1:0] B;
   input [2:0] FuncCode;
   output [`WORD_SIZE-1:0] C;
   output [3:0] BranchCondition;

   reg [`WORD_SIZE-1:0] C;
   reg [3:0] BranchCondition;
   reg signed [`WORD_SIZE-1:0] AS;

   always @(*) begin
      case (FuncCode)
         `FUNC_ADD: begin
            C = A + B;
         end
         `FUNC_SUB: begin
            C = A - B;
         end
         `FUNC_AND: begin
            C = A & B;
         end
         `FUNC_ORR: begin
            C = A | B;
         end
         `FUNC_NOT: begin
            C = ~A;
         end
         `FUNC_TCP: begin
            C = ~A + 1;
         end
         `FUNC_SHL: begin
            C = A <<< 1;
         end
         `FUNC_SHR: begin
            AS = A;
            C = AS >>> 1;
         end
      endcase
      BranchCondition[0] = ((A-B) != 0) ? 1 : 0;
      BranchCondition[1] = ((A-B) == 0) ? 1 : 0;
      BranchCondition[2] = (A > 0) ? 1 : 0;
      BranchCondition[3] = (A < 0) ? 1 : 0;
   end
endmodule


//ALU control
module ALUControl(InstFunc, opcode, ALUFunc);
   input [5:0] InstFunc;
   input [3:0] opcode;
   output [2:0] ALUFunc;

   reg [2:0] ALUFunc;
   always @(InstFunc, opcode) begin
      case (opcode)
         `ALU_OP: begin
            case(InstFunc)
               `INST_FUNC_ADD: ALUFunc = `FUNC_ADD;
               `INST_FUNC_SUB: ALUFunc = `FUNC_SUB;
               `INST_FUNC_AND: ALUFunc = `FUNC_AND;
               `INST_FUNC_ORR: ALUFunc = `FUNC_ORR;
               `INST_FUNC_NOT: ALUFunc = `FUNC_NOT;
               `INST_FUNC_TCP: ALUFunc = `FUNC_TCP;
               `INST_FUNC_SHL: ALUFunc = `FUNC_SHL;
               `INST_FUNC_SHR: ALUFunc = `FUNC_SHR;
            endcase
         end
         `ADI_OP: ALUFunc = `FUNC_ADD;
         `ORI_OP: ALUFunc = `FUNC_ORR;
         `LWD_OP: ALUFunc = `FUNC_ADD;
         `SWD_OP: ALUFunc = `FUNC_ADD;
         `BNE_OP: ALUFunc = `FUNC_SUB;
         `BEQ_OP: ALUFunc = `FUNC_SUB;
      endcase
   end
endmodule


//Instruction Decoder(Instruction Memory)
module InstructionDecoder(inst, opcode, rs, rt, rd, func, imm, target);
   input [`WORD_SIZE-1:0] inst;
   output [3:0] opcode;
   output [1:0] rs;
   output [1:0] rt;
   output [1:0] rd;
   output [5:0] func;
   output [7:0] imm;
   output [11:0] target;

   assign opcode = inst[15:12];
   assign rs = inst[11:10];
   assign rt = inst[9:8];
   assign rd = inst[7:6];
   assign func = inst[5:0];
   assign imm = inst[7:0];
   assign target = inst[11:0];
endmodule


//Registers
module Registers(readData1, readData2, readReg1, readReg2, writeReg, writeData, RegWrite);
   input [1:0] readReg1;
   input [1:0] readReg2;
   input [1:0] writeReg;
   input [`WORD_SIZE-1:0] writeData;
   input RegWrite; //if 1, write writeData on writeReg
   output [`WORD_SIZE-1:0] readData1;
   output [`WORD_SIZE-1:0] readData2;

   // in this lab we have 4 REGS
   reg [`WORD_SIZE-1:0] REGISTER [`NUM_REGS-1:0];
   integer i;

   initial begin
      for(i = 0; i < `NUM_REGS; i = i + 1) begin
         REGISTER[i] <= 16'b0;
      end
   end

   always @(RegWrite) begin
      if (RegWrite) begin
         REGISTER[writeReg] = writeData;
      end
   end

   assign readData1 = REGISTER[readReg1];
   assign readData2 = REGISTER[readReg2];
endmodule


//Imm Gen
module ImmGen(opcode, imm, pc, target, imm16);
   input [3:0] opcode;
   input [7:0] imm;
   input [`WORD_SIZE-1:0] pc;
   input [11:0] target;
   output [`WORD_SIZE-1:0] imm16;

   reg [`WORD_SIZE-1:0] imm16;

   integer i;

   always @(opcode, imm) begin
      case (opcode)
         //Rtype
         `ALU_OP: begin
            imm16 = 16'b0;
         end
         //Itype
         `ADI_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
         `ORI_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = 0;
            end
            imm16[7:0] = imm[7:0];
         end
         `LHI_OP: begin
            for(i=0; i<8; i=i+1) begin
               imm16[i] = 0;
            end
            imm16[15:8] = imm[7:0];
         end
	//load
         `LWD_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
	//store
         `SWD_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
	//branch condition
         `BNE_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
         `BEQ_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
         `BGZ_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
         `BLZ_OP: begin
            for(i=15; i>7; i=i-1) begin
               imm16[i] = imm[7];
            end
            imm16[7:0] = imm[7:0];
         end
         //Jtype
         `JMP_OP: begin
            for(i=15; i>11; i=i-1) begin
               imm16[i] = pc[i];
            end
            imm16[11:0] = target[11:0];
         end
         `JAL_OP: begin
            for(i=15; i>11; i=i+1) begin
               imm16[i] = pc[i];
            end
            imm16[11:0] = target[11:0];
         end 
      endcase
   end
endmodule


//16bit MUX
module MUX16(A, B, S, O);
   input [`WORD_SIZE-1:0] A;
   input [`WORD_SIZE-1:0] B;
   input S;
   output [`WORD_SIZE-1:0] O;

   reg [`WORD_SIZE-1:0]O;

   always @(*) begin
      if (S == 1'b0) begin
         O = A;
      end
      else begin
         O = B;
      end
   end
endmodule


//2bit MUX
module MUX2(A, B, S, O);
   input [1:0] A;
   input [1:0] B;
   input S;
   output [1:0] O;

   reg [1:0]O;

   always @(*) begin
      if (S == 1'b0) begin
         O = A;
      end
      else begin
         O = B;
      end
   end
endmodule


//JumpControl
module JmpControl(opcode, funct, readData1, immediate16, PCFromJmp);
   input [3:0] opcode;
   input [5:0] funct;
   input [`WORD_SIZE-1:0] readData1;
   input [`WORD_SIZE-1:0] immediate16;
   output [`WORD_SIZE-1:0] PCFromJmp;

   reg [`WORD_SIZE-1:0] PCFromJmp;

   always @(immediate16, readData1) begin
      case (opcode)
         `JMP_OP: begin
            PCFromJmp = immediate16-1;
         end
         `JAL_OP: begin
            PCFromJmp = immediate16-1;
         end
         `JPR_OP: begin
            case (funct)
               `INST_FUNC_JPR: begin
                  PCFromJmp = readData1-1;
               end
               `INST_FUNC_JRL: begin
                  PCFromJmp = readData1-1;
               end
            endcase
         end
      endcase
   end
endmodule

//Control
module Control (opcode, funct, ReadytoWrite_reg, IsRtype, PCtoReg, RegWrite, ALUSrc, MemWrite, MemtoReg, MemRead, Branch, Jmp, Lhi);
   input [3:0] opcode;
   input [5:0] funct;
   input ReadytoWrite_reg;
   output IsRtype;
   output PCtoReg;
   output RegWrite;
   output ALUSrc;
   output MemWrite;
   output MemtoReg;
   output MemRead;
   output Branch;
   output Jmp;
   output Lhi;

   reg IsRtype;
   reg PCtoReg;
   reg RegWrite;
   reg ALUSrc;
   reg MemWrite;
   reg MemtoReg;
   reg MemRead;
   reg Branch;
   reg Jmp;
   reg Lhi;

   always @(opcode, ReadytoWrite_reg) begin
      IsRtype = (opcode == `ALU_OP);
      PCtoReg = ((opcode == `JRL_OP) && (funct == `INST_FUNC_JRL)) || (opcode == `JAL_OP);
      RegWrite = ReadytoWrite_reg && (opcode != `SWD_OP) && ((opcode != `BNE_OP) && (opcode != `BEQ_OP) && (opcode != `BGZ_OP) && (opcode != `BLZ_OP)) && (opcode != `JMP_OP) && ((opcode != `JRL_OP) || (funct != `INST_FUNC_JRL));
      ALUSrc = (opcode != `ALU_OP) && ((opcode != `BNE_OP) && (opcode != `BEQ_OP) && (opcode != `BGZ_OP) && (opcode != `BLZ_OP));
      MemWrite = (opcode == `SWD_OP);
      MemtoReg = (opcode == `LWD_OP);
      MemRead = (opcode == `LWD_OP);
      Branch = (opcode == `BNE_OP) || (opcode == `BEQ_OP) || (opcode == `BGZ_OP) || (opcode == `BLZ_OP);
      Jmp = (opcode == `JMP_OP) || (opcode == `JAL_OP) || ((opcode == `JPR_OP) && (funct == `INST_FUNC_JPR)) || ((opcode == `JRL_OP) && (funct == `INST_FUNC_JRL));
      Lhi = (opcode == `LHI_OP);
   end
endmodule

//cpu
module cpu (readM, writeM, address, data, ackOutput, inputReady, reset_n, clk);
   output readM;                           
   output writeM;                        
   output [`WORD_SIZE-1:0] address;   
   inout [`WORD_SIZE-1:0] data;      
   input ackOutput;         
   input inputReady;   
   input reset_n;   
   input clk;   

   reg [`WORD_SIZE-1:0] tempData;
   reg [`WORD_SIZE-1:0] pc;
   wire [3:0] opcode;
   wire [1:0] rs;
   wire [1:0] rt;
   wire [1:0] rd;
   wire [5:0] funct;
   wire [7:0] immediate;
   wire [11:0] target;
   wire [`WORD_SIZE-1:0] imm_full;

   wire [`WORD_SIZE-1:0] readData1;
   wire [`WORD_SIZE-1:0] readData2;
   wire [1:0] readReg1;
   wire [1:0] readReg2;
   wire [1:0] writeReg;
   wire [`WORD_SIZE-1:0] writeData;

   wire [`WORD_SIZE-1:0] A;
   wire [`WORD_SIZE-1:0] B;
   wire [2:0] ALUFunc;
   wire [`WORD_SIZE-1:0] ALUresult;
   wire [3:0] bcond;

   reg ReadytoWrite_reg;
   reg [`WORD_SIZE-1:0] DataFromMem;
   wire IsRtype;
   wire PCtoReg;
   wire RegWrite;
   wire ALUSrc;
   wire MemWrite;
   wire MemtoReg;
   wire MemRead;
   wire Branch;
   wire Jmp;
   wire Lhi;
   
   wire [1:0] TypeWriteReg;
   wire [1:0] integer2;
   assign integer2 = 2'd2;

   wire [`WORD_SIZE-1:0] BranchPC;
   wire [`WORD_SIZE-1:0] NextPC; 
   wire [`WORD_SIZE-1:0] PCformBranch;
   wire BranchChecker;
   
   assign PCformBranch = imm_full + pc;
   assign BranchChecker = Branch && bcond[opcode];

   wire [`WORD_SIZE-1:0] PCFromJmp;

   wire [`WORD_SIZE-1:0] MemMuxALU;
   wire [`WORD_SIZE-1:0] PCMuxMem;

   //instruction decode
   InstructionDecoder InstDec(tempData, opcode, rs, rt, rd, funct, immediate, target);
   //control flow
   Control controlFlow(opcode, funct, ReadytoWrite_reg, IsRtype, PCtoReg, RegWrite, ALUSrc, MemWrite, MemtoReg, MemRead, Branch, Jmp, Lhi);

   //registers
   MUX2 TyperegWriteMux(rt, rd, IsRtype, TypeWriteReg);
   MUX2 JmpregWriteMux(TypeWriteReg, integer2, PCtoReg, writeReg);

   Registers pregister(A, readData2, rs, rt, writeReg, writeData, RegWrite);
   ImmGen immgen(opcode, immediate, pc, target, imm_full);
   MUX16 ALUSrcMux(readData2, imm_full, ALUSrc, B);

   //execute
   ALUControl aluCon(funct, opcode, ALUFunc);
   ALU alu(A, B, ALUFunc, ALUresult, bcond);

   JmpControl jmpCon(opcode, funct, readData1, imm_full, PCFromJmp);
   
   MUX16 BranchPCMux(pc, PCformBranch, BranchChecker, BranchPC);
   MUX16 JumpPCMux(BranchPC, PCFromJmp, Jmp, NextPC);

   //mem

   //write back
   MUX16 MemtoRegMux(ALUresult, DataFromMem, MemtoReg, MemMuxALU);
   MUX16 PCtoRegMux(MemMuxALU, PCFromJmp, PCtoReg, PCMuxMem);
   MUX16 ImmtoRegMux(PCMuxMem, imm_full, Lhi, writeData);

// Fill it your codes
   reg readM;                           
   reg writeM;                        
   reg [`WORD_SIZE-1:0] address;   
   reg [`WORD_SIZE-1:0] data;

   initial begin
      pc = -1;
      writeM = 0;
      readM = 0;
   end

   assign data = (writeM) ? readData2 : 16'bz;

   always @(posedge(clk)) begin
      ReadytoWrite_reg = 1;
      readM = 1;
      if(readM) begin
         if(NextPC) begin
            pc = NextPC + 1;
         end
         else begin
            pc = pc+1;
         end
         address = pc;
         wait(inputReady);
         ReadytoWrite_reg = 0;
         tempData = data;
         readM = 0;
      end
   end

   always @(negedge(inputReady)) begin
      if(MemWrite) begin
         address[`WORD_SIZE-1:0] = ALUresult[`WORD_SIZE-1:0];
         writeM = 1;
         wait(ackOutput);
         writeM = 0;
      end
      else if(MemRead) begin
         address[`WORD_SIZE-1:0] = ALUresult[`WORD_SIZE-1:0];
         readM = 1;
         wait(inputReady);
         DataFromMem = data;
         readM = 0;
      end
   end
endmodule
