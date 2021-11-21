`timescale 1ns/1ns
`include "const.v"

module HazardDetectionUnit(clk, reset_n, IDrs, IDrt, EXrt, IDuse_rs, IDuse_rt, branchtaken, EX_is_J_type, EX_is_JR_type, is_halted, EXisLWD, EX_is_real, PCWrite, IFIDWrite, setZero, PC_src);
	input clk;
	input reset_n;
	input [1:0] IDrs;
	input [1:0] IDrt;
	input [1:0] EXrt;
	input IDuse_rs;
	input IDuse_rt;
	input branchtaken;
	input EX_is_J_type;
	input EX_is_JR_type;
	input is_halted;
	input EXisLWD;
	input EX_is_real;

	output PCWrite;
	output IFIDWrite;
	output setZero;
	output [1:0] PC_src;

	reg [1:0] stallreg;

	wire stall;
	assign stall = (EXisLWD && ((IDrs == EXrt) && IDuse_rs)) ||
					(EXisLWD && ((IDrt == EXrt) && IDuse_rt));
	assign PCWrite = !is_halted && !stall;
	assign IFIDWrite = !is_halted && !stall;
	assign setZero = (stall || (stallreg[0] | stallreg[1]));
	assign PC_src = (EX_is_real && branchtaken) ? 2'b01 : (EX_is_real && EX_is_JR_type) ? 2'b10 : (EX_is_real && EX_is_J_type) ? 2'b11 : 2'b00;

	always @(*) begin
		if (EX_is_real && (branchtaken || EX_is_J_type)) begin
			stallreg <= 2'b10;
		end
	end
// && !stallreg[0] && !stallreg[1]
	always @(posedge(clk)) begin
		if (!reset_n) begin
			stallreg <= 0;
		end
		else begin
			stallreg <= stallreg >> 1;
		end
	end
endmodule

module ForwardingUnit(MEMRegWrite, MEMWriteReg, WBRegWrite, WBWriteReg, EXrs, EXrt, EXuse_rs, EXuse_rt, ForwardA, ForwardB, forwardornot);
	input MEMRegWrite;
	input [1:0] MEMWriteReg;
	input WBRegWrite;
	input [1:0] WBWriteReg;
	input [1:0] EXrs;
	input [1:0] EXrt;
	input EXuse_rs;
	input EXuse_rt;
	
	output [1:0] ForwardA;
	output ForwardB;
	output forwardornot;

	assign ForwardA[0] = EXuse_rs && ((EXrs == WBWriteReg) && WBRegWrite);
	assign ForwardA[1] = EXuse_rs && ((EXrs == MEMWriteReg) && MEMRegWrite);
	assign ForwardB = EXuse_rt && ((EXrt == MEMWriteReg) && MEMRegWrite);
	assign forwardornot = (EXuse_rt && ((EXrt == MEMWriteReg) && MEMRegWrite)) || (EXuse_rt && ((EXrt == WBWriteReg) && WBRegWrite));
endmodule
