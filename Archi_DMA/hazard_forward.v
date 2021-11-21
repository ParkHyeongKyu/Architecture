`timescale 1ns/1ns
`include "const.v"

module HazardDetectionUnit(
	clk, reset_n, IC_missed, DC_missed, IDrs, IDrt, EXrt, IDuse_rs, IDuse_rt, branchtaken, EX_is_J_type, EX_is_JR_type, is_halted, EXisLWD, EX_is_real, access_mem,
	read_IC, PCWrite, IFIDWrite, IDEXWrite, EXMEMWrite, setZero, PC_src, LWDstalled
);
	input clk;
	input reset_n;
	input IC_missed;
	input DC_missed;
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
	input access_mem;

	output read_IC;
	output PCWrite;
	output IFIDWrite;
	output IDEXWrite;
	output EXMEMWrite;
	output setZero;
	output [1:0] PC_src;
	output LWDstalled;

	reg [1:0] stallreg;

	wire stall;

	reg LWDstalled;

	always @(posedge(clk)) begin
		if (!reset_n) begin
			LWDstalled <= 0;
		end
		if (stall) begin
			LWDstalled <= 1;
		end
		else begin
			LWDstalled <= 0;
		end
	end

	assign read_IC = !IC_missed;
	assign stall = !access_mem || ((EX_is_real && EXisLWD && ((IDrs == EXrt) && IDuse_rs)) ||
					(EX_is_real && EXisLWD && ((IDrt == EXrt) && IDuse_rt)));

	assign PCWrite = !IC_missed && !DC_missed && !is_halted && !stall || (EX_is_real && (branchtaken || EX_is_J_type));
	assign IFIDWrite = !is_halted && !stall && !IC_missed && !DC_missed;
	assign IDEXWrite = !DC_missed;
	assign EXMEMWrite = !DC_missed;
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

module ForwardingUnit(MEM_is_real, WB_is_real, MEMRegWrite, MEMWriteReg, WBRegWrite, WBWriteReg, EXrs, EXrt, EXuse_rs, EXuse_rt, ForwardA, ForwardB, forwardornot);
	input MEMRegWrite;
	input [1:0] MEMWriteReg;
	input WBRegWrite;
	input [1:0] WBWriteReg;
	input [1:0] EXrs;
	input [1:0] EXrt;
	input EXuse_rs;
	input EXuse_rt;
	input MEM_is_real;
	input WB_is_real;
	
	output [1:0] ForwardA;
	output ForwardB;
	output forwardornot;

	assign ForwardA[0] = WB_is_real && EXuse_rs && ((EXrs == WBWriteReg) && WBRegWrite);
	assign ForwardA[1] = MEM_is_real && EXuse_rs && ((EXrs == MEMWriteReg) && MEMRegWrite);
	assign ForwardB = EXuse_rt && ((EXrt == MEMWriteReg) && MEMRegWrite);
	assign forwardornot = (EXuse_rt && ((EXrt == MEMWriteReg) && MEMRegWrite)) || (WB_is_real && EXuse_rt && ((EXrt == WBWriteReg) && WBRegWrite));
endmodule
