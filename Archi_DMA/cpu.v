`timescale 1ns/1ns
`include "const.v"

module cpu(Clk, Reset_N, dma_interrupt, Interrupt, BR, readM1, address1, data1, readM2, writeM2, address2, data2, num_inst, output_port, is_halted, BG, length);
	input Clk;
	wire Clk;
	input Reset_N;
	wire Reset_N;

	input dma_interrupt;
	wire dma_interrupt;
	input Interrupt;
	wire Interrupt;
	input BR;
	wire BR;	

	output BG;
	reg BG;

	output length;
	reg length;

	output readM1;
	wire readM1;
	output [`WORD_SIZE-1:0] address1;
	wire [`WORD_SIZE-1:0] address1;
	output readM2;
	wire readM2;
	output writeM2;
	wire writeM2;
	output [`WORD_SIZE-1:0] address2;
	wire [`WORD_SIZE-1:0] address2;

	input [`WORD_SIZE-1:0] data1;
	wire [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;
	wire [`WORD_SIZE-1:0] data2;

	output [`WORD_SIZE-1:0] num_inst;
	wire [`WORD_SIZE-1:0] num_inst;
	output [`WORD_SIZE-1:0] output_port;
	wire [`WORD_SIZE-1:0] output_port;
	output is_halted;
	wire is_halted;
	// TODO : Implement your pipelined CPU!

	// hazard detect
	wire PCWrite;
	wire IFIDWrite;
	wire setZero;
	wire [1:0] PC_src;
	// forwarding
	wire [1:0] ForwardA;
	wire ForwardB;
	wire forwardornot;
	// alu control
	wire [2:0] ALU_func;
	// main control
	wire readM;
	wire writeM;
	wire Reg_write;
	wire is_JMPJALLHI;
	wire ALU_src_B;
	wire is_halt;
	wire is_WWD;
	wire storeRT;
	wire is_JL_type;
	wire is_J_type;
	wire is_JR_type;
	wire use_rs;
	wire use_rt;
	wire [1:0] m2r;
	// ALU
	wire [`WORD_SIZE-1:0] alu_out;
	// adder
	wire [`WORD_SIZE-1:0] PC_p_one;
	wire [`WORD_SIZE-1:0] EX_PC_p_one_p_imm;
	// imm gen
	wire [`WORD_SIZE-1:0] imm_16;
	// gpr
	wire [`WORD_SIZE-1:0] read_data_1;
	wire [`WORD_SIZE-1:0] read_data_2;
	// pc
	wire [`WORD_SIZE-1:0] current_pc;
	// MUX2
	wire [1:0] RrtorRrd;
	wire [1:0] write_reg;
	// MUX16_2
	wire [`WORD_SIZE-1:0] input_A;
	wire [`WORD_SIZE-1:0] input_B;
	wire [`WORD_SIZE-1:0] forwardB_input;
	wire [`WORD_SIZE-1:0] ALU_input_B;
	wire [`WORD_SIZE-1:0] buffer_rt;
	// MUX16_4
	wire [`WORD_SIZE-1:0] next_pc;
	wire [`WORD_SIZE-1:0] ALU_input_A;
	wire [`WORD_SIZE-1:0] WBresult;
	// IFnID
	wire [`WORD_SIZE-1:0] ID_PC_plus_one;
	wire [3:0] ID_opcode;
	wire [1:0] ID_rs;
	wire [1:0] ID_rt;
	wire [1:0] ID_rd;
	wire [5:0] ID_funct;
	wire [7:0] ID_imm_offset;
	wire [11:0] ID_target;
	wire [`WORD_SIZE-1:0] ID_current_pc;
	// IDnEX
	wire is_empty;
	wire EX_is_real;
	wire [`WORD_SIZE-1:0] EX_PC_plus_one;
	wire [`WORD_SIZE-1:0] EX_read_rs;
	wire [`WORD_SIZE-1:0] EX_read_rt;
	wire [3:0] EX_opcode;
	wire [5:0] EX_funct;
	wire [`WORD_SIZE-1:0] EX_imm16;
	wire [1:0] EX_rs;
	wire [1:0] EX_rt;
	wire [1:0] EX_write_reg;
	wire EX_EX_ALU_src_A;
	wire EX_EX_ALU_src_B;
	wire EX_MEM_readM;
	wire EX_MEM_writeM;
	wire [1:0] EX_WB_memtoreg;
	wire EX_WB_reg_write;
	wire EX_use_rs;
	wire EX_use_rt;
	wire EX_is_J_type;
	wire EX_is_JR_type;
	// EXnMEM
	wire MEM_is_real;
	wire [`WORD_SIZE-1:0] MEM_PC_plus_one;
	wire [`WORD_SIZE-1:0] MEM_ALU_result;
	wire [`WORD_SIZE-1:0] MEM_write_data;
	wire [1:0] MEM_write_reg;
	wire [1:0] MEM_WB_memtoreg;
	wire MEM_WB_reg_write;
	// MEMnWB
	wire WB_is_real;
	wire [`WORD_SIZE-1:0] WB_PC_plus_one;
	wire [`WORD_SIZE-1:0] WB_ALU_result;
	wire [`WORD_SIZE-1:0] WB_read_data;
	wire [1:0] WB_write_reg;
	wire [1:0] WB_WB_memtoreg;
	wire WB_WB_reg_write;
	//cache
	wire [`WORD_SIZE-1:0] IC_cached_data;
	wire IC_missed;
	wire [`WORD_SIZE-1:0] DC_cached_data;
	wire DC_missed;

	//branch
	wire [3:0] bcond;
	assign bcond[0] = ((ALU_input_A - ALU_input_B) != 0) ? 1 : 0;
	assign bcond[1] = ((ALU_input_A - ALU_input_B) == 0) ? 1 : 0;
	assign bcond[2] = ((ALU_input_A < 16'h8000) && (ALU_input_A > 16'h0000)) ? 1 : 0;
	assign bcond[3] = (ALU_input_A > 16'h7fff) ? 1 : 0;
	wire branchtaken;
	assign branchtaken = ((EX_opcode == `BNE_OP) || (EX_opcode == `BEQ_OP) || (EX_opcode == `BGZ_OP) || (EX_opcode == `BLZ_OP)) && bcond[EX_opcode];

	//
	wire read_IC;
	wire fetched;
	wire is_real;
	wire read_DC;
	wire write_DC;
	wire LWDstalled;
	wire IDEXWrite;

	reg access_mem;

	
	reg IFIDWrite_temp;

	assign is_real = ((!Interrupt && access_mem) && LWDstalled) || !setZero && IFIDWrite_temp && fetched;
	
	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			IFIDWrite_temp <= 0;
		end
		if(IFIDWrite) begin
			IFIDWrite_temp <= 1;
		end
		else if (!DC_missed) begin
			IFIDWrite_temp <= 0;
		end
	end

	// modules
	// control
	HazardDetectionUnit hazdet(Clk, Reset_N, IC_missed, DC_missed, ID_rs, ID_rt, EX_rt, use_rs, use_rt, branchtaken, EX_is_J_type, EX_is_JR_type, is_halted, EX_MEM_readM, EX_is_real, access_mem,
	read_IC, PCWrite, IFIDWrite, IDEXWrite, EXMEMWrite, setZero, PC_src, LWDstalled);
	ForwardingUnit forwd(MEM_is_real, WB_is_real, MEM_WB_reg_write, MEM_write_reg, WB_WB_reg_write, WB_write_reg, EX_rs, EX_rt, EX_use_rs, EX_use_rt, ForwardA, ForwardB, forwardornot);
	main_control CONTROL(ID_opcode, ID_funct, readM, writeM, Reg_write, is_JMPJALLHI, ALU_src_B, is_halt, storeRT, is_JL_type, is_J_type, is_JR_type, use_rs, use_rt, m2r);
	//IF
	ADDER adder_pc_one(current_pc, 16'b1, PC_p_one);
	MUX16_4 next_PC_value(PC_p_one, EX_PC_p_one_p_imm, ALU_input_A, EX_imm16, PC_src, next_pc);
	program_counter PC(Clk, Reset_N, PCWrite, next_pc, current_pc);
	//IF
	IFandID IFnID(Clk, Reset_N, IFIDWrite, PC_p_one, IC_cached_data, current_pc, ID_PC_plus_one, ID_opcode, ID_rs, ID_rt, ID_rd, ID_funct, ID_imm_offset, ID_target, ID_current_pc, fetched);
	//ID
	GPR gpr(Clk, Reset_N, WB_WB_reg_write, ID_rs, ID_rt, WB_write_reg, WBresult, WB_is_real, read_data_1, read_data_2);
	imm_gen IMMGEN(ID_opcode, ID_imm_offset, ID_current_pc, ID_target, imm_16);
	MUX2_2 rdorrt(ID_rd, ID_rt, storeRT, RrtorRrd);
	MUX2_2 twoornot(RrtorRrd, 2'b10, is_JL_type, write_reg);
	//ID
	IDandEX IDnEX(Clk, Reset_N, IDEXWrite, is_halt, is_real, ID_PC_plus_one, read_data_1, read_data_2, ID_opcode, ID_funct, imm_16, ID_rs, ID_rt, write_reg, is_JMPJALLHI, ALU_src_B, readM, writeM, m2r, Reg_write, use_rs, use_rt, setZero, is_J_type, is_JR_type,
	EX_is_halt, is_empty, EX_is_real, EX_PC_plus_one, EX_read_rs, EX_read_rt, EX_opcode, EX_funct, EX_imm16, EX_rs, EX_rt, EX_write_reg, EX_EX_ALU_src_A, EX_EX_ALU_src_B, EX_MEM_readM, EX_MEM_writeM, EX_WB_memtoreg, EX_WB_reg_write, EX_use_rs, EX_use_rt, EX_is_J_type, EX_is_JR_type);
	// EXE
	ADDER adder_pc_imm(EX_PC_plus_one, EX_imm16, EX_PC_p_one_p_imm);
	MUX16_2 rsorzero(EX_read_rs, 16'b00, EX_EX_ALU_src_A, input_A);
	MUX16_2 rtorimm(EX_read_rt, EX_imm16, EX_EX_ALU_src_B, input_B);
	MUX16_4 ALU_input_A_MUX(input_A, WBresult, MEM_ALU_result, MEM_ALU_result, ForwardA, ALU_input_A);
	MUX16_2 ALU_input_B_MUX(WBresult, MEM_ALU_result, ForwardB, forwardB_input);
	MUX16_2 ALU_input_B_MUX_2(input_B, forwardB_input, forwardornot, ALU_input_B);
	MUX16_2 rtforSWD(EX_read_rt, forwardB_input, forwardornot, buffer_rt);
	ALU_control ALU_CONTROL(EX_funct, EX_opcode, ALU_func);
	ALU alu(ALU_input_A, ALU_input_B, ALU_func, alu_out);
	//EXE
	EXandMEM EXnMEM(Clk, Reset_N, EXMEMWrite, EX_is_real, EX_PC_plus_one, alu_out, buffer_rt, EX_write_reg, EX_MEM_readM, EX_MEM_writeM, EX_WB_memtoreg, EX_WB_reg_write,
	MEM_is_real, MEM_PC_plus_one, MEM_ALU_result, MEM_write_data, MEM_write_reg, read_DC, write_DC, MEM_WB_memtoreg, MEM_WB_reg_write);
	// MEM
	
	wire mem_is_real_g;
	assign mem_is_real_g = MEM_is_real && EXMEMWrite;

	//MEM
	MEMandWB MEMnWB(Clk, Reset_N, EXMEMWrite, mem_is_real_g, MEM_PC_plus_one, MEM_ALU_result, DC_cached_data, MEM_write_reg, MEM_WB_memtoreg, MEM_WB_reg_write,
	WB_is_real, WB_PC_plus_one, WB_ALU_result, WB_read_data, WB_write_reg, WB_WB_memtoreg, WB_WB_reg_write);
	// WB
	MUX16_4 WB_data(WB_ALU_result, WB_read_data, WB_PC_plus_one, WB_PC_plus_one, WB_WB_memtoreg, WBresult);

	wire read_dc_c;
	assign read_dc_c = MEM_is_real && read_DC;
	wire write_dc_c;
	assign write_dc_c = MEM_is_real && write_DC;


	//cache
	wire [`WORD_SIZE-1:0] address_cache;
	cache CACHE(Clk, Reset_N, read_IC, current_pc, data1, read_DC, write_dc_c, MEM_ALU_result, MEM_write_data, data2,
	IC_cached_data, readM1, address1, IC_missed, DC_cached_data, readM2, writeM2, address_cache, DC_missed
);

	// general
	reg [`WORD_SIZE-1:0] executed_instruction;

	reg temp;

	assign num_inst = executed_instruction;
	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			executed_instruction <= 0;
			temp <= 0;
		end
		if(!temp && (EX_is_real && IDEXWrite) && !is_empty) begin
			executed_instruction <= executed_instruction + 1;
		end
		if(EX_is_real && (EX_is_J_type || branchtaken)) begin
			temp <= 1;
		end
		else begin
			temp <= 0;
		end
	end
	
	// when halt
	reg cpu_halt;
	assign is_halted = cpu_halt;
	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			cpu_halt <= 0;
		end
		if(EX_is_halt && EX_is_real) begin
			cpu_halt <= 1;
		end
	end
	// WWD
	reg [`WORD_SIZE-1:0] output_p;
	assign output_port = output_p;
	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			output_p <= 0;
		end
		if(EX_is_real && (EX_opcode == `WWD_OP && EX_funct == `INST_FUNC_WWD)) begin
			output_p <= ALU_input_A;
		end
	end
	//DMA

	reg dma_interrupted;
	reg temp_d;

	always @(*) begin
		if(dma_interrupt) begin
			dma_interrupted <= 1;
		end
		if(Interrupt) begin
			dma_interrupted <= 0;
			access_mem <= 1;
			temp_d <= 0;
		end
		if(BR) BG <= 1;
		else BG <= 0;
	end

	always @(posedge(Clk)) begin
		if(!Reset_N) begin
			access_mem <= 1;
			dma_interrupted <= 0;
			temp_d <= 0;
			length <= 0;
		end
		if(!temp_d && dma_interrupted && !(readM2 || writeM2)) begin
			access_mem <= 0;
			length <= 1;
			temp_d <= 1;
		end
		if(temp_d) begin
			length <= 0;
		end
	end
	assign address2 = (!access_mem) ? 16'hc7 : address_cache;
endmodule
