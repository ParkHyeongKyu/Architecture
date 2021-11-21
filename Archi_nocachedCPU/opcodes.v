		  
// Opcode
`define	ALU_OP	4'd15
`define	ADI_OP	4'd4
`define	ORI_OP	4'd5
`define	LHI_OP	4'd6
`define	LWD_OP	4'd7   		  
`define	SWD_OP	4'd8  
`define	BNE_OP	4'd0
`define	BEQ_OP	4'd1
`define BGZ_OP	4'd2
`define BLZ_OP	4'd3
`define	JMP_OP	4'd9
`define JAL_OP	4'd10
`define	JPR_OP	4'd15
`define	JRL_OP	4'd15

// ALU Function Codes
`define	ALU_FUNC_ADD	3'b000
`define	ALU_FUNC_SUB	3'b001				 
`define	ALU_FUNC_AND	3'b010
`define	ALU_FUNC_ORR	3'b011								    
`define	ALU_FUNC_NOT	3'b100
`define	ALU_FUNC_TCP	3'b101
`define	ALU_FUNC_SHL	3'b110
`define	ALU_FUNC_SHR	3'b111	

// ALU instruction function codes
`define INST_FUNC_ADD 6'd0
`define INST_FUNC_SUB 6'd1
`define INST_FUNC_AND 6'd2
`define INST_FUNC_ORR 6'd3
`define INST_FUNC_NOT 6'd4
`define INST_FUNC_TCP 6'd5
`define INST_FUNC_SHL 6'd6
`define INST_FUNC_SHR 6'd7
`define INST_FUNC_JPR 6'd25
`define INST_FUNC_JRL 6'd26
`define INST_FUNC_RWD 6'd27
`define INST_FUNC_WWD 6'd28
`define INST_FUNC_HLT 6'd29
`define INST_FUNC_ENI 6'd30
`define INST_FUNC_DSI 6'd31
	
`define	NUM_REGS	4
`define WORD_SIZE 16