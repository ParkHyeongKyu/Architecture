// Opcode
`define	ALU_OP	4'b1111 //15
`define	ADI_OP	4'b0100 // 4
`define	ORI_OP	4'b0101 // 5
`define	LHI_OP	4'b0110 // 6
`define RWD_OP	4'b1111 //15
`define WWD_OP	4'b1111 //15
`define	LWD_OP	4'b0111 // 7
`define	SWD_OP	4'b1000 // 8
`define	BNE_OP	4'b0000 // 0
`define	BEQ_OP	4'b0001 // 1
`define BGZ_OP	4'b0010 // 2
`define BLZ_OP	4'b0011 // 3
`define	JMP_OP	4'b1001 // 9
`define JAL_OP	4'b1010 //10
`define	JPR_OP	4'b1111 //15
`define	JRL_OP	4'b1111 //15
`define HLT_OP	4'b1111 //15
`define ENI_OP	4'b1111 //15
`define DSI_OP	4'b1111 //15


// Function Codes
// ALU
`define INST_FUNC_ADD 6'b000000
`define INST_FUNC_SUB 6'b000001
`define INST_FUNC_AND 6'b000010
`define INST_FUNC_ORR 6'b000011
`define INST_FUNC_NOT 6'b000100
`define INST_FUNC_TCP 6'b000101
`define INST_FUNC_SHL 6'b000110
`define INST_FUNC_SHR 6'b000111
// RWD
`define INST_FUNC_RWD 6'b011011 //27
// WWD
`define INST_FUNC_WWD 6'b011100 //28
// JPR
`define INST_FUNC_JPR 6'b011001 //25
// JRL
`define INST_FUNC_JRL 6'b011010 //26
// HLT
`define INST_FUNC_HLT 6'b011101 //29
// ENI
`define INST_FUNC_ENI 6'b011110 //30
// DSI
`define INST_FUNC_DSI 6'b011111 //31


// ALU Function Codes
`define	ALU_FUNC_ADD	3'b000 //0
`define	ALU_FUNC_SUB	3'b001 //1
`define	ALU_FUNC_AND	3'b010 //2
`define	ALU_FUNC_ORR	3'b011 //3
`define	ALU_FUNC_NOT	3'b100 //4
`define	ALU_FUNC_TCP	3'b101 //5
`define	ALU_FUNC_SHL	3'b110 //6
`define	ALU_FUNC_SHR	3'b111 //7


// const
`define	NUM_REGS	4
`define WORD_SIZE	16


// state
`define ST_SIZE	4
`define ST_IF1	4'b0000
`define ST_IF2	4'b0001
`define ST_IF3	4'b0010
`define ST_IF4	4'b0011
`define ST_ID	4'b0100
`define ST_EX1	4'b0101
`define ST_EX2	4'b0110
`define ST_MEM1	4'b0111
`define ST_MEM2	4'b1000
`define ST_MEM3	4'b1001
`define ST_MEM4	4'b1010
`define ST_WB	4'b1011
`define ST_HLT	4'b1100

