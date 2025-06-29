// NPC control signal
`define NPC_PLUS4   3'b000
`define NPC_BRANCH  3'b001
`define NPC_JUMP    3'b010
`define NPC_JALR    3'b100

// ALU control signal
`define ALU_NOP     5'b00000 
`define ALU_ADD     5'b00001
`define ALU_SUB     5'b00010 
`define ALU_AND     5'b00011
`define ALU_OR      5'b00100
`define ALU_XOR     5'b00101
`define ALU_SLL     5'b00110
`define ALU_SRL     5'b00111
`define ALU_SRA     5'b01000
`define ALU_SLT     5'b01001
`define ALU_SLTU    5'b01010
`define ALU_LUI     5'b01011
`define ALU_AUIPC   5'b01100
`define ALU_BEQ     5'b01101
`define ALU_BNE     5'b01110
`define ALU_BLT     5'b01111
`define ALU_BGE     5'b10000
`define ALU_BLTU    5'b10001
`define ALU_BGEU    5'b10010

//EXT CTRL itype, stype, btype, utype, jtype
`define EXT_CTRL_ITYPE_SHAMT 6'b100000
`define EXT_CTRL_ITYPE	6'b010000
`define EXT_CTRL_STYPE	6'b001000
`define EXT_CTRL_BTYPE	6'b000100
`define EXT_CTRL_UTYPE	6'b000010
`define EXT_CTRL_JTYPE	6'b000001

//GPRSel = General Purpose Register Selection
`define GPRSel_RD 2'b00 //RD = Destination Register
`define GPRSel_RT 2'b01 //RT = Source Register
`define GPRSel_31 2'b10 //31 = Constant 31,x31,not used in riscv

//WDSel = Write Data Selection
`define WDSel_FromALU 2'b00
`define WDSel_FromMEM 2'b01
`define WDSel_FromPC 2'b10

// Memory access types
`define DM_WORD 3'b000
`define DM_HALFWORD 3'b001
`define DM_HALFWORD_UNSIGNED 3'b010
`define DM_BYTE 3'b011
`define DM_BYTE_UNSIGNED 3'b100

// RISC-V RV32I Opcodes
`define OPCODE_LUI     7'b0110111
`define OPCODE_AUIPC   7'b0010111
`define OPCODE_JAL     7'b1101111
`define OPCODE_JALR    7'b1100111
`define OPCODE_BRANCH  7'b1100011
`define OPCODE_LOAD    7'b0000011
`define OPCODE_STORE   7'b0100011
`define OPCODE_OP_IMM  7'b0010011
`define OPCODE_OP      7'b0110011

// RISC-V RV32I Funct3 codes
`define FUNCT3_BEQ     3'b000
`define FUNCT3_BNE     3'b001
`define FUNCT3_BLT     3'b100
`define FUNCT3_BGE     3'b101
`define FUNCT3_BLTU    3'b110
`define FUNCT3_BGEU    3'b111
`define FUNCT3_LB      3'b000
`define FUNCT3_LH      3'b001
`define FUNCT3_LW      3'b010
`define FUNCT3_LBU     3'b100
`define FUNCT3_LHU     3'b101
`define FUNCT3_SB      3'b000
`define FUNCT3_SH      3'b001
`define FUNCT3_SW      3'b010
`define FUNCT3_ADDI    3'b000
`define FUNCT3_SLTI    3'b010
`define FUNCT3_SLTIU   3'b011
`define FUNCT3_XORI    3'b100
`define FUNCT3_ORI     3'b110
`define FUNCT3_ANDI    3'b111
`define FUNCT3_SLLI    3'b001
`define FUNCT3_SRLI    3'b101
`define FUNCT3_SRAI    3'b101
`define FUNCT3_ADD     3'b000
`define FUNCT3_SUB     3'b000
`define FUNCT3_SLL     3'b001
`define FUNCT3_SLT     3'b010
`define FUNCT3_SLTU    3'b011
`define FUNCT3_XOR     3'b100
`define FUNCT3_SRL     3'b101
`define FUNCT3_SRA     3'b101
`define FUNCT3_OR      3'b110
`define FUNCT3_AND     3'b111

// RISC-V RV32I Funct7 codes
`define FUNCT7_ADD     7'b0000000
`define FUNCT7_SUB     7'b0100000
`define FUNCT7_SRL     7'b0000000
`define FUNCT7_SRA     7'b0100000

