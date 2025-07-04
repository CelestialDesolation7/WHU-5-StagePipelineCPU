`include "ctrl_encode_def.v"

module ctrl(input  [6:0] Op,       // opcode
            input  [6:0] Funct7,    // funct7
            input  [2:0] Funct3,    // funct3
            
            output       RegWrite, // control signal for register write
            output       MemWrite, // control signal for memory write
            output       MemRead,  // control signal for memory read
            output [5:0] EXTOp,    // control signal to signed extension
            output [4:0] ALUOp,    // ALU opertion
            output       ALUSrc,   // ALU source for A
            output [2:0] DMType,   // Data memory access type
            output [1:0] WDSel    // (register) write data selection
            );
            
   // ------------------------------------------------------------
   // Instruction type encoding begins
   // Instruction type detection
   wire lui     = (Op == `OPCODE_LUI);
   wire auipc    = (Op == `OPCODE_AUIPC);
   wire jal      = (Op == `OPCODE_JAL);
   wire jalr     = (Op == `OPCODE_JALR);
   wire branch   = (Op == `OPCODE_BRANCH);
   wire load     = (Op == `OPCODE_LOAD);
   wire store    = (Op == `OPCODE_STORE);
   wire op_imm   = (Op == `OPCODE_OP_IMM);
   wire op       = (Op == `OPCODE_OP);
   
   // Branch instructions
   wire beq      = branch & (Funct3 == `FUNCT3_BEQ);
   wire bne      = branch & (Funct3 == `FUNCT3_BNE);
   wire blt      = branch & (Funct3 == `FUNCT3_BLT);
   wire bge      = branch & (Funct3 == `FUNCT3_BGE);
   wire bltu     = branch & (Funct3 == `FUNCT3_BLTU);
   wire bgeu     = branch & (Funct3 == `FUNCT3_BGEU);
   
   // Load instructions
   wire lb       = load & (Funct3 == `FUNCT3_LB);
   wire lh       = load & (Funct3 == `FUNCT3_LH);
   wire lw       = load & (Funct3 == `FUNCT3_LW);
   wire lbu      = load & (Funct3 == `FUNCT3_LBU);
   wire lhu      = load & (Funct3 == `FUNCT3_LHU);
   
   // Store instructions
   wire sb       = store & (Funct3 == `FUNCT3_SB);
   wire sh       = store & (Funct3 == `FUNCT3_SH);
   wire sw       = store & (Funct3 == `FUNCT3_SW);
   
   // Immediate arithmetic/logical instructions
   wire addi     = op_imm & (Funct3 == `FUNCT3_ADDI);
   wire slti     = op_imm & (Funct3 == `FUNCT3_SLTI);
   wire sltiu    = op_imm & (Funct3 == `FUNCT3_SLTIU);
   wire xori     = op_imm & (Funct3 == `FUNCT3_XORI);
   wire ori      = op_imm & (Funct3 == `FUNCT3_ORI);
   wire andi     = op_imm & (Funct3 == `FUNCT3_ANDI);
   wire slli     = op_imm & (Funct3 == `FUNCT3_SLLI);
   wire srli     = op_imm & (Funct3 == `FUNCT3_SRLI) & (Funct7 == `FUNCT7_SRL);
   wire srai     = op_imm & (Funct3 == `FUNCT3_SRAI) & (Funct7 == `FUNCT7_SRA);
   
   // Register arithmetic/logical instructions
   wire add      = op & (Funct3 == `FUNCT3_ADD) & (Funct7 == `FUNCT7_ADD);
   wire sub      = op & (Funct3 == `FUNCT3_SUB) & (Funct7 == `FUNCT7_SUB);
   wire sll      = op & (Funct3 == `FUNCT3_SLL);
   wire slt      = op & (Funct3 == `FUNCT3_SLT);
   wire sltu     = op & (Funct3 == `FUNCT3_SLTU);
   wire xor_op   = op & (Funct3 == `FUNCT3_XOR);
   wire srl      = op & (Funct3 == `FUNCT3_SRL) & (Funct7 == `FUNCT7_SRL);
   wire sra      = op & (Funct3 == `FUNCT3_SRA) & (Funct7 == `FUNCT7_SRA);
   wire or_op    = op & (Funct3 == `FUNCT3_OR);
   wire and_op   = op & (Funct3 == `FUNCT3_AND);
   
   // instruction type encoding ends
   // ------------------------------------------------------------
   

   // ------------------------------------------------------------
   // Control signals generation begins
   // Generate control signals
   assign RegWrite = lui | auipc | jal | jalr | load | addi | slti | sltiu | xori | ori | andi | slli | srli | srai | add | sub | sll | slt | sltu | xor_op | srl | sra | or_op | and_op;
   assign MemWrite = store;
   assign MemRead = load;
   assign ALUSrc = auipc | jal | jalr | load | store | addi | slti | sltiu | xori | ori | andi | slli | srli | srai;
   
   // Signed extension control
   assign EXTOp[5] = slli | srli | srai;  // ITYPE_SHAMT
   assign EXTOp[4] = addi | slti | sltiu | xori | ori | andi | jalr;  // ITYPE
   assign EXTOp[3] = store;  // STYPE
   assign EXTOp[2] = branch;  // BTYPE
   assign EXTOp[1] = lui | auipc;  // UTYPE
   assign EXTOp[0] = jal;  // JTYPE
   
   // Write data selection
   assign WDSel[1] = jal | jalr;
   assign WDSel[0] = load;
   
   
   // ALU operation encoding
   assign ALUOp = (beq) ? `ALU_BEQ :
                (bne) ? `ALU_BNE :
                (blt) ? `ALU_BLT :
                (bge) ? `ALU_BGE :
                (bltu) ? `ALU_BLTU :
                (bgeu) ? `ALU_BGEU :
                (lui) ? `ALU_LUI :
                (auipc) ? `ALU_AUIPC :
                (addi | add | load | store | jalr) ? `ALU_ADD :
                (sub) ? `ALU_SUB :
                (slti | slt) ? `ALU_SLT :
                (sltiu | sltu) ? `ALU_SLTU :
                (xori | xor_op) ? `ALU_XOR :
                (ori | or_op) ? `ALU_OR :
                (andi | and_op) ? `ALU_AND :
                (slli | sll) ? `ALU_SLL :
                (srli | srl) ? `ALU_SRL :
                (srai | sra) ? `ALU_SRA : `ALU_NOP;

   // Data memory access type
   assign DMType[2] = lbu | lhu;
   assign DMType[1] = lh | lhu | sh;
   assign DMType[0] = lb | lbu | sb;

   // Control signals generation ends
   // ------------------------------------------------------------

endmodule