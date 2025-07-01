`include "ctrl_encode_def.v"
module EXE_stage(
    input clk,
    input rst,
    input [31:0] PC_EX,
    input [31:0] rs1_data_EX,
    input [31:0] rs2_data_EX,
    input [31:0] imm_EX,
    input [4:0] rs1_addr_EX,
    input [4:0] rs2_addr_EX,
    input [4:0] rd_addr_EX,
    input [6:0] opcode_EX,
    input [2:0] funct3_EX,
    input [6:0] funct7_EX,
    input RegWrite_EX,
    input MemWrite_EX,
    input MemRead_EX,
    input [5:0] EXTOp_EX,
    input [4:0] ALUOp_EX,
    input [2:0] NPCOp_EX,
    input ALUSrc_EX,
    input [1:0] GPRSel_EX,
    input [1:0] WDSel_EX,
    input [2:0] DMType_EX,
    input [1:0] forward_rs1,
    input [1:0] forward_rs2,
    input [31:0] alu_result_MEM,
    input [31:0] wb_data_WB,
    output [31:0] alu_result_EX,
    output [31:0] rs2_data_forwarded_EX,
    output Zero_EX, Sign_EX, Overflow_EX, Carry_EX,
    output branch_taken_EX
);
    wire [31:0] rs1_data_forwarded_EX;
    assign rs1_data_forwarded_EX = (forward_rs1 == 2'b01) ? alu_result_MEM :
                                   (forward_rs1 == 2'b10) ? wb_data_WB :
                                   rs1_data_EX;
    assign rs2_data_forwarded_EX = (forward_rs2 == 2'b01) ? alu_result_MEM :
                                   (forward_rs2 == 2'b10) ? wb_data_WB :
                                   rs2_data_EX;
    wire [31:0] alu_B_EX = ALUSrc_EX ? imm_EX : rs2_data_forwarded_EX;
    alu alu_unit(
        .A(rs1_data_forwarded_EX),
        .B(alu_B_EX),
        .ALUOp(ALUOp_EX),
        .C(alu_result_EX),
        .Zero(Zero_EX),
        .PC(PC_EX),
        .Sign(Sign_EX),
        .Overflow(Overflow_EX),
        .Carry(Carry_EX)
    );
    assign branch_taken_EX = (opcode_EX == `OPCODE_BRANCH) && (
        (funct3_EX == `FUNCT3_BEQ && Zero_EX) ||
        (funct3_EX == `FUNCT3_BNE && !Zero_EX) ||
        (funct3_EX == `FUNCT3_BLT && (Sign_EX ^ Overflow_EX)) ||
        (funct3_EX == `FUNCT3_BGE && !(Sign_EX ^ Overflow_EX)) ||
        (funct3_EX == `FUNCT3_BLTU && Carry_EX) ||
        (funct3_EX == `FUNCT3_BGEU && !Carry_EX)
    );
endmodule 