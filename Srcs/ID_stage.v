`include "ctrl_encode_def.v"
module ID_stage(
    input clk,
    input rst,
    input [31:0] instr,
    input [31:0] PC,
    input [4:0] reg_sel,
    input [4:0] rd_addr_WB,
    input [31:0] wb_data_WB,
    input RegWrite_WB,
    // hazard detection inputs
    input [4:0] rd_addr_EX,
    input [4:0] rd_addr_MEM,
    input MemRead_EX,
    input RegWrite_EX,
    input RegWrite_MEM,
    input [6:0] opcode_EX,
    input [2:0] funct3_EX,
    input branch_taken_EX,
    input [6:0] opcode_ID,
    output [4:0] rs1_ID, rs2_ID, rd_ID,
    output [6:0] opcode_ID_out, funct7_ID,
    output [2:0] funct3_ID,
    output [31:0] rs1_data_ID, rs2_data_ID, imm_ID,
    output RegWrite_ID, MemWrite_ID, MemRead_ID,
    output [5:0] EXTOp_ID,
    output [4:0] ALUOp_ID,
    output [2:0] NPCOp_ID,
    output ALUSrc_ID,
    output [1:0] GPRSel_ID, WDSel_ID,
    output [2:0] DMType_ID,
    output Zero_ID, Sign_ID, Overflow_ID, Carry_ID,
    output [31:0] reg_data
);
    // 译码
    assign opcode_ID_out = instr[6:0];
    assign funct7_ID = instr[31:25];
    assign funct3_ID = instr[14:12];
    assign rs1_ID = instr[19:15];
    assign rs2_ID = instr[24:20];
    assign rd_ID = instr[11:7];
    // 控制
    ctrl ctrl_unit(
        .Op(opcode_ID_out), .Funct7(funct7_ID), .Funct3(funct3_ID), .Zero(Zero_ID),
        .RegWrite(RegWrite_ID), .MemWrite(MemWrite_ID), .MemRead(MemRead_ID),
        .EXTOp(EXTOp_ID), .ALUOp(ALUOp_ID), .NPCOp(NPCOp_ID),
        .ALUSrc(ALUSrc_ID), .GPRSel(GPRSel_ID), .WDSel(WDSel_ID), .DMType(DMType_ID)
    );
    // 立即数扩展
    wire [4:0] iimm_shamt_ID = instr[24:20];
    wire [11:0] iimm_ID = instr[31:20];
    wire [11:0] simm_ID = {instr[31:25], instr[11:7]};
    wire [11:0] bimm_ID = {instr[31], instr[7], instr[30:25], instr[11:8]};
    wire [19:0] uimm_ID = instr[31:12];
    wire [19:0] jimm_ID = {instr[31], instr[19:12], instr[20], instr[30:21]};
    EXT ext_unit(
        .iimm_shamt(iimm_shamt_ID), .iimm(iimm_ID), .simm(simm_ID), .bimm(bimm_ID),
        .uimm(uimm_ID), .jimm(jimm_ID), .EXTOp(EXTOp_ID), .immout(imm_ID)
    );
    // 寄存器堆
    RF rf_unit(
        .clk(clk), .rst(rst), .RFWr(RegWrite_WB),
        .A1(rs1_ID), .A2(rs2_ID), .A3(rd_addr_WB),
        .WD(wb_data_WB), .RD1(rs1_data_ID), .RD2(rs2_data_ID),
        .reg_sel(reg_sel), .reg_data(reg_data)
    );
    // 冒险检测
    HazardDetectionUnit hazard_detection_unit(
        .rs1_ID(rs1_ID), .rs2_ID(rs2_ID), .rd_EX(rd_addr_EX), .rd_MEM(rd_addr_MEM),
        .MemRead_EX(MemRead_EX), .RegWrite_EX(RegWrite_EX), .RegWrite_MEM(RegWrite_MEM),
        .opcode_EX(opcode_EX), .funct3_EX(funct3_EX), .branch_taken_EX(branch_taken_EX),
        .opcode_ID(opcode_ID), .stall_IF(), .flush_IF(), .flush_ID(), .flush_EX()
    );
endmodule 