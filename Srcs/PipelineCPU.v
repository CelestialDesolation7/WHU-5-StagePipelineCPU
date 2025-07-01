`include "ctrl_encode_def.v"

module PipelineCPU(
    input clk,
    input rst,
    input [31:0] instr_in,
    input [31:0] Data_in,
    output mem_w,
    output [31:0] PC_out,
    output [31:0] Addr_out,
    output [31:0] Data_out,
    output MemWrite_MEM,
    output [2:0] DMType_MEM,
    output [31:0] alu_result_MEM,
    output [31:0] rs2_data_MEM,
    input [4:0] reg_sel,
    output [31:0] reg_data
);
    // IF阶段
    wire [31:0] PC_IF, NPC_IF, instr_IF;
    wire stall_IF;
    wire [2:0] npc_op_sel;
    wire [31:0] npc_imm_sel, npc_pc_sel;
    // npc_op_sel等信号可由ID/EXE阶段输出
    IF_stage if_stage(
        .clk(clk), .rst(rst), .stall(stall_IF),
        .instr_in(instr_in),
        .npc_op(npc_op_sel), .npc_imm(npc_imm_sel), .npc_pc(npc_pc_sel),
        .PC(PC_IF), .instr(instr_IF), .NPC(NPC_IF)
    );

    // IF/ID流水线寄存器
    wire [31:0] PC_IF_ID, instr_IF_ID;
    
    IF_ID_Reg if_id_reg(
        .clk(clk), .rst(rst), .flush(1'b0), .stall(stall_IF),
        .PC_in(PC_IF), .instr_in(instr_IF),
        .PC_out(PC_IF_ID), .instr_out(instr_IF_ID)
    );

    // ID阶段
    wire [4:0] rs1_ID, rs2_ID, rd_ID;
    wire [6:0] opcode_ID, funct7_ID;
    wire [2:0] funct3_ID;
    wire [31:0] rs1_data_ID, rs2_data_ID, imm_ID;
    wire RegWrite_ID, MemWrite_ID, MemRead_ID;
    wire [5:0] EXTOp_ID;
    wire [4:0] ALUOp_ID;
    wire [2:0] NPCOp_ID;
    wire ALUSrc_ID;
    wire [1:0] GPRSel_ID, WDSel_ID;
    wire [2:0] DMType_ID;
    wire Zero_ID, Sign_ID, Overflow_ID, Carry_ID;

    ID_stage id_stage(
        .clk(clk), 
        .rst(rst),
        .instr(instr_IF_ID), 
        .PC(PC_IF_ID),
        .reg_sel(reg_sel), 
        .rd_addr_WB(rd_addr_WB), 
        .wb_data_WB(wb_data_WB), 
        .RegWrite_WB(RegWrite_WB),
        .rd_addr_EX(rd_addr_EX), 
        .rd_addr_MEM(rd_addr_MEM),
        .MemRead_EX(MemRead_EX), 
        .RegWrite_EX(RegWrite_EX), 
        .RegWrite_MEM(RegWrite_MEM),
        .opcode_EX(opcode_EX), 
        .funct3_EX(funct3_EX), 
        .branch_taken_EX(branch_taken_EX), 
        .opcode_ID(opcode_ID),
        .rs1_ID(rs1_ID), 
        .rs2_ID(rs2_ID), 
        .rd_ID(rd_ID),
        .opcode_ID_out(opcode_ID), 
        .funct7_ID(funct7_ID), 
        .funct3_ID(funct3_ID),
        .rs1_data_ID(rs1_data_ID), 
        .rs2_data_ID(rs2_data_ID), 
        .imm_ID(imm_ID),
        .RegWrite_ID(RegWrite_ID), 
        .MemWrite_ID(MemWrite_ID), 
        .MemRead_ID(MemRead_ID),
        .EXTOp_ID(EXTOp_ID), 
        .ALUOp_ID(ALUOp_ID), 
        .NPCOp_ID(NPCOp_ID), 
        .ALUSrc_ID(ALUSrc_ID),
        .GPRSel_ID(GPRSel_ID), 
        .WDSel_ID(WDSel_ID), 
        .DMType_ID(DMType_ID),
        .Zero_ID(Zero_ID), 
        .Sign_ID(Sign_ID), 
        .Overflow_ID(Overflow_ID), 
        .Carry_ID(Carry_ID),
        .reg_data(reg_data)
    );

    // ID/EX流水线寄存器
    wire [31:0] PC_ID_EX, rs1_data_ID_EX, rs2_data_ID_EX, imm_ID_EX;
    wire [4:0] rs1_addr_ID_EX, rs2_addr_ID_EX, rd_addr_ID_EX;
    wire [6:0] opcode_ID_EX, funct7_ID_EX;
    wire [2:0] funct3_ID_EX;
    wire RegWrite_ID_EX, MemWrite_ID_EX, MemRead_ID_EX;
    wire [5:0] EXTOp_ID_EX;
    wire [4:0] ALUOp_ID_EX;
    wire [2:0] NPCOp_ID_EX;
    wire ALUSrc_ID_EX;
    wire [1:0] GPRSel_ID_EX, WDSel_ID_EX;
    wire [2:0] DMType_ID_EX;

    ID_EX_Reg id_ex_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(1'b0),
        .PC_in(PC_IF_ID), 
        .rs1_data_in(rs1_data_ID), 
        .rs2_data_in(rs2_data_ID), 
        .imm_in(imm_ID),
        .rs1_addr_in(rs1_ID), 
        .rs2_addr_in(rs2_ID), 
        .rd_addr_in(rd_ID),
        .opcode_in(opcode_ID), 
        .funct3_in(funct3_ID), 
        .funct7_in(funct7_ID),
        .RegWrite_in(RegWrite_ID), 
        .MemWrite_in(MemWrite_ID), 
        .MemRead_in(MemRead_ID),
        .EXTOp_in(EXTOp_ID), 
        .ALUOp_in(ALUOp_ID), 
        .NPCOp_in(NPCOp_ID),
        .ALUSrc_in(ALUSrc_ID), 
        .GPRSel_in(GPRSel_ID), 
        .WDSel_in(WDSel_ID), 
        .DMType_in(DMType_ID),
        .PC_out(PC_ID_EX), 
        .rs1_data_out(rs1_data_ID_EX), 
        .rs2_data_out(rs2_data_ID_EX), 
        .imm_out(imm_ID_EX),
        .rs1_addr_out(rs1_addr_ID_EX), 
        .rs2_addr_out(rs2_addr_ID_EX), 
        .rd_addr_out(rd_addr_ID_EX),
        .opcode_out(opcode_ID_EX), 
        .funct3_out(funct3_ID_EX), 
        .funct7_out(funct7_ID_EX),
        .RegWrite_out(RegWrite_ID_EX), 
        .MemWrite_out(MemWrite_ID_EX), 
        .MemRead_out(MemRead_ID_EX),
        .EXTOp_out(EXTOp_ID_EX), 
        .ALUOp_out(ALUOp_ID_EX), 
        .NPCOp_out(NPCOp_ID_EX),
        .ALUSrc_out(ALUSrc_ID_EX), 
        .GPRSel_out(GPRSel_ID_EX), 
        .WDSel_out(WDSel_ID_EX), 
        .DMType_out(DMType_ID_EX)
    );

    // EXE阶段
    wire [31:0] alu_result_EX, rs2_data_forwarded_EX;
    wire Zero_EX, Sign_EX, Overflow_EX, Carry_EX;
    wire branch_taken_EX;

    EXE_stage exe_stage(
        .clk(clk), 
        .rst(rst),
        .PC_EX(PC_ID_EX), 
        .rs1_data_EX(rs1_data_ID_EX), 
        .rs2_data_EX(rs2_data_ID_EX), 
        .imm_EX(imm_ID_EX),
        .rs1_addr_EX(rs1_addr_ID_EX), 
        .rs2_addr_EX(rs2_addr_ID_EX), 
        .rd_addr_EX(rd_addr_ID_EX),
        .opcode_EX(opcode_ID_EX), 
        .funct3_EX(funct3_ID_EX), 
        .funct7_EX(funct7_ID_EX),
        .RegWrite_EX(RegWrite_ID_EX), 
        .MemWrite_EX(MemWrite_ID_EX), 
        .MemRead_EX(MemRead_ID_EX),
        .EXTOp_EX(EXTOp_ID_EX), 
        .ALUOp_EX(ALUOp_ID_EX), 
        .NPCOp_EX(NPCOp_ID_EX), 
        .ALUSrc_EX(ALUSrc_ID_EX),
        .GPRSel_EX(GPRSel_ID_EX), 
        .WDSel_EX(WDSel_ID_EX), 
        .DMType_EX(DMType_ID_EX),
        .forward_rs1(forward_rs1), 
        .forward_rs2(forward_rs2),
        .alu_result_MEM(alu_result_MEM), 
        .wb_data_WB(wb_data_WB),
        .alu_result_EX(alu_result_EX), 
        .rs2_data_forwarded_EX(rs2_data_forwarded_EX),
        .Zero_EX(Zero_EX), 
        .Sign_EX(Sign_EX), 
        .Overflow_EX(Overflow_EX), 
        .Carry_EX(Carry_EX),
        .branch_taken_EX(branch_taken_EX)
    );

    // EX/MEM流水线寄存器
    wire [31:0] alu_result_EX_MEM, rs2_data_EX_MEM;
    wire [4:0] rd_addr_EX_MEM;
    wire RegWrite_EX_MEM, MemWrite_EX_MEM, MemRead_EX_MEM;
    wire [1:0] WDSel_EX_MEM;
    wire [2:0] DMType_EX_MEM;
    wire [31:0] PC_EX_MEM;

    EX_MEM_Reg ex_mem_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(1'b0),
        .alu_result_in(alu_result_EX), 
        .rs2_data_in(rs2_data_forwarded_EX), 
        .rd_addr_in(rd_addr_ID_EX),
        .RegWrite_in(RegWrite_ID_EX), 
        .MemWrite_in(MemWrite_ID_EX), 
        .MemRead_in(MemRead_ID_EX),
        .WDSel_in(WDSel_ID_EX), 
        .DMType_in(DMType_ID_EX), 
        .PC_in(PC_ID_EX),
        .alu_result_out(alu_result_EX_MEM), 
        .rs2_data_out(rs2_data_EX_MEM), 
        .rd_addr_out(rd_addr_EX_MEM),
        .RegWrite_out(RegWrite_EX_MEM), 
        .MemWrite_out(MemWrite_EX_MEM), 
        .MemRead_out(MemRead_EX_MEM),
        .WDSel_out(WDSel_EX_MEM), 
        .DMType_out(DMType_EX_MEM), 
        .PC_out(PC_EX_MEM)
    );

    // MEM阶段
    wire [31:0] mem_data_MEM;
    MEM_stage mem_stage(
        .clk(clk), .rst(rst),
        .alu_result_MEM(alu_result_EX_MEM), .rs2_data_MEM(rs2_data_EX_MEM), .rd_addr_MEM(rd_addr_EX_MEM),
        .RegWrite_MEM(RegWrite_EX_MEM), .MemWrite_MEM(MemWrite_EX_MEM), .MemRead_MEM(MemRead_EX_MEM),
        .WDSel_MEM(WDSel_EX_MEM), .DMType_MEM(DMType_EX_MEM), .Data_in(Data_in),
        .mem_data_MEM(mem_data_MEM)
    );

    // MEM/WB流水线寄存器
    wire [31:0] alu_result_MEM_WB, mem_data_MEM_WB;
    wire [4:0] rd_addr_MEM_WB;
    wire RegWrite_MEM_WB;
    wire [1:0] WDSel_MEM_WB;
    wire [31:0] PC_MEM_WB;

    MEM_WB_Reg mem_wb_reg(
        .clk(clk), 
        .rst(rst),
        .alu_result_in(alu_result_EX_MEM), 
        .mem_data_in(mem_data_MEM), 
        .rd_addr_in(rd_addr_EX_MEM),
        .RegWrite_in(RegWrite_EX_MEM), 
        .WDSel_in(WDSel_EX_MEM), 
        .PC_in(PC_EX_MEM),
        .alu_result_out(alu_result_MEM_WB), 
        .mem_data_out(mem_data_MEM_WB), 
        .rd_addr_out(rd_addr_MEM_WB),
        .RegWrite_out(RegWrite_EX_MEM), 
        .WDSel_out(WDSel_EX_MEM), 
        .PC_out(PC_EX_MEM)
    );

    // WB阶段
    wire [31:0] alu_result_WB = alu_result_MEM_WB;
    wire [31:0] mem_data_WB = mem_data_MEM_WB;
    wire [4:0] rd_addr_WB = rd_addr_MEM_WB;
    wire RegWrite_WB = RegWrite_EX_MEM;
    wire [1:0] WDSel_WB = WDSel_EX_MEM;
    wire [31:0] PC_WB = PC_EX_MEM;
    wire [31:0] wb_data_WB;
    assign wb_data_WB = (WDSel_WB == `WDSel_FromALU) ? alu_result_WB :
                        (WDSel_WB == `WDSel_FromMEM) ? mem_data_WB :
                        (WDSel_WB == `WDSel_FromPC) ? (PC_WB + 4) : alu_result_WB;
    // 输出
    assign PC_out = PC_IF;
    assign Addr_out = alu_result_EX_MEM;
    assign Data_out = rs2_data_EX_MEM;
    assign mem_w = MemWrite_EX_MEM;
    assign MemWrite_MEM = MemWrite_EX_MEM;
    assign DMType_MEM = DMType_EX_MEM;
    assign alu_result_MEM = alu_result_EX_MEM;
    assign rs2_data_MEM = rs2_data_EX_MEM;
endmodule