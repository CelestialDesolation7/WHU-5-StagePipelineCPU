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
    input [4:0] reg_sel,
    output [31:0] reg_data
);

    // ----------------------------------------------------------------
    // Control signals wire preparation begins
    // IF stage signals
    wire [31:0] PC_IF, NPC_IF;
    wire [31:0] instr_IF;
    
    // IF/ID pipeline register signals
    wire [31:0] PC_IF_ID, instr_IF_ID;
    
    // ID stage signals
    wire [31:0] PC_ID = PC_IF_ID;
    wire [31:0] instr_ID = instr_IF_ID;
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
    
    // ID/EX pipeline register signals
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
    
    // EX stage signals
    wire [31:0] PC_EX = PC_ID_EX;
    wire [31:0] rs1_data_EX = rs1_data_ID_EX;
    wire [31:0] rs2_data_EX = rs2_data_ID_EX;
    wire [31:0] imm_EX = imm_ID_EX;
    wire [4:0] rs1_addr_EX = rs1_addr_ID_EX;
    wire [4:0] rs2_addr_EX = rs2_addr_ID_EX;
    wire [4:0] rd_addr_EX = rd_addr_ID_EX;
    wire [6:0] opcode_EX = opcode_ID_EX;
    wire [2:0] funct3_EX = funct3_ID_EX;
    wire [6:0] funct7_EX = funct7_ID_EX;
    wire RegWrite_EX = RegWrite_ID_EX;
    wire MemWrite_EX = MemWrite_ID_EX;
    wire MemRead_EX = MemRead_ID_EX;
    wire [5:0] EXTOp_EX = EXTOp_ID_EX;
    wire [4:0] ALUOp_EX = ALUOp_ID_EX;
    wire [2:0] NPCOp_EX = NPCOp_ID_EX;
    wire ALUSrc_EX = ALUSrc_ID_EX;
    wire [1:0] GPRSel_EX = GPRSel_ID_EX;
    wire [1:0] WDSel_EX = WDSel_ID_EX;
    wire [2:0] DMType_EX = DMType_ID_EX;
    wire [31:0] alu_result_EX, alu_B_EX;
    wire Zero_EX, Sign_EX, Overflow_EX, Carry_EX;
    
    // EX/MEM pipeline register signals
    wire [31:0] alu_result_EX_MEM, rs2_data_EX_MEM;
    wire [4:0] rd_addr_EX_MEM;
    wire RegWrite_EX_MEM, MemWrite_EX_MEM, MemRead_EX_MEM;
    wire [1:0] WDSel_EX_MEM;
    wire [2:0] DMType_EX_MEM;
    wire [31:0] PC_EX_MEM;
    
    // MEM stage signals
    wire [31:0] alu_result_MEM = alu_result_EX_MEM;
    wire [31:0] rs2_data_MEM = rs2_data_EX_MEM;
    wire [4:0] rd_addr_MEM = rd_addr_EX_MEM;
    wire RegWrite_MEM = RegWrite_EX_MEM;
    wire MemWrite_MEM = MemWrite_EX_MEM;
    wire MemRead_MEM = MemRead_EX_MEM;
    wire [1:0] WDSel_MEM = WDSel_EX_MEM;
    wire [2:0] DMType_MEM = DMType_EX_MEM;
    wire [31:0] PC_MEM = PC_EX_MEM;
    wire [31:0] mem_data_MEM;
    
    // MEM/WB pipeline register signals
    wire [31:0] alu_result_MEM_WB, mem_data_MEM_WB;
    wire [4:0] rd_addr_MEM_WB;
    wire RegWrite_MEM_WB;
    wire [1:0] WDSel_MEM_WB;
    wire [31:0] PC_MEM_WB;
    
    // WB stage signals
    wire [31:0] alu_result_WB = alu_result_MEM_WB;
    wire [31:0] mem_data_WB = mem_data_MEM_WB;
    wire [4:0] rd_addr_WB = rd_addr_MEM_WB;
    wire RegWrite_WB = RegWrite_MEM_WB;
    wire [1:0] WDSel_WB = WDSel_MEM_WB;
    wire [31:0] PC_WB = PC_MEM_WB;
    wire [31:0] wb_data_WB;
    
    // Hazard detection and forwarding signals
    wire stall_IF, flush_IF, flush_ID, flush_EX;
    wire [1:0] forward_rs1, forward_rs2;
    wire branch_taken_EX;  // 新增：EX阶段分支是否被采取的信号
    // Control signals wire preparation ends
    // ----------------------------------------------------------------
    
    
    // ----------------------------------------------------------------
    // Instruction Decode wire preparation begins
    // Instruction decode
    assign opcode_ID = instr_ID[6:0];
    assign funct7_ID = instr_ID[31:25];
    assign funct3_ID = instr_ID[14:12];
    assign rs1_ID = instr_ID[19:15];
    assign rs2_ID = instr_ID[24:20];
    assign rd_ID = instr_ID[11:7];
    
    // Immediate extraction
    wire [4:0] iimm_shamt_ID;
    wire [11:0] iimm_ID, simm_ID, bimm_ID;
    wire [19:0] uimm_ID, jimm_ID;
    assign iimm_shamt_ID = instr_ID[24:20];
    assign iimm_ID = instr_ID[31:20];
    assign simm_ID = {instr_ID[31:25], instr_ID[11:7]};
    assign bimm_ID = {instr_ID[31], instr_ID[7], instr_ID[30:25], instr_ID[11:8]};
    assign uimm_ID = instr_ID[31:12];
    assign jimm_ID = {instr_ID[31], instr_ID[19:12], instr_ID[20], instr_ID[30:21]};
    // Instruction Decode wire preparation ends
    // ----------------------------------------------------------------
    

    // ----------------------------------------------------------------
    // ALU B operand selection
    assign alu_B_EX = ALUSrc_EX ? imm_EX : rs2_data_EX;
    
    // Write back data selection
    assign wb_data_WB = (WDSel_WB == `WDSel_FromALU) ? alu_result_WB :
                        (WDSel_WB == `WDSel_FromMEM) ? mem_data_WB :
                        (WDSel_WB == `WDSel_FromPC) ? (PC_WB + 4) : alu_result_WB;
    
    // EX阶段的分支判断逻辑 - 基于ALU标志位
    wire branch_taken_EX;
    assign branch_taken_EX = (opcode_EX == `OPCODE_BRANCH) && (
        (funct3_EX == `FUNCT3_BEQ && Zero_EX) ||                    // beq: 相等时跳转
        (funct3_EX == `FUNCT3_BNE && !Zero_EX) ||                   // bne: 不等时跳转
        (funct3_EX == `FUNCT3_BLT && (Sign_EX ^ Overflow_EX)) ||    // blt: 有符号小于时跳转
        (funct3_EX == `FUNCT3_BGE && !(Sign_EX ^ Overflow_EX)) ||   // bge: 有符号大于等于时跳转
        (funct3_EX == `FUNCT3_BLTU && Carry_EX) ||                  // bltu: 无符号小于时跳转
        (funct3_EX == `FUNCT3_BGEU && !Carry_EX)                    // bgeu: 无符号大于等于时跳转
    );
    
    // Output assignments
    assign PC_out = PC_IF;
    assign Addr_out = alu_result_MEM;
    assign Data_out = rs2_data_MEM;
    assign mem_w = MemWrite_MEM;
    // ----------------------------------------------------------------
    


    // ----------------------------------------------------------------
    // IF Stage Hardware instantiation begins
    // PC, offer reset functionality
    PC pc_unit(.clk(clk), .rst(rst), .NPC(NPC_IF), .PC(PC_IF), .stall(stall_IF));
    
    // 修改NPC连接：JAL指令在ID阶段生效，JALR指令在EX阶段生效，分支指令在EX阶段生效
    wire [2:0] npc_op_sel;
    wire [31:0] npc_imm_sel;
    wire [31:0] npc_pc_sel;
    
    // 选择NPC操作：JAL指令使用ID阶段的信号，其他使用EX阶段的信号
    assign npc_op_sel = (opcode_ID == `OPCODE_JAL) ? NPCOp_ID : 
                        (branch_taken_EX) ? `NPC_BRANCH : NPCOp_EX;
    assign npc_imm_sel = (opcode_ID == `OPCODE_JAL) ? imm_ID : imm_EX;
    assign npc_pc_sel = (opcode_ID == `OPCODE_JAL) ? PC_ID : PC_EX;
    
    NPC npc_unit(.PC(npc_pc_sel), .NPCOp(npc_op_sel), .IMM(npc_imm_sel), .NPC(NPC_IF), .aluout(alu_result_EX));
    assign instr_IF = instr_in;
    
    
    // IF Stage Hardware instantiation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // IF/ID pipeline register instantiation begins
    
    // IF/ID pipeline register - 使用冒险检测信号
    IF_ID_Reg if_id_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(flush_IF), 
        .stall(stall_IF),
        .PC_in(PC_IF), 
        .instr_in(instr_IF),
        .PC_out(PC_IF_ID), 
        .instr_out(instr_IF_ID)
    );

    // IF/ID pipeline register instantiation ends
    // ----------------------------------------------------------------
    
    // ID stage

    // 实例化冒险检测单元
    HazardDetectionUnit hazard_detection_unit(
        .rs1_ID(rs1_ID),
        .rs2_ID(rs2_ID),
        .rd_EX(rd_addr_EX),
        .rd_MEM(rd_addr_MEM),
        .MemRead_EX(MemRead_EX),
        .RegWrite_EX(RegWrite_EX),
        .RegWrite_MEM(RegWrite_MEM),
        .opcode_EX(opcode_EX),           // EX阶段的opcode
        .funct3_EX(funct3_EX),           // EX阶段的funct3
        .branch_taken_EX(branch_taken_EX), // EX阶段分支是否被采取
        .opcode_ID(opcode_ID),           // ID阶段的opcode，用于检测JAL指令
        .stall_IF(stall_IF),
        .flush_IF(flush_IF),
        .flush_ID(flush_ID),
        .flush_EX(flush_EX)
    );

    ctrl ctrl_unit(
        .Op(opcode_ID), 
        .Funct7(funct7_ID), 
        .Funct3(funct3_ID), 
        .Zero(Zero_ID),
        .RegWrite(RegWrite_ID), 
        .MemWrite(MemWrite_ID), 
        .MemRead(MemRead_ID),
        .EXTOp(EXTOp_ID), 
        .ALUOp(ALUOp_ID), 
        .NPCOp(NPCOp_ID),
        .ALUSrc(ALUSrc_ID), 
        .GPRSel(GPRSel_ID), 
        .WDSel(WDSel_ID), 
        .DMType(DMType_ID)
    );
    
    EXT ext_unit(
        .iimm_shamt(iimm_shamt_ID), 
        .iimm(iimm_ID), 
        .simm(simm_ID), 
        .bimm(bimm_ID),
        .uimm(uimm_ID), 
        .jimm(jimm_ID),
        .EXTOp(EXTOp_ID), 
        .immout(imm_ID)
    );
    
    RF rf_unit(
        .clk(clk), 
        .rst(rst), 
        .RFWr(RegWrite_WB),
        .A1(rs1_ID), 
        .A2(rs2_ID), 
        .A3(rd_addr_WB),
        .WD(wb_data_WB), 
        .RD1(rs1_data_ID), 
        .RD2(rs2_data_ID)
    );
    
    // ID/EX pipeline register
    ID_EX_Reg id_ex_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(flush_EX),
        .PC_in(PC_ID), 
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
    
    // EX stage
    wire [31:0] rs1_data_forwarded_EX, rs2_data_forwarded_EX;
    
    // 实例化前递单元
    ForwardingUnit forwarding_unit(
        .rs1_EX(rs1_addr_EX),
        .rs2_EX(rs2_addr_EX),
        .rd_MEM(rd_addr_MEM),
        .rd_WB(rd_addr_WB),
        .RegWrite_MEM(RegWrite_MEM),
        .RegWrite_WB(RegWrite_WB),
        .forward_rs1(forward_rs1),
        .forward_rs2(forward_rs2)
    );
    
    // 前递逻辑 - 根据前递单元的输出选择数据源
    assign rs1_data_forwarded_EX = (forward_rs1 == 2'b01) ? alu_result_MEM :  // 从MEM阶段前递
                                   (forward_rs1 == 2'b10) ? wb_data_WB :       // 从WB阶段前递
                                   rs1_data_EX;                                // 不使用前递
    
    assign rs2_data_forwarded_EX = (forward_rs2 == 2'b01) ? alu_result_MEM :  // 从MEM阶段前递
                                   (forward_rs2 == 2'b10) ? wb_data_WB :       // 从WB阶段前递
                                   rs2_data_EX;                                // 不使用前递
    
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
    
    // EX/MEM pipeline register
    EX_MEM_Reg ex_mem_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(flush_EX),
        .alu_result_in(alu_result_EX), 
        .rs2_data_in(rs2_data_forwarded_EX), 
        .rd_addr_in(rd_addr_EX),
        .RegWrite_in(RegWrite_EX), 
        .MemWrite_in(MemWrite_EX), 
        .MemRead_in(MemRead_EX),
        .WDSel_in(WDSel_EX), 
        .DMType_in(DMType_EX), 
        .PC_in(PC_EX),

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
    
    // MEM stage
    assign mem_data_MEM = Data_in;
    
    // MEM/WB pipeline register
    MEM_WB_Reg mem_wb_reg(
        .clk(clk), 
        .rst(rst),
        .alu_result_in(alu_result_MEM), 
        .mem_data_in(mem_data_MEM), 
        .rd_addr_in(rd_addr_MEM),
        .RegWrite_in(RegWrite_MEM), 
        .WDSel_in(WDSel_MEM), 
        .PC_in(PC_MEM),

        .alu_result_out(alu_result_MEM_WB), 
        .mem_data_out(mem_data_MEM_WB), 
        .rd_addr_out(rd_addr_MEM_WB),
        .RegWrite_out(RegWrite_MEM_WB), 
        .WDSel_out(WDSel_MEM_WB), 
        .PC_out(PC_MEM_WB)
    );
    
    // WB stage - register write happens in RF module
    
    // Register file debug output - 支持对内部寄存器文件的访问
    assign reg_data = (reg_sel == 5'b0) ? 32'b0 : 
                      (reg_sel == rs1_ID) ? rs1_data_ID :
                      (reg_sel == rs2_ID) ? rs2_data_ID :
                      (reg_sel == rd_addr_WB) ? wb_data_WB : 
                      rf_unit.rf[reg_sel];  // 直接访问寄存器文件

endmodule