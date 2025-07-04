`include "ctrl_encode_def.v"

module PipelineCPU(
    input clk,
    input rst,
    input [31:0] instr_in,  // 读入指令值
    input [31:0] Data_in,   // 读入数据值

    output mem_w,           // 写使能信号
    output [31:0] PC_out,   // 输出PC值
    output [31:0] Addr_out, // 输出地址值
    output [31:0] Data_out, // 输出数据值
    input [4:0] reg_sel,    // 读寄存器地址
    output [31:0] reg_data, // 输出寄存器数据
    output [2:0] DMType_out, // 输出数据类型
    output [31:0] debug_data // 输出调试数据
);

    // ----------------------------------------------------------------
    // Control signals wire preparation begins
    // IF stage signals
    wire [31:0] PC_IF;      // 输出PC值
    
    // ID stage signals
    // 长连接
    wire [31:0] instr_ID;   // 输出指令值，表示此阶段正在执行的指令
    wire [31:0] PC_ID;      // ID阶段PC值
    wire [31:0] rs1_data_ID; // 源寄存器1数据，在此阶段读出，向后传递等待计算或写入
    wire [31:0] rs2_data_ID; // 源寄存器2数据，在此阶段读出，向后传递等待计算或写入
    wire [31:0] imm_ID; // 立即数，在此阶段拼装，向后传递等待计算
    // 控制信号短连接
    wire RegWrite_ID; // 输出寄存器写使能信号，表示指令是否需要写入寄存器
    wire MemWrite_ID; // 输出内存写使能信号，表示指令是否需要写入内存
    wire MemRead_ID; // 输出内存读使能信号，表示指令是否需要读取内存
    wire [5:0] EXTOp_ID; // 输出扩展操作类型，表示指令是否需要扩展操作
    wire [4:0] ALUOp_ID; // 输出ALU操作类型，表示指令需要何种ALU操作
    wire ALUSrc_ID; // 输出ALU源选择信号
    wire [1:0] WDSel_ID; // 输出写数据选择信号
    wire [2:0] DMType_ID; // 输出数据类型
    
    // EX stage signals
    // 长连接
    wire [31:0] instr_EX; // 输出指令值
    wire [31:0] PC_EX;  // 输出PC值
    wire [31:0] rs1_data_EX; // 输出寄存器数据
    wire [31:0] rs2_data_EX; // 输出寄存器数据
    wire [31:0] imm_EX; // 输出立即数
    // 控制信号短连接
    wire RegWrite_EX; // 输出寄存器写使能信号
    wire MemWrite_EX; // 输出内存写使能信号
    wire MemRead_EX; // 输出内存读使能信号
    wire [4:0] ALUOp_EX; // 输出ALU操作类型
    wire ALUSrc_EX; // 输出ALU源选择信号
    wire [1:0] WDSel_EX; // 输出写数据选择信号
    wire [2:0] DMType_EX; // 输出数据类型
    wire [31:0] alu_result_EX, alu_B_EX; // 输出ALU结果
    wire Zero_EX, Sign_EX, Overflow_EX, Carry_EX; // 输出标志信号
    
    // MEM stage signals
    // 长连接
    wire [31:0] instr_MEM; // 输出指令值
    wire [31:0] alu_result_MEM; // 输出ALU结果
    wire [31:0] rs2_data_MEM; // 输出寄存器数据
    wire [31:0] PC_MEM; // 输出PC值
    wire [31:0] mem_data_MEM; // 输出内存数据
    wire [4:0] rd_addr_MEM = instr_MEM[11:7]; // 输出寄存器地址
    // 控制信号短连接
    wire RegWrite_MEM; // 输出寄存器写使能信号
    wire MemWrite_MEM; // 输出内存写使能信号
    wire MemRead_MEM; // 输出内存读使能信号
    wire [1:0] WDSel_MEM; // 输出写数据选择信号
    wire [2:0] DMType_MEM; // 输出数据类型
    
    // WB stage signals
    // 长连接
    wire [31:0] instr_WB; // 输出指令值
    wire [31:0] alu_result_WB; // 输出ALU结果
    wire [31:0] mem_data_WB; // 输出内存数据
    wire [31:0] PC_WB; // 输出PC值
    wire [31:0] wb_data_WB; // 输出写数据
    wire [4:0] rd_addr_WB = instr_WB[11:7]; // 输出寄存器地址
    // 控制信号短连接
    wire RegWrite_WB; // 输出寄存器写使能信号
    wire [1:0] WDSel_WB; // 输出写数据选择信号
    

    // Control signals wire preparation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // Instruction decode wire preparation begins
    wire [4:0] rs1_ID, rs2_ID;
    wire [6:0] opcode_ID, funct7_ID;
    wire [2:0] funct3_ID;

    assign opcode_ID = instr_ID[6:0];
    assign funct7_ID = instr_ID[31:25];
    assign funct3_ID = instr_ID[14:12];
    assign rs1_ID = instr_ID[19:15];
    assign rs2_ID = instr_ID[24:20];
    
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

    wire [4:0] rs1_addr_EX = instr_EX[19:15];
    wire [4:0] rs2_addr_EX = instr_EX[24:20];
    wire [4:0] rd_addr_EX = instr_EX[11:7];
    wire [6:0] opcode_EX = instr_EX[6:0];
    wire [2:0] funct3_EX = instr_EX[14:12];
    // Instruction Decode wire preparation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // Hazard detection and forwarding signals preparation begins
    wire stall_IF; // 暂停IF阶段
    wire flush_ID; // 清空ID阶段
    wire flush_EX; // 清空EX阶段
    wire [1:0] forward_rs1_EX; // EX阶段源寄存器1前递信号
    wire [1:0] forward_rs2_EX; // EX阶段源寄存器2前递信号
    wire forward_rs1_ID; // ID阶段源寄存器1前递信号
    wire forward_rs2_ID; // ID阶段源寄存器2前递信号
    wire [31:0] rs1_data_forwarded_EX, rs2_data_forwarded_EX;
    wire [31:0] rs1_data_forwarded_ID, rs2_data_forwarded_ID;
    wire branch_taken_EX;

    // Write back data selection
    assign wb_data_WB = (WDSel_WB == `WDSel_FromALU) ? alu_result_WB :
                        (WDSel_WB == `WDSel_FromMEM) ? mem_data_WB :
                        (WDSel_WB == `WDSel_FromPC) ? (PC_WB + 4) : alu_result_WB;
    
    // EX阶段的分支判断逻辑 - 基于ALU标志位

    assign branch_taken_EX = (opcode_EX == `OPCODE_BRANCH) && (
        (funct3_EX == `FUNCT3_BEQ && Zero_EX) ||                    // beq: 相等时跳转
        (funct3_EX == `FUNCT3_BNE && !Zero_EX) ||                   // bne: 不等时跳转
        (funct3_EX == `FUNCT3_BLT && (Sign_EX ^ Overflow_EX)) ||    // blt: 有符号小于时跳转
        (funct3_EX == `FUNCT3_BGE && !(Sign_EX ^ Overflow_EX)) ||   // bge: 有符号大于等于时跳转
        (funct3_EX == `FUNCT3_BLTU && Carry_EX) ||                  // bltu: 无符号小于时跳转
        (funct3_EX == `FUNCT3_BGEU && !Carry_EX)                    // bgeu: 无符号大于等于时跳转
    );
    


    // ID阶段前递逻辑 - 从WB阶段前递数据到ID阶段
    assign rs1_data_forwarded_ID = forward_rs1_ID ? wb_data_WB :  // 从WB阶段前递
                                   rs1_data_ID;                  // 不使用前递
    
    assign rs2_data_forwarded_ID = forward_rs2_ID ? wb_data_WB :  // 从WB阶段前递
                                   rs2_data_ID;                  // 不使用前递

    // PC_NPC: 统一PC与下一个PC的计算
    // npc_op_sel和npc_imm_sel由HazardDetectionUnit输出
    wire [2:0] npc_op_sel;
    wire [31:0] npc_imm_sel;
    wire [31:0] npc_base_pc;

    // EX阶段前递逻辑 - 根据前递单元的输出选择数据源
    assign rs1_data_forwarded_EX = (forward_rs1_EX == 2'b01) ? alu_result_MEM :  // 从MEM阶段前递
                                   (forward_rs1_EX == 2'b10) ? wb_data_WB :       // 从WB阶段前递
                                   rs1_data_EX;                                // 不使用前递
    
    assign rs2_data_forwarded_EX = (forward_rs2_EX == 2'b01) ? alu_result_MEM :  // 从MEM阶段前递
                                   (forward_rs2_EX == 2'b10) ? wb_data_WB :       // 从WB阶段前递
                                   rs2_data_EX;                                // 不使用前递
    
    // ALU B operand selection
    assign alu_B_EX = ALUSrc_EX ? imm_EX : rs2_data_forwarded_EX;

    // ----------------------------------------------------------------
    // IF Stage Hardware instantiation begins


    PC_NPC pc_npc_unit(
        .clk(clk),
        .rst(rst),
        .stall(stall_IF),
        .base_PC(npc_base_pc),
        .NPCOp(npc_op_sel),
        .IMM(npc_imm_sel),
        .aluout(alu_result_EX),
        .PC(PC_IF)
    );
    // IF Stage Hardware instantiation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // IF/ID pipeline register instantiation begins
    // IF/ID pipeline register - 使用冒险检测信号
    IF_ID_Reg if_id_reg(
        .clk(clk), 
        .rst(rst), 
        .flush(flush_ID),  // 支持IF和ID阶段的flush
        .stall(stall_IF),
        .PC_in(PC_IF), 
        .instr_in(instr_in),
        .PC_out(PC_ID), 
        .instr_out(instr_ID)
    );
    // IF/ID pipeline register instantiation ends
    // ----------------------------------------------------------------
    


    // ----------------------------------------------------------------
    // HazardDetectionUnit instantiation begins
    // 实例化冒险检测单元
    HazardDetectionUnit hazard_detection_unit(
        .rs1_ID(rs1_ID),
        .rs2_ID(rs2_ID),
        .rd_EX(rd_addr_EX),
        .rd_MEM(rd_addr_MEM),
        .MemRead_EX(MemRead_EX),
        .MemRead_MEM(MemRead_MEM),
        .RegWrite_EX(RegWrite_EX),
        .RegWrite_MEM(RegWrite_MEM),
        .opcode_EX(opcode_EX),           // EX阶段的opcode
        .funct3_EX(funct3_EX),           // EX阶段的funct3
        .branch_taken_EX(branch_taken_EX), // EX阶段分支是否被采取
        .opcode_ID(opcode_ID),           // ID阶段的opcode，用于检测JAL指令
        .imm_EX(imm_EX),                 // EX阶段立即数
        .imm_ID(imm_ID),                 // ID阶段立即数
        .alu_result_EX(alu_result_EX),   // EX阶段ALU输出
        .PC_EX(PC_EX),                   // EX阶段PC
        .PC_ID(PC_ID),                   // ID阶段PC
        .stall_IF(stall_IF),
        .flush_ID(flush_ID),
        .flush_EX(flush_EX),
        .NPCOp_out(npc_op_sel),
        .NPCImm_out(npc_imm_sel),
        .base_PC_out(npc_base_pc)
    );
    // HazardDetectionUnit instantiation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // ID Stage Hardware instantiation begins
    ctrl ctrl_unit(
        .Op(opcode_ID), 
        .Funct7(funct7_ID), 
        .Funct3(funct3_ID), 
        .RegWrite(RegWrite_ID), 
        .MemWrite(MemWrite_ID), 
        .MemRead(MemRead_ID),
        .EXTOp(EXTOp_ID), 
        .ALUOp(ALUOp_ID), 
        .ALUSrc(ALUSrc_ID), 
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
        .RD2(rs2_data_ID),
        .reg_sel(reg_sel),
        .reg_data(reg_data)
    );
    

    // ID Stage Hardware instantiation ends
    // ----------------------------------------------------------------
    


    // ----------------------------------------------------------------
    // ID/EX pipeline register instantiation begins
    ID_EX_Reg id_ex_reg(
        .clk(clk),
        .rst(rst),
        .flush(flush_EX),
        .PC_in(PC_ID),
        .instr_in(instr_ID),
        .rs1_data_in(rs1_data_forwarded_ID),
        .rs2_data_in(rs2_data_forwarded_ID),
        .imm_in(imm_ID),
        .RegWrite_in(RegWrite_ID),
        .MemWrite_in(MemWrite_ID),
        .MemRead_in(MemRead_ID),
        .ALUOp_in(ALUOp_ID),
        .ALUSrc_in(ALUSrc_ID),
        .WDSel_in(WDSel_ID),
        .DMType_in(DMType_ID),
        .PC_out(PC_EX),
        .instr_out(instr_EX),
        .rs1_data_out(rs1_data_EX),
        .rs2_data_out(rs2_data_EX),
        .imm_out(imm_EX),
        .RegWrite_out(RegWrite_EX),
        .MemWrite_out(MemWrite_EX),
        .MemRead_out(MemRead_EX),
        .ALUOp_out(ALUOp_EX),
        .ALUSrc_out(ALUSrc_EX),
        .WDSel_out(WDSel_EX),
        .DMType_out(DMType_EX)
    );
    // ID/EX pipeline register instantiation ends
    // ----------------------------------------------------------------
    


    // ----------------------------------------------------------------
    // EXE Stage Hardware instantiation begins
    // 实例化前递单元
    ForwardingUnit forwarding_unit(
        .rs1_EX(rs1_addr_EX),
        .rs2_EX(rs2_addr_EX),
        .rs1_ID(rs1_ID),
        .rs2_ID(rs2_ID),
        .rd_MEM(rd_addr_MEM),
        .rd_WB(rd_addr_WB),
        .RegWrite_MEM(RegWrite_MEM),
        .RegWrite_WB(RegWrite_WB),
        .forward_rs1_EX(forward_rs1_EX),
        .forward_rs2_EX(forward_rs2_EX),
        .forward_rs1_ID(forward_rs1_ID),
        .forward_rs2_ID(forward_rs2_ID)
    );
    


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
    // EXE Stage Hardware instantiation ends
    // ----------------------------------------------------------------
    


    // ----------------------------------------------------------------
    // EX/MEM pipeline register instantiation begins
    EX_MEM_Reg ex_mem_reg(
        .clk(clk), 
        .rst(rst), 
        
        .alu_result_in(alu_result_EX), 
        .rs2_data_in(rs2_data_forwarded_EX), 
        .instr_in(instr_EX),
        .RegWrite_in(RegWrite_EX), 
        .MemWrite_in(MemWrite_EX), 
        .MemRead_in(MemRead_EX),
        .WDSel_in(WDSel_EX), 
        .DMType_in(DMType_EX), 
        .PC_in(PC_EX),

        .alu_result_out(alu_result_MEM), 
        .rs2_data_out(rs2_data_MEM), 
        .instr_out(instr_MEM),
        .RegWrite_out(RegWrite_MEM), 
        .MemWrite_out(MemWrite_MEM), 
        .MemRead_out(MemRead_MEM),
        .WDSel_out(WDSel_MEM), 
        .DMType_out(DMType_MEM), 
        .PC_out(PC_MEM)
    );
    // EX/MEM pipeline register instantiation ends
    // ----------------------------------------------------------------




    // ----------------------------------------------------------------
    // MEM stage Hardware instantiation begins
    assign mem_data_MEM = Data_in;
    
    // MEM/WB pipeline register
    MEM_WB_Reg mem_wb_reg(
        .clk(clk), 
        .rst(rst), 

        .alu_result_in(alu_result_MEM), 
        .mem_data_in(mem_data_MEM), 
        .instr_in(instr_MEM),
        .RegWrite_in(RegWrite_MEM), 
        .WDSel_in(WDSel_MEM), 
        .PC_in(PC_MEM),

        .alu_result_out(alu_result_WB), 
        .mem_data_out(mem_data_WB), 
        .instr_out(instr_WB),
        .RegWrite_out(RegWrite_WB), 
        .WDSel_out(WDSel_WB), 
        .PC_out(PC_WB)
    );
    
    // MEM stage Hardware instantiation ends
    // ----------------------------------------------------------------



    // ----------------------------------------------------------------
    // Output assignments begins
    assign PC_out = PC_IF;
    assign Addr_out = alu_result_MEM;
    assign Data_out = rs2_data_MEM;
    assign mem_w = MemWrite_MEM;
    assign DMType_out = DMType_MEM;
    assign debug_data = PC_IF;
    // Output assignments ends
    // ----------------------------------------------------------------
endmodule