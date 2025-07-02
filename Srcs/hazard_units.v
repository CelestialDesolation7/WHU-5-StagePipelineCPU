// 危险检测单元 - 检测和处理流水线中的数据冒险和控制冒险
// 该模块实现了Load-Use冒险检测和分支控制冒险检测
module HazardDetectionUnit(
    input [4:0] rs1_ID, rs2_ID,    // ID阶段的源寄存器地址
    input [4:0] rd_EX, rd_MEM,     // EX和MEM阶段的目标寄存器地址
    input MemRead_EX,              // EX阶段是否为Load指令
    input RegWrite_EX, RegWrite_MEM, // EX和MEM阶段是否写寄存器
    input [6:0] opcode_EX,         // EX阶段的opcode，用于检测分支指令
    input [2:0] funct3_EX,         // EX阶段的funct3，用于检测分支指令
    input branch_taken_EX,         // EX阶段分支是否被采取
    input [6:0] opcode_ID,         // ID阶段的opcode，用于检测JAL指令
    input [31:0] imm_EX,           // EX阶段的立即数
    input [31:0] imm_ID,           // ID阶段的立即数
    input [31:0] alu_result_EX,    // EX阶段ALU输出（JALR用）
    output reg stall_IF,           // 暂停IF阶段的信号
    output reg flush_IF,           // 清空IF阶段的信号
    output reg flush_ID,           // 清空ID阶段的信号
    output reg flush_EX,           // 清空EX阶段的信号
    output reg [2:0] NPCOp_out,
    output reg [31:0] NPCImm_out
);
    // 检测是否为分支指令
    wire is_branch_EX;
    assign is_branch_EX = (opcode_EX == `OPCODE_BRANCH); // BRANCH opcode
    
    // 检测是否为JAL指令
    wire is_jal_ID;
    assign is_jal_ID = (opcode_ID == `OPCODE_JAL); // JAL opcode
    
    // 检测是否为JALR指令
    wire is_jalr_EX;
    assign is_jalr_EX = (opcode_EX == `OPCODE_JALR); // JALR opcode
    
    always @(*) begin
        // Load-Use冒险检测逻辑
        // 当EX阶段是Load指令且目标寄存器与ID阶段的源寄存器相同时会发生冒险
        if (MemRead_EX && RegWrite_EX && 
            ((rd_EX == rs1_ID && rs1_ID != 5'b0) ||    // EX阶段目标寄存器与ID阶段rs1相同
             (rd_EX == rs2_ID && rs2_ID != 5'b0))) begin // EX阶段目标寄存器与ID阶段rs2相同
            stall_IF = 1'b1;  // 暂停PC和IF/ID寄存器
            flush_IF = 1'b0;  // IF阶段不需要冲刷，stall会使其保持
            flush_ID = 1'b0;  // ID阶段不需要冲刷，stall会使其保持
            flush_EX = 1'b1;  // 向EX阶段插入一个气泡 (NOP)
        end
        // JALR优先级最高
        else if (opcode_EX == `OPCODE_JALR) begin
            stall_IF = 1'b0;
            flush_IF = 1'b1;
            flush_ID = 1'b1;
            flush_EX = 1'b0;
        end
        // Branch次之
        else if ((opcode_EX == `OPCODE_BRANCH) && branch_taken_EX) begin
            stall_IF = 1'b0;
            flush_IF = 1'b1;
            flush_ID = 1'b1;
            flush_EX = 1'b0;
        end
        else if (opcode_ID == `OPCODE_JAL) begin
            stall_IF = 1'b0;
            flush_IF = 1'b1;
            flush_ID = 1'b0;
            flush_EX = 1'b0;
        end
        else begin
            stall_IF = 1'b0;
            flush_IF = 1'b0;
            flush_ID = 1'b0;
            flush_EX = 1'b0;
        end


        // NPCOp/NPCImm优先级决策
        if (opcode_EX == `OPCODE_JALR) begin
            NPCOp_out = `NPC_JALR;
            NPCImm_out = 32'b0; // JALR用alu_result_EX
        end else if ((opcode_EX == `OPCODE_BRANCH) && branch_taken_EX) begin
            NPCOp_out = `NPC_BRANCH;
            NPCImm_out = imm_EX;
        end else if (opcode_ID == `OPCODE_JAL) begin
            NPCOp_out = `NPC_JUMP;
            NPCImm_out = imm_ID;
        end else begin
            NPCOp_out = `NPC_PLUS4;
            NPCImm_out = 32'b0;
        end
    end
endmodule

// 转发单元 - 实现数据转发以解决数据冒险
// 该模块检测数据依赖并生成转发控制信号，将最新数据转发到EX阶段
module ForwardingUnit(
    input [4:0] rs1_EX, rs2_EX,    // EX阶段的源寄存器地址
    input [4:0] rd_MEM, rd_WB,     // MEM和WB阶段的目标寄存器地址
    input RegWrite_MEM, RegWrite_WB, // MEM和WB阶段是否写寄存器
    output reg [1:0] forward_rs1,  // rs1的转发控制信号
    output reg [1:0] forward_rs2   // rs2的转发控制信号
);
    always @(*) begin
        // rs1的转发逻辑
        if (RegWrite_MEM && rd_MEM != 5'b0 && rd_MEM == rs1_EX)
            // 如果MEM阶段写寄存器且目标寄存器与EX阶段的rs1相同
            forward_rs1 = 2'b01;  // 从MEM阶段转发数据
        else if (RegWrite_WB && rd_WB != 5'b0 && rd_WB == rs1_EX)
            // 如果WB阶段写寄存器且目标寄存器与EX阶段的rs1相同
            forward_rs1 = 2'b10;  // 从WB阶段转发数据
        else
            // 没有数据依赖，不需要转发
            forward_rs1 = 2'b00;  // 不转发，使用寄存器堆中的数据
            
        // rs2的转发逻辑（与rs1类似）
        if (RegWrite_MEM && rd_MEM != 5'b0 && rd_MEM == rs2_EX)
            // 如果MEM阶段写寄存器且目标寄存器与EX阶段的rs2相同
            forward_rs2 = 2'b01;  // 从MEM阶段转发数据
        else if (RegWrite_WB && rd_WB != 5'b0 && rd_WB == rs2_EX)
            // 如果WB阶段写寄存器且目标寄存器与EX阶段的rs2相同
            forward_rs2 = 2'b10;  // 从WB阶段转发数据
        else
            // 没有数据依赖，不需要转发
            forward_rs2 = 2'b00;  // 不转发，使用寄存器堆中的数据
    end
endmodule 