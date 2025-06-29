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
    output reg stall_IF,           // 暂停IF阶段的信号
    output reg flush_IF,           // 清空IF阶段的信号
    output reg flush_ID            // 清空ID阶段的信号
);
    // 检测是否为分支指令
    wire is_branch_EX;
    assign is_branch_EX = (opcode_EX == 7'b1100011); // BRANCH opcode
    
    always @(*) begin
        // Load-Use冒险检测逻辑
        // 当EX阶段是Load指令且目标寄存器与ID阶段的源寄存器相同时会发生冒险
        if (MemRead_EX && RegWrite_EX && 
            ((rd_EX == rs1_ID && rs1_ID != 5'b0) ||    // EX阶段目标寄存器与ID阶段rs1相同
             (rd_EX == rs2_ID && rs2_ID != 5'b0))) begin // EX阶段目标寄存器与ID阶段rs2相同
            stall_IF = 1'b1;  // 暂停IF阶段，防止新指令进入流水线
            flush_IF = 1'b1;  // 清空IF阶段，插入气泡
            flush_ID = 1'b0;  // 不清空ID阶段，保持当前指令
        end
        // 分支控制冒险检测逻辑
        // 当EX阶段是分支指令且分支被采取时，需要清空IF和ID阶段
        else if (is_branch_EX && branch_taken_EX) begin
            stall_IF = 1'b0;  // 不暂停IF阶段，允许取新指令
            flush_IF = 1'b1;  // 清空IF阶段，丢弃错误的指令
            flush_ID = 1'b1;  // 清空ID阶段，丢弃错误的指令
        end
        else begin
            // 没有冒险时，正常执行
            stall_IF = 1'b0;  // 不暂停IF阶段
            flush_IF = 1'b0;  // 不清空IF阶段
            flush_ID = 1'b0;  // 不清空ID阶段
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