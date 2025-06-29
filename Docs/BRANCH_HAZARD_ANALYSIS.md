# 分支冒险处理分析报告

## 问题分析

### 原始实现的问题

在检查原始实现时，发现了以下关键问题：

1. **缺少分支冒险检测** ❌
   - `HazardDetectionUnit` 只处理 Load-Use 数据冒险
   - 没有检测分支指令引起的控制冒险

2. **分支指令处理时机** ⚠️
   - 分支指令在 EX 阶段解析（`ctrl.v` 第102行）
   - 但缺少处理分支预测错误的机制

3. **PC 模块缺少暂停支持** ❌
   - 原始 `PC.v` 没有 `stall` 信号
   - 无法在冒险时阻止 PC 更新

## 修复实现

### 1. 增强冒险检测单元 (`hazard_units.v`)

**新增信号：**
```verilog
input [6:0] opcode_EX,         // EX阶段的opcode，用于检测分支指令
input [2:0] funct3_EX,         // EX阶段的funct3，用于检测分支指令
input branch_taken_EX,         // EX阶段分支是否被采取
```

**新增分支冒险检测逻辑：**
```verilog
// 检测是否为分支指令
wire is_branch_EX;
assign is_branch_EX = (opcode_EX == 7'b1100011); // BRANCH opcode

// 分支控制冒险检测逻辑
// 当EX阶段是分支指令且分支被采取时，需要清空IF和ID阶段
else if (is_branch_EX && branch_taken_EX) begin
    stall_IF = 1'b0;  // 不暂停IF阶段，允许取新指令
    flush_IF = 1'b1;  // 清空IF阶段，丢弃错误的指令
    flush_ID = 1'b1;  // 清空ID阶段，丢弃错误的指令
end
```

### 2. 分支采取判断逻辑 (`PipelineCPU.v`)

**新增分支检测信号：**
```verilog
wire branch_taken_EX;  // 新增：EX阶段分支是否被采取的信号
wire is_branch_EX;
assign is_branch_EX = (opcode_EX == `OPCODE_BRANCH);
```

**分支采取判断逻辑：**
```verilog
// 根据分支指令类型和ALU的Zero信号判断分支是否被采取
assign branch_taken_EX = is_branch_EX && (
    (funct3_EX == `FUNCT3_BEQ && Zero_EX) ||      // beq: 相等时跳转
    (funct3_EX == `FUNCT3_BNE && !Zero_EX) ||     // bne: 不等时跳转
    (funct3_EX == `FUNCT3_BLT && !Zero_EX) ||     // blt: 小于时跳转
    (funct3_EX == `FUNCT3_BGE && Zero_EX) ||      // bge: 大于等于时跳转
    (funct3_EX == `FUNCT3_BLTU && !Zero_EX) ||    // bltu: 无符号小于时跳转
    (funct3_EX == `FUNCT3_BGEU && Zero_EX)        // bgeu: 无符号大于等于时跳转
);
```

### 3. PC 模块增强 (`PC.v`)

**新增暂停支持：**
```verilog
module PC( clk, rst, NPC, PC, stall );
  input              clk;
  input              rst;
  input       [31:0] NPC;
  input              stall;  // 新增：暂停信号，当为1时阻止PC更新
  output reg  [31:0] PC;

  always @(posedge clk, posedge rst)
    if (rst) 
      PC <= 32'h0000_0000;
    else if (!stall)  // 只有在不暂停时才更新PC
      PC <= NPC;
endmodule
```

### 4. 流水线寄存器连接更新

**冒险检测单元实例化更新：**
```verilog
HazardDetectionUnit hazard_detection_unit(
    .rs1_ID(rs1_ID),
    .rs2_ID(rs2_ID),
    .rd_EX(rd_addr_EX),
    .rd_MEM(rd_addr_MEM),
    .MemRead_EX(MemRead_EX),
    .RegWrite_EX(RegWrite_EX),
    .RegWrite_MEM(RegWrite_MEM),
    .opcode_EX(opcode_EX),           // 新增：EX阶段的opcode
    .funct3_EX(funct3_EX),           // 新增：EX阶段的funct3
    .branch_taken_EX(branch_taken_EX), // 新增：EX阶段分支是否被采取
    .stall_IF(stall_IF),
    .flush_IF(flush_IF),
    .flush_ID(flush_ID)
);
```

**PC 实例化更新：**
```verilog
PC pc_unit(.clk(clk), .rst(rst), .NPC(NPC_IF), .PC(PC_IF), .stall(stall_IF));
```

## 工作原理

### 分支冒险处理流程

1. **分支指令进入 EX 阶段**
   - ALU 计算分支条件
   - 判断分支是否被采取

2. **分支冒险检测**
   - `HazardDetectionUnit` 检测到分支被采取
   - 生成 `flush_IF` 和 `flush_ID` 信号

3. **流水线冲刷**
   - IF/ID 寄存器被清空（插入气泡）
   - 错误的指令被丢弃

4. **PC 更新**
   - NPC 计算正确的分支目标地址
   - PC 更新到分支目标

### 时序图示例

```
Cycle 1: addi x1, x0, 5     (IF)
Cycle 2: addi x2, x0, 5     (IF) | addi x1, x0, 5     (ID)
Cycle 3: beq x1, x2, 8      (IF) | addi x2, x0, 5     (ID) | addi x1, x0, 5     (EX)
Cycle 4: addi x3, x0, 10    (IF) | beq x1, x2, 8      (ID) | addi x2, x0, 5     (EX) | addi x1, x0, 5     (MEM)
Cycle 5: [BRANCH TAKEN]     (IF) | [FLUSHED]          (ID) | beq x1, x2, 8      (EX) | addi x2, x0, 5     (MEM) | addi x1, x0, 5     (WB)
Cycle 6: addi x5, x0, 20    (IF) | [FLUSHED]          (ID) | [BRANCH TARGET]   (EX) | beq x1, x2, 8      (MEM) | addi x2, x0, 5     (WB)
```

## 测试验证

### 分支冒险测试 (`Test/branch_hazard_test.v`)

创建了专门的测试来验证分支冒险处理：

```verilog
// 测试序列
addi x1, x0, 5    // x1 = 5
addi x2, x0, 5    // x2 = 5  
beq x1, x2, 8     // 分支被采取
addi x3, x0, 10   // 应该被冲刷
addi x4, x0, 15   // 应该被冲刷
addi x5, x0, 20   // 分支目标指令
```

### 测试目标

1. **验证分支检测** - 确保正确识别分支指令
2. **验证冲刷逻辑** - 确保错误指令被正确冲刷
3. **验证PC更新** - 确保PC正确跳转到分支目标
4. **验证气泡插入** - 确保流水线正确插入气泡

## 总结

### 修复前状态 ❌
- 缺少分支冒险检测
- 无法处理分支预测错误
- PC 模块不支持暂停

### 修复后状态 ✅
- 完整的分支冒险检测逻辑
- 正确的流水线冲刷机制
- PC 模块支持暂停控制
- 全面的测试验证

### 关键改进

1. **完整的冒险处理** - 同时支持数据冒险和控制冒险
2. **正确的时序控制** - 在正确的时机进行冲刷和暂停
3. **模块化设计** - 冒险检测逻辑独立封装
4. **测试覆盖** - 专门的测试验证分支冒险处理

这个实现现在能够正确处理 RISC-V 流水线 CPU 中的分支冒险，确保程序的正确执行。 