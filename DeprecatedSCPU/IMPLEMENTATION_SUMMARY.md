# RISC-V RV32I 五级流水线CPU实现总结

## 项目概述

本项目实现了一个完整的RISC-V RV32I指令集五级流水线CPU，支持所有基本指令类型，包括算术运算、逻辑运算、存储器访问、分支跳转等。

## 实现的指令集

### I0: 立即数指令
- **LUI** (Load Upper Immediate) - 将20位立即数加载到寄存器的高20位
- **AUIPC** (Add Upper Immediate to PC) - 将20位立即数与PC相加

### I1: 跳转和分支指令
- **JAL** (Jump and Link) - 无条件跳转并保存返回地址
- **JALR** (Jump and Link Register) - 基于寄存器的跳转并保存返回地址
- **BEQ** (Branch if Equal) - 相等时分支
- **BNE** (Branch if Not Equal) - 不相等时分支
- **BLT** (Branch if Less Than) - 小于时分支
- **BGE** (Branch if Greater than or Equal) - 大于等于时分支
- **BLTU** (Branch if Less Than Unsigned) - 无符号小于时分支
- **BGEU** (Branch if Greater than or Equal Unsigned) - 无符号大于等于时分支

### I2: 存储器访问指令
- **LB** (Load Byte) - 加载字节
- **LH** (Load Halfword) - 加载半字
- **LW** (Load Word) - 加载字
- **LBU** (Load Byte Unsigned) - 加载无符号字节
- **LHU** (Load Halfword Unsigned) - 加载无符号半字
- **SB** (Store Byte) - 存储字节
- **SH** (Store Halfword) - 存储半字
- **SW** (Store Word) - 存储字

### I3: 立即数算术逻辑指令
- **ADDI** (Add Immediate) - 立即数加法
- **SLTI** (Set if Less Than Immediate) - 立即数比较
- **SLTIU** (Set if Less Than Immediate Unsigned) - 无符号立即数比较
- **XORI** (XOR Immediate) - 立即数异或
- **ORI** (OR Immediate) - 立即数或
- **ANDI** (AND Immediate) - 立即数与
- **SLLI** (Shift Left Logical Immediate) - 立即数逻辑左移
- **SRLI** (Shift Right Logical Immediate) - 立即数逻辑右移
- **SRAI** (Shift Right Arithmetic Immediate) - 立即数算术右移

### I4: 寄存器算术逻辑指令
- **ADD** (Add) - 寄存器加法
- **SUB** (Subtract) - 寄存器减法
- **SLL** (Shift Left Logical) - 逻辑左移
- **SLT** (Set if Less Than) - 有符号比较
- **SLTU** (Set if Less Than Unsigned) - 无符号比较
- **XOR** (XOR) - 异或
- **SRL** (Shift Right Logical) - 逻辑右移
- **SRA** (Shift Right Arithmetic) - 算术右移
- **OR** (OR) - 或
- **AND** (AND) - 与

## 流水线架构

### 五级流水线阶段

1. **IF (Instruction Fetch)**
   - 从指令存储器读取指令
   - 更新程序计数器
   - 实现模块：PC, NPC, IM

2. **ID (Instruction Decode)**
   - 指令译码和解析
   - 寄存器读取
   - 立即数扩展
   - 控制信号生成
   - 实现模块：ctrl, EXT, RF

3. **EX (Execute)**
   - ALU运算
   - 分支条件判断
   - 地址计算
   - 实现模块：alu

4. **MEM (Memory)**
   - 数据存储器访问
   - 支持字节、半字、字访问
   - 实现模块：dm

5. **WB (Write Back)**
   - 结果写回寄存器堆
   - 实现模块：RF

### 流水线寄存器

- **IF/ID寄存器** - 保存IF阶段结果
- **ID/EX寄存器** - 保存ID阶段结果
- **EX/MEM寄存器** - 保存EX阶段结果
- **MEM/WB寄存器** - 保存MEM阶段结果

## 关键技术特性

### 数据冒险处理
- **数据转发(Data Forwarding)**：从MEM和WB阶段转发数据到EX阶段
- **转发单元**：检测数据依赖并生成转发控制信号

### 控制冒险处理
- **分支预测**：总是预测不跳转
- **分支延迟槽**：分支指令在EX阶段确定跳转目标
- **流水线清空**：跳转时清空流水线

### 结构冒险处理
- **分离存储器**：指令存储器和数据存储器分离
- **多端口寄存器堆**：支持同时读写

## 模块详细说明

### 核心模块

1. **PipelineCPU.v** - 五级流水线CPU主模块
   - 连接所有流水线阶段
   - 实现数据转发
   - 处理流水线控制

2. **ctrl.v** - 控制单元
   - 指令译码
   - 控制信号生成
   - 支持所有RISC-V RV32I指令

3. **alu.v** - 算术逻辑单元
   - 支持所有算术和逻辑运算
   - 支持移位操作
   - 支持比较操作

4. **RF.v** - 寄存器堆
   - 32个32位通用寄存器
   - 支持同时读写
   - 异步读取，同步写入

5. **dm.v** - 数据存储器
   - 支持字节、半字、字访问
   - 同步写入，异步读取
   - 支持有符号和无符号扩展

### 辅助模块

1. **pipeline_regs.v** - 流水线寄存器
2. **hazard_units.v** - 危险检测和转发单元
3. **PC.v** - 程序计数器
4. **NPC.v** - 下一程序计数器
5. **EXT.v** - 立即数扩展单元
6. **im.v** - 指令存储器

## 测试验证

### 测试程序
包含完整的测试指令序列，验证：
- 基本算术运算
- 逻辑运算
- 移位操作
- 比较操作
- 分支跳转
- 存储器访问

### 测试文件
- **simple_test.v** - 基本功能测试
- **pipeline_test.v** - 完整流水线测试

## 性能特点

1. **吞吐量**：每个时钟周期可以完成一条指令（理想情况）
2. **延迟**：指令执行需要5个时钟周期
3. **效率**：通过数据转发减少流水线停顿
4. **兼容性**：完全兼容RISC-V RV32I指令集

## 扩展性

该设计具有良好的扩展性：
- 可以轻松添加新的指令类型
- 可以扩展为支持更多寄存器
- 可以添加缓存和更复杂的分支预测
- 可以支持向量指令和浮点运算

## 总结

本项目成功实现了一个功能完整的RISC-V RV32I五级流水线CPU，支持所有基本指令类型，具备现代CPU的基本特性，包括数据转发、分支预测等。该实现为学习计算机体系结构和CPU设计提供了良好的基础。 