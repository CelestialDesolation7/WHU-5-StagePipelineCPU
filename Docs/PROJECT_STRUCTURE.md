# RISC-V 五级流水线CPU项目结构说明

## 项目概述
本项目实现了一个完整的RISC-V RV32I五级流水线CPU，支持在Nexys4DDR FPGA开发板上运行，具有时钟分频、暂停控制和多种数据显示功能。

## 文件结构树形图

```
Main/
├── 核心CPU模块
│   ├── PipelineCPU.v              # 五级流水线CPU主模块
│   ├── sccomp.v                   # CPU顶层封装模块
│   ├── ctrl.v                     # 控制单元
│   ├── alu.v                      # 算术逻辑单元
│   ├── RF.v                       # 寄存器文件
│   ├── PC.v                       # 程序计数器
│   ├── NPC.v                      # 下一条指令地址计算
│   ├── EXT.v                      # 立即数扩展单元
│   ├── dm.v                       # 数据存储器
│   ├── im.v                       # 指令存储器
│   └── pipeline_regs.v            # 流水线寄存器
│
├── 控制信号定义
│   └── ctrl_encode_def.v          # 控制信号编码定义
│
├── 危险处理单元
│   └── hazard_units.v             # 冒险检测和转发单元
│
├── FPGA顶层模块
│   ├── top_module.v               # FPGA顶层模块（符合约束文件接口）
│   └── Nexys4DDR_CPU.xdc          # FPGA约束文件
│
├── 测试文件
│   ├── simple_test.v              # 简单功能测试
│   ├── pipeline_test.v            # 流水线功能测试
│   └── vivado_testbench.v         # Vivado仿真测试文件
│
└── 文档
    ├── README.md                  # 项目说明文档
    ├── IMPLEMENTATION_SUMMARY.md  # 实现总结
    └── PROJECT_STRUCTURE.md       # 项目结构说明（本文件）
```

## 模块层次结构

### 顶层模块层次
```
top_module (FPGA顶层)
├── sccomp (CPU系统)
│   ├── PipelineCPU (五级流水线CPU)
│   │   ├── PC (程序计数器)
│   │   ├── NPC (下一条指令地址)
│   │   ├── IF_ID_Reg (IF/ID流水线寄存器)
│   │   ├── ctrl (控制单元)
│   │   ├── EXT (立即数扩展)
│   │   ├── RF (寄存器文件)
│   │   ├── ID_EX_Reg (ID/EX流水线寄存器)
│   │   ├── alu (算术逻辑单元)
│   │   ├── EX_MEM_Reg (EX/MEM流水线寄存器)
│   │   └── MEM_WB_Reg (MEM/WB流水线寄存器)
│   ├── dm (数据存储器)
│   └── im (指令存储器)
└── seg7_controller (七段数码管控制器)
```

### 流水线阶段
```
IF (取指令) → ID (译码) → EX (执行) → MEM (访存) → WB (写回)
```

## 接口说明

### FPGA接口 (top_module.v)
- **输入信号**:
  - `clk`: 系统时钟 (100MHz)
  - `rstn`: 复位信号 (低电平有效)
  - `sw_i[15:0]`: 16位开关输入

- **输出信号**:
  - `led_o[15:0]`: 16位LED输出
  - `disp_seg_o[7:0]`: 七段数码管段选信号
  - `disp_an_o[7:0]`: 七段数码管位选信号

### 开关控制功能
- `sw_i[15]`: 时钟分频控制 (1=低速, 0=高速)
- `sw_i[1]`: CPU暂停控制 (1=暂停, 0=运行)
- `sw_i[0]`: 显示模式选择 (1=LED数据, 0=其他数据)
- `sw_i[14:11]`: 数据显示选择
  - `1000`: 显示当前指令
  - `0100`: 显示寄存器数据
  - `0010`: 显示ALU数据
  - `0001`: 显示数据存储器数据
- `sw_i[13]`: 寄存器数据轮询使能
- `sw_i[12]`: ALU数据轮询使能
- `sw_i[11]`: 数据存储器轮询使能

## 时钟分频逻辑

```verilog
// 时钟分频计数器
reg [27:0] clkdiv;

// 根据sw_i[15]选择分频速率
assign Clk_CPU = (sw_i[15]) ? clkdiv[27] : clkdiv[25];

// CPU工作时钟与暂停控制
assign Clk_instr = Clk_CPU & ~sw_i[1];
```

## 数据显示逻辑

### 寄存器数据显示
- 当`sw_i[13]=1`时，自动轮询显示所有寄存器
- 通过`sccomp.reg_data`输出访问寄存器内容

### ALU数据显示
- 当`sw_i[12]=1`时，轮询显示ALU相关数据
- 显示内容：PC、指令、ALU结果、数据输出

### 数据存储器显示
- 当`sw_i[11]=1`时，轮询显示数据存储器内容
- 显示ALU结果地址作为存储器数据

### 七段数码管显示
- 支持8位七段数码管显示
- 显示16位数据的低16位
- 自动扫描显示

## 测试文件说明

### simple_test.v
- 基本功能测试
- 验证CPU基本指令执行

### pipeline_test.v
- 流水线功能测试
- 验证数据冒险处理

### vivado_testbench.v
- Vivado仿真测试
- 测试所有开关控制功能
- 验证时钟分频和暂停功能


## 功能特性

### 支持的RISC-V指令
- **算术指令**: ADD, ADDI, SUB, LUI, AUIPC
- **逻辑指令**: AND, ANDI, OR, ORI, XOR, XORI
- **移位指令**: SLL, SLLI, SRL, SRLI, SRA, SRAI
- **比较指令**: SLT, SLTI, SLTU, SLTIU
- **分支指令**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **跳转指令**: JAL, JALR
- **存储器指令**: LW, LH, LHU, LB, LBU, SW, SH, SB

### 流水线特性
- 五级流水线：IF → ID → EX → MEM → WB
- 数据冒险检测和转发
- Load-Use冒险处理
- 流水线寄存器

### FPGA特性
- 时钟分频控制
- CPU暂停功能
- 多种数据显示模式
- 七段数码管显示
- LED状态指示

## 使用说明

### 基本操作
1. 将比特流文件下载到Nexys4DDR开发板
2. 使用开关控制CPU运行模式
3. 观察LED和七段数码管显示

### 调试功能
- 使用开关13-11轮询显示不同数据
- 使用开关15控制时钟速度便于观察
- 使用开关1暂停CPU执行

### 显示模式
- 开关0=0：显示CPU内部数据
- 开关0=1：显示LED数据（PC值）

这个项目提供了一个完整的RISC-V流水线CPU实现，具有良好的可观察性和调试功能，适合用于教学和实验。 