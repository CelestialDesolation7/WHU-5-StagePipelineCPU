# WHU-5-Stage-PipelineCPU

# <font color='#f35336'>**如果你真的希望用这个项目验收，请确保你理解了Arielle的代码，不要让老师发现Arielle的这个repo喵！**</font>

如果你觉得这个项目对你有帮助的话给Arielle点个star⭐谢谢喵！

## 这是什么？

这是2025年武汉大学计算机学院，计算机类，计算机科学与技术专业方向，本科二年级第三学期的"计算机系统综合设计A/B"课程作业。该作业是一个经典的五级流水线架构CPU，实现RISC-V 32I指令集。

项目编写自2025年6月28日夜晚开始，到2025年7月4日中午结束，耗时约5.5天（包含Arielle的摸鱼时间喵）。

## 如何使用？

### 环境要求

- **Vivado 2020.1** 或更高版本
- **Nexys4 DDR** FPGA开发板

此项目的文件结构为：
```
MyPipelineCPU/
├── Srcs/                         # 源代码文件
│   ├── PipelineCPU.v             # 五级流水线CPU顶层模块
│   ├── PC.v                      # 程序计数器
│   ├── alu.v                     # 算术逻辑单元
│   ├── ctrl.v                    # 控制单元
│   ├── ctrl_encode_def.v         # 控制信号定义
│   ├── dm.v                      # 数据存储器
│   ├── EXT.v                     # 立即数扩展单元
│   ├── hazard_units.v            # 冒险检测和前递单元
│   ├── pipeline_regs.v           # 流水线寄存器模块
│   ├── RF.v                      # 寄存器文件
│   ├── sccomp.v                  # CPU系统顶层
│   ├── top_module.v              # FPGA顶层模块
│   └── dm_32bit.v                # 32位地址空间数据存储器（未使用）
├── Test/                         # 测试文件
│   ├── simple_dump.py            # 简单反汇编脚本
│   ├── riscv-studentnosorting.coe # 指令存储器初始化文件
│   ├── Venus.txt                 # Venus模拟器输出
│   └── disasm_output.txt         # 反汇编输出
├── Docs/                         # 文档
│   ├── 验收教程by Arielle.md     # 详细的验收指导文档
│   └── assets/                   # 文档资源文件
├── Refs/                         # 参考资料
│   ├── COAD AUX MEM.md           # 计组基础知识笔记
│   ├── info.json                 # 项目信息
│   └── assets/                   # 参考资料资源文件
├── Nexys4DDR_CPU.xdc             # FPGA约束文件
└── README.md                     # 项目说明文档
```
要将这个项目描述的CPU在您的电脑上运行起来，您可以直接在Release中下载已经编译好的bit文件，然后烧录到您的FPGA开发板上喵。

如果您希望在您的电脑上从头运行这个项目，您需要执行以下步骤喵：

1. 将仓库克隆到本地喵：
  ```bash
  git clone https://github.com/CelestialDesolation7/WHU-5-StagePipelineCPU.git
  cd WHU-5-StagePipelineCPU
  ```

2. 使用Vivado新建一个项目喵：
	- 打开Vivado，选择"Create Project"
	- 项目名称：`MyPipelineCPU`
	- 项目类型：选择"RTL Project"
	- 目标器件：选择`xc7a100tcsg324-1`（Nexys4 DDR）
3. 将源代码文件和仿真文件分别添加到项目喵：
	- 在Vivado中，右键点击"Sources"窗口
	- 选择"Add Sources" → "Add or create design sources"
	- 添加以下文件：
		- `Srcs/top_module.v`（设为顶层模块）
		- `Srcs/sccomp.v`
		- `Srcs/PipelineCPU.v`
		- `Srcs/PC.v`
		- `Srcs/alu.v`
		- `Srcs/ctrl.v`
		- `Srcs/ctrl_encode_def.v`
		- `Srcs/dm.v`
		- `Srcs/EXT.v`
		- `Srcs/hazard_units.v`
		- `Srcs/pipeline_regs.v`
		- `Srcs/RF.v`
	- 添加约束文件
		- 右键点击"Sources"窗口
		- 选择"Add Sources" → "Add or create constraints"
		- 添加`Nexys4DDR_CPU.xdc`
	- 创建ROM IP核
	  - 此项目中ROM IP核名为dist_mem_gen_0，如果你要改名，请修改sccomp中的实例化喵。
	- 添加测试文件（可选）
		- 右键点击"Sources"窗口
		- 选择"Add Sources" → "Add or create simulation sources"
		- 添加`Test/sccomp_rom_testbench.v`
		- 右键点击sccomp_rom_testbench.v，选择"Set as Top"
		- 点击左侧导航栏中的"Run Simulation" → "Run Behavioral Simulation"
		- 如果一切顺利，您应该会看到仿真结果喵。
4. 生成位流并下板运行喵：
	- 点击"Run Synthesis"
	- 综合完成后，点击"Run Implementation"
	- 实现完成后，点击"Generate Bitstream"
	- 将Nexys4 DDR开发板连接到电脑
	- 点击"Open Hardware Manager"
	- 点击"Open target" → "Auto Connect"
	- 右键点击FPGA设备，选择"Program Device"
	- 选择生成的.bit文件，点击"Program"

然后无论你是否希望用这个项目去验收，阅读[验收教程](https://github.com/CelestialDesolation7/WHU-5-StagePipelineCPU/blob/main/Docs/%E9%AA%8C%E6%94%B6%E6%95%99%E7%A8%8Bby%20Arielle.md)都能帮助您理解喵。

## 技术架构摘要

### 流水线阶段

1. **IF（取指令）**：PC_NPC模块负责指令地址计算和取指令
2. **ID（指令解码）**：控制单元、寄存器文件、立即数扩展
3. **EX（执行）**：ALU执行算术逻辑运算
4. **MEM（存储器访问）**：数据存储器读写操作
5. **WB（写回）**：寄存器写回操作

### 冒险处理

- **数据冒险**：通过ForwardingUnit实现前递
- **Load-Use冒险**：通过HazardDetectionUnit检测，结合前递解决
- **控制冒险**：通过HazardDetectionUnit检测，清空流水线

## 指令支持

### 算术指令

- `add`, `addi`, `sub`：加法和减法
- `slt`, `slti`, `sltu`, `sltiu`：比较指令
- `lui`, `auipc`：高位立即数指令

### 逻辑指令

- `and`, `andi`, `or`, `ori`, `xor`, `xori`：逻辑运算
- `sll`, `slli`, `srl`, `srli`, `sra`, `srai`：移位指令

### 存储器指令

- `lw`, `lh`, `lhu`, `lb`, `lbu`：加载指令
- `sw`, `sh`, `sb`：存储指令

### 控制指令

- `beq`, `bne`, `blt`, `bge`, `bltu`, `bgeu`：条件分支
- `jal`, `jalr`：无条件跳转


## 32位地址空间内存实现指南

⚠警告：此项目默认使用dm.v作为内存，仅支持7位字地址，即9位字节地址，合计512Byte喵⚠

如果你的测试程序需要访问32位地址空间，请看本文最后的教程喵

（Arielle的测试程序没有用到这么大的地址喵）

在FPGA中实现32位地址空间（4GB）的内存时，不能简单地在Verilog中分配如此大量的寄存器，因为这会消耗巨大的硬件资源。

### 步骤1：创建Block Memory Generator IP核

1. 在Vivado中打开您的项目
2. 在Flow Navigator中点击"IP Catalog"
3. 搜索"Block Memory Generator"
4. 双击打开配置界面

### 步骤2：配置IP核参数

```
Basic Tab:
- Memory Type: Simple Dual Port RAM
- Write Width: 32
- Write Depth: 65536 (或您需要的大小)
- Read Width: 32
- Read Depth: 65536
- Enable Port Type: Use ENA Pin
- Write Enable: Use WEA Pin

Port A Options:
- Write Width: 32
- Write Depth: 65536
- Enable Port Type: Use ENA Pin
- Write Enable: Use WEA Pin
- Byte Enable: Yes (重要！)

Port B Options:
- Read Width: 32
- Read Depth: 65536
- Enable Port Type: Use ENB Pin
```

### 步骤3：生成IP核

1. 点击"Generate"按钮
2. 在项目中实例化生成的IP核
3. 使用提供的`dm_32bit.v`模块

### 步骤4：修改顶层模块

在`sccomp.v`中替换原有的`dm`模块：

```verilog
// 替换原有的dm实例化
dm_32bit U_dm_32bit(
    .clk(clk),
    .DMWr(MemWrite),
    .DMType(DMType),
    .addr(dm_addr),
    .din(dm_din),
    .dout(dm_dout)
);
```

### 如果我就是懒得生成IP核会怎样

目前的实现中，如果你试图对dm输入超过表示范围的地址，dm将拒绝写入，并输出0。
