# 面向验收指导文档

$$
Fuck\ The\ Prof\bf{\textit{！}}\\

Fuck\ The\ Training\ Plan\bf{\textit{！}}\\

Fuck\ The\ School\ of\ Computer\ Science\bf{\textit{！}}\\

Fuck\ The\ Wuhan\ University\ Management\ Layer\bf{\textit{！}}\\

\textit{有用的话给Arielle点个Star谢谢喵}\bf{\textit{！}}
$$

模块的包含结构如下：

```
top_module (FPGA顶层)
├── sccomp (CPU系统)
│   ├── PipelineCPU (五级流水线CPU)
│   │   ├── PC (程序计数器)
│   │   ├── NPC (下一条指令地址)
│   │   │
│   │   ├── IF_ID_Reg (IF/ID流水线寄存器)
│   │   │
│   │   ├── ctrl (控制单元)
│   │   ├── EXT (立即数扩展)
│   │   ├── RF (寄存器文件)
│   │   │
│   │   ├── ID_EX_Reg (ID/EX流水线寄存器)
│   │   │
│   │   ├── alu (算术逻辑单元)
│   │   ├── ForwardingUnit (算术逻辑单元)
│   │   │
│   │   ├── EX_MEM_Reg (EX/MEM流水线寄存器)
│   │   │
│   │   └── MEM_WB_Reg (MEM/WB流水线寄存器)
│   ├── dm (数据存储器)
│   └── im (指令存储器)
└── seg7_controller (七段数码管控制器)
```

![第5 章中央处理器| Notes](https://notes.widcard.win/_astro/jzliushuixiantonglu.B9M9F4vu_ZSBtaw.webp)

# IF

这个部分的全部硬件为：

- PC和NPC

## 定义

### PC

```verilog
module PC( clk, rst, NPC, PC );

  input              clk;
  input              rst;
  input       [31:0] NPC;
  output reg  [31:0] PC;

  always @(posedge clk, posedge rst)
    if (rst) 
      PC <= 32'h0000_0000;
    else
      PC <= NPC;
      
endmodule
```

- 这是一个形式上的模块，只是为了提供指令地址复位到0的功能。

### NPC

```verilog
`include "ctrl_encode_def.v"

module NPC(PC, NPCOp, IMM, NPC,aluout);  // next pc module
    
   input  [31:0] PC;        // pc
   input  [2:0]  NPCOp;     // next pc operation
   input  [31:0] IMM;       // immediate
	input [31:0] aluout;
   output reg [31:0] NPC;   // next pc
   
   wire [31:0] PCPLUS4;
   
   assign PCPLUS4 = PC + 4; // pc + 4
   
   always @(*) begin
      case (NPCOp)
          `NPC_PLUS4:  NPC = PCPLUS4;
          `NPC_BRANCH: NPC = PC+IMM;
          `NPC_JUMP:   NPC = PC+IMM;
		  `NPC_JALR:	NPC =aluout;
          default:     NPC = PCPLUS4;
      endcase
   end // end always
   
endmodule
```

- 这是主要的模块。
- NPCOp, IMM, aluout接收控制信号等和跳转指令有关的指示数据，PC与指令存储器相连。
- 该模块以此决定将什么指令向后发送。

## 实例化

### PC


```verilog
// IF stage
    PC pc_unit(.clk(clk), .rst(rst), .NPC(NPC_IF), .PC(PC_IF));
```

### NPC


```verilog
    // 修改NPC连接：JAL指令在ID阶段生效，JALR指令在EX阶段生效，其他跳转在EX阶段生效
    wire [2:0] npc_op_sel;
    wire [31:0] npc_imm_sel;
    
    // 选择NPC操作：JAL指令使用ID阶段的信号，其他使用EX阶段的信号
    assign npc_op_sel = (opcode_ID == `OPCODE_JAL) ? NPCOp_ID : NPCOp_EX;
    assign npc_imm_sel = (opcode_ID == `OPCODE_JAL) ? imm_ID : imm_EX;
    
    NPC npc_unit(.PC(PC_IF), .NPCOp(npc_op_sel), .IMM(npc_imm_sel), .NPC(NPC_IF), .aluout(alu_result_EX));
    assign instr_IF = instr_in;
```

## 理解过程

你的疑惑：

- 这里npc_op_sel，npc_imm_sel ，opcode_ID 三个信号是从何而来，起什么作用？

对应理解：

- 检测到JAL类型指令时，说明控制冒险发生，流水线应插入空泡。
- 事实：一个指令对应的，在其全执行过程内需要用到的控制信号，由该指令的编码唯一确定。
	- 也就是说，只要获得了指令（也就是IF阶段完成，ID阶段开始），就可以获得其控制信号值。
	- 换而言之，控制信号计算是一个查找表（LUT），即CTRL硬件。
- 回顾：为什么有控制冒险？
	- 在 IF 阶段：我们不知道当前是否该跳转，先默认 `PC+4`。
	- 在 ID 阶段：解码出 `jal`，立即数 `imm` 可以拿到，跳转地址 = `PC + imm`。
	- ⚠️ 此时，**下一条指令已经被 IF 取出（PC+4 的那一条），如果 jal 真要跳，就必须丢弃这条指令**。
	- 这个"下一条指令"现在在哪？在 IF 阶段的硬件。
	- 显然，这就是我们正在设计的硬件。
	- 它要接收流水线后端的信号，并在后端汇报JAL指令时，将当前指令标记为"被丢弃"。
- 对于JAL指令，跳转地址由指令本身直接可以给出。
	- 如果JAL引起了跳转，那么这个JAL现在位于ID。
	- 显然，我们要从流水线寄存器获得它。
- 对于JALR指令，跳转地址要由给定的寄存器和偏移量相加得到。
  - 如果JJALR引起了跳转，那么这个JAL现在位于EX。
  - 显然，我们要从流水线寄存器获得它。

# ID

我知道按空间顺序来讲，我应该先讲IF-ID寄存器，但为了便于理解，请先看这一模块硬件。

这个部分的全部硬件为：

- CTRL控制单元（ctrl）
- 冒险检测单元（HazardDetectionUnit）
- 立即数生成单元（EXT）
- 寄存器文件（RF）

## 定义

### CTRL

```verilog
`include "ctrl_encode_def.v"

module ctrl(input  [6:0] Op;       // opcode
            input  [6:0] Funct7;    // funct7
            input  [2:0] Funct3;    // funct3
            input        Zero;
            
            output       RegWrite; // control signal for register write
            output       MemWrite; // control signal for memory write
            output       MemRead;  // control signal for memory read
            output [5:0] EXTOp;    // control signal to signed extension
            output [4:0] ALUOp;    // ALU opertion
            output [2:0] NPCOp;    // next pc operation
            output       ALUSrc;   // ALU source for A
            output [2:0] DMType;   // Data memory access type
            output [1:0] GPRSel;   // general purpose register selection
            output [1:0] WDSel;    // (register) write data selection
            );
            
   // ------------------------------------------------------------
   // Instruction type encoding begins
   // Instruction type detection
   wire lui     = (Op == `OPCODE_LUI);
   wire auipc    = (Op == `OPCODE_AUIPC);
   wire jal      = (Op == `OPCODE_JAL);
   wire jalr     = (Op == `OPCODE_JALR);
   wire branch   = (Op == `OPCODE_BRANCH);
   wire load     = (Op == `OPCODE_LOAD);
   wire store    = (Op == `OPCODE_STORE);
   wire op_imm   = (Op == `OPCODE_OP_IMM);
   wire op       = (Op == `OPCODE_OP);
   

   //balabala，省略
   
   // Register arithmetic/logical instructions
   wire add      = op & (Funct3 == `FUNCT3_ADD) & (Funct7 == `FUNCT7_ADD);
   wire sub      = op & (Funct3 == `FUNCT3_SUB) & (Funct7 == `FUNCT7_SUB);
   wire sll      = op & (Funct3 == `FUNCT3_SLL);
   wire slt      = op & (Funct3 == `FUNCT3_SLT);
   wire sltu     = op & (Funct3 == `FUNCT3_SLTU);
   wire xor_op   = op & (Funct3 == `FUNCT3_XOR);
   wire srl      = op & (Funct3 == `FUNCT3_SRL) & (Funct7 == `FUNCT7_SRL);
   wire sra      = op & (Funct3 == `FUNCT3_SRA) & (Funct7 == `FUNCT7_SRA);
   wire or_op    = op & (Funct3 == `FUNCT3_OR);
   wire and_op   = op & (Funct3 == `FUNCT3_AND);

   // instruction type encoding ends
   // ------------------------------------------------------------
   

   // ------------------------------------------------------------
   // Control signals generation begins
   // Generate control signals
   assign RegWrite = lui | auipc | jal | jalr | load | addi | slti | sltiu | xori | ori | andi | slli | srli | srai | add | sub | sll | slt | sltu | xor_op | srl | sra | or_op | and_op;
   assign MemWrite = store;
   assign MemRead = load;
   assign ALUSrc = auipc | jal | jalr | load | store | addi | slti | sltiu | xori | ori | andi | slli | srli | srai;
   
   // Signed extension control
   assign EXTOp[5] = slli | srli | srai;  // ITYPE_SHAMT
   assign EXTOp[4] = addi | slti | sltiu | xori | ori | andi | jalr;  // ITYPE
   assign EXTOp[3] = store;  // STYPE
   assign EXTOp[2] = branch;  // BTYPE
   assign EXTOp[1] = lui | auipc;  // UTYPE
   assign EXTOp[0] = jal;  // JTYPE
   
   // Write data selection
   assign WDSel[1] = jal | jalr;
   assign WDSel[0] = load;
   
   // Next PC operation
   assign NPCOp[2] = jalr;
   assign NPCOp[1] = jal;
   assign NPCOp[0] = (beq & Zero) | (bne & ~Zero) | (blt & ~Zero) | (bge & Zero) | (bltu & ~Zero) | (bgeu & Zero);
   
   // ALU operation encoding
   assign ALUOp[4] = lui | auipc;
   assign ALUOp[3] = (addi | add) | (slti | slt) | (sltiu | sltu) | (xori | xor_op) | (ori | or_op) | (andi | and_op);
   assign ALUOp[2] = (slli | sll) | (srli | srl) | (srai | sra) | (xori | xor_op) | (ori | or_op) | (andi | and_op);
   assign ALUOp[1] = (sub | srl | sra) | (slti | slt) | (sltiu | sltu) | (xori | xor_op) | (ori | or_op) | (andi | and_op) | (beq | bne) | (blt | bge) | (bltu | bgeu);
   assign ALUOp[0] = (addi | add) | (sub) | (slti | slt) | (sltiu | sltu) | (xori | xor_op) | (ori | or_op) | (andi | and_op) | (slli | sll) | (srli | srl) | (srai | sra) | (load | store) | jalr;
   
   // Data memory access type
   assign DMType[2] = lbu | lhu;
   assign DMType[1] = lh | lhu | sh;
   assign DMType[0] = lb | lbu | sb;

   // Control signals generation ends
   // ------------------------------------------------------------
   
   // Register selection
   assign GPRSel = `GPRSel_RD;

endmodule

```

- 该模块生成控制信号，内部可以分为：

	- 指令字段分离为更直观的wire信号，以便生成控制信号。
	- 正式计算控制信号。

- 控制信号标准编码如下：

	- NPCOp — Next PC Operation（跳转控制）

		| 编码 (`NPCOp`) | 三位二进制 | 含义                    | 适用指令                                   |
	|----------------|------------|-------------------------|--------------------------------------------|
	| `000`          | `3'b000`   | 顺序执行：PC + 4        | 非跳转指令                                 |
	| `001`          | `3'b001`   | 条件分支跳转            | `beq`, `bne`, `blt`, `bge`, `bltu`, `bgeu` |
	| `010`          | `3'b010`   | 无条件跳转（jal）       | `jal`                                      |
	| `100`          | `3'b100`   | 寄存器跳转（jalr）      | `jalr`                                     |

	- EXTOp — Immediate Extension Type（立即数类型）
	
	  | 位号     | 含义                       | 适用指令类型 / 指令                                    |
		| -------- | -------------------------- | ------------------------------------------------------ |
		| EXTOp[5] | I-type 移位立即数（shamt） | `slli`, `srli`, `srai`                                 |
		| EXTOp[4] | I-type 普通立即数          | `addi`, `slti`, `sltiu`, `xori`, `ori`, `andi`, `jalr` |
		| EXTOp[3] | S-type 存储型立即数        | `sw`, `sh`, `sb`                                       |
		| EXTOp[2] | B-type 分支型立即数        | `beq`, `bne`, `blt`, `bge`, `bltu`, `bgeu`             |
		| EXTOp[1] | U-type 高位立即数          | `lui`, `auipc`                                         |
		| EXTOp[0] | J-type 跳转立即数          | `jal`                                                  |

	- WDSel — Write Data Select（寄存器写回数据来源）
	  | 编码 (`WDSel`) | 二位二进制 | 含义                         | 适用指令       |
	  |----------------|------------|------------------------------|----------------|
	  | `00`           | `2'b00`    | ALU 计算结果                 | 普通算术/逻辑指令 |
	  | `01`           | `2'b01`    | 数据存储器读取数据（DM）     | `load`         |
	  | `10`           | `2'b10`    | 跳转返回地址（PC + 4）       | `jal`, `jalr`  |
	  | `11`           | `2'b11`    | 保留/未定义                  | —              |

	
	- ALUOp — ALU 操作选择（位掩码风格）
	  | 位号       | 含义                                     | 典型指令                                  |
	  |------------|------------------------------------------|-------------------------------------------|
	  | ALUOp[4]   | 特殊类型（lui/auipc）                     | `lui`, `auipc`                            |
	  | ALUOp[3]   | 通用运算：加、比较、逻辑（or/xor/and）   | `addi`, `slti`, `ori`, `andi`, `xor`, 等  |
	  | ALUOp[2]   | 位移类操作                                | `slli`, `srli`, `srai`, `sll`, `srl`, `sra` |
	  | ALUOp[1]   | 减法、比较、分支                          | `sub`, `slt`, `beq`, `bne`, `blt`, 等     |
	  | ALUOp[0]   | 加法/地址计算类                           | `addi`, `add`, `load`, `store`, `jalr`    |

	
	- DMType — 数据存储类型（字节对齐宽度）
	  
    | 编码 (`DMType`) | 三位二进制 | 含义                | 典型指令             |
    |----------------|------------|---------------------|----------------------|
    | `000`          | `3'b000`   | word（32-bit）      | `lw`, `sw`           |
    | `001`          | `3'b001`   | byte（8-bit）       | `lb`, `lbu`, `sb`    |
    | `010`          | `3'b010`   | half-word（16-bit） | `lh`, `lhu`, `sh`    |
    
  - GPRSel — 通用目的寄存器选择，在risc-v中没有意义

### HazardDetectionUnit

```verilog
// 危险检测单元 - 检测和处理流水线中的数据冒险
// 该模块实现了Load-Use冒险检测，当检测到冒险时会暂停流水线
module HazardDetectionUnit(
    input [4:0] rs1_ID, rs2_ID,    // ID阶段的源寄存器地址
    input [4:0] rd_EX, rd_MEM,     // EX和MEM阶段的目标寄存器地址
    input MemRead_EX,              // EX阶段是否为Load指令
    input RegWrite_EX, RegWrite_MEM, // EX和MEM阶段是否写寄存器
    output reg stall_IF,           // 暂停IF阶段的信号
    output reg flush_IF,           // 清空IF阶段的信号
    output reg flush_ID            // 清空ID阶段的信号
);
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
        else begin
            // 没有冒险时，正常执行
            stall_IF = 1'b0;  // 不暂停IF阶段
            flush_IF = 1'b0;  // 不清空IF阶段
            flush_ID = 1'b0;  // 不清空ID阶段
        end
    end
endmodule
```

停顿旨在解决由ld加载类指令引起的（不能由前递解决的）<font color='#f35336'>**载入-使用型数据冒险**</font>，以及beq类分支指令引起的特殊的**控制冒险**。

- 在不引入指令顺序调整的时候，解决此类冒险需要依赖一个进行冒险检测，并插入空指令（即停顿）的控制单元。
	检查条件同时成立时需要停顿：
	- 指令是ld类加载指令（唯一需要读内存的指令）。
		信号线表示为ID/EX.MemRead有效。
	- 在EX阶段即将被执行的这一ld类加载指令，其"目标寄存器"与"ID阶段即将解码然后访问寄存器"的指令指定的某个源操作数一致
		（即发生冲突）
		信号线表示为`(ID/EX.RegisterRd=IF/ID.RegisterRs1 or ID/EX.RegisterRd=IF/ID.RegisterRs2)`。
- 该单元物理层面的操作对象，是ID阶段流水线硬件的前后两个流水线寄存器。
	其通过信号线检测紧邻的两条指令是否引发了载入-使用型数据冒险。
- 为了在检测冒险后实现停顿，该控制单元需要同时禁止当前即将开始执行IF和ID阶段的指令继续向后步进执行。
- 进一步而言，需要禁止PC寄存器，IF/ID流水线寄存器发生状态改变。
- 为了阻止这两个寄存器发生状态改变，一方面需要为ID/EX流水线寄存器的信号接收路径加入一个Mux，由冒险检测单元控制路径选择。
	如果有冒险，选择 0 路径传入ID/EX流水线寄存器中所有的7个控制信号位以阻止这一指令修改任何寄存器的值。
	这一写控制信号称为IF/IDWrite信号。
- 另一方面，为PC的写入引入一条使能线，由冒险检测单元控制。
	通过这条线的控制信号冒险检测单元在一个周期的时长内阻止PC修改自身。
	这一写控制信号称为PCWrite信号。
- 当前已经被错误取出的指令，将因为RegWrite和MemWrite信号均为0而不能够修改处理器状态。
	其自EX阶段开始的，流水线后半部分执行的操作将会使用错误的数据，但其操作本身已经不再具有任何意义，成为空指令。
	PC在这个周期内也被锁定，以确保下个周期重复地取出该指令重新执行。
- 随后，流水线可以在本次冒险以后正常运行。完成上述功能的硬件结构示意图如下。

![image-20250629161928653](assets/image-20250629161928653.png)

### EXT

```verilog
`include "ctrl_encode_def.v"
module EXT( 
	input   [4:0] 	iimm_shamt,
    input	[11:0]			iimm, //instr[31:20], 12 bits
	input	[11:0]			simm, //instr[31:25, 11:7], 12 bits
	input	[11:0]			bimm, //instrD[31], instrD[7], instrD[30:25], instrD[11:8], 12 bits
	input	[19:0]			uimm,
	input	[19:0]			jimm,
	input	[5:0]			EXTOp,

	output	reg [31:0] 	       immout);
   
always  @(*)
	 case (EXTOp)
		`EXT_CTRL_ITYPE_SHAMT:   immout<={27'b0,iimm_shamt[4:0]};
		`EXT_CTRL_ITYPE:	immout <= {{20{iimm[11]}}, iimm[11:0]};
		`EXT_CTRL_STYPE:	immout <= {{20{simm[11]}}, simm[11:0]};
		`EXT_CTRL_BTYPE:    immout <= {{20{bimm[11]}}, bimm[11:0]};
		`EXT_CTRL_UTYPE:	immout <= {uimm[19:0], 12'b0};
		`EXT_CTRL_JTYPE:	immout <= {{12{jimm[19]}}, jimm[19:0]};
		default:	        immout <= 32'b0;
	 endcase
 
endmodule
```









### RF

```verilog
  module RF(   input         clk, 
               input         rst,
               input         RFWr, 
               input  [4:0]  A1, A2, A3, 
               input  [31:0] WD, 
               output [31:0] RD1, RD2);

  reg [31:0] rf[31:0];

  integer i;

  always @(posedge clk, posedge rst)
    if (rst) begin    //  reset
      for (i=1; i<32; i=i+1)
        rf[i] <= 0; //  i;
    end
      
    else 
      if (RFWr) begin
        rf[A3] <= WD;
      end
    

  assign RD1 = (A1 != 0) ? rf[A1] : 0;
  assign RD2 = (A2 != 0) ? rf[A2] : 0;
  //assign reg_data = (reg_sel != 0) ? rf[reg_sel] : 0; 

endmodule 
```



## 实例化

### CTRL

```verilog
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
```



### HazardDetectionUnit

```verilog

```



### EXT

```verilog

```



### RF

```verilog

```



## 理解过程











# EXE

这个部分的全部硬件为：

- ALU（算术逻辑单元）
- 前递单元（ForwardingUnit）
- ID-EX流水线寄存器

## 定义

### ALU

```verilog
`include "ctrl_encode_def.v"

module alu( input  [31:0] A, B,       // ALU 32-bit Inputs                 
            input  [4:0]  ALUOp,      // ALU Operation
            output reg [31:0] Result, // ALU 32-bit Output
            output Zero);             // Zero Flag

    wire [31:0] shamt;
    assign shamt = {27'b0, B[4:0]};  // 移位量，取B的低5位
    
    always @(*) begin
        case (ALUOp)
            // 特殊类型指令
            `ALU_CTRL_LUI:   Result = B;                    // lui: 将立即数加载到高位
            `ALU_CTRL_AUIPC: Result = A + B;                // auipc: PC + 立即数
            
            // 算术运算
            `ALU_CTRL_ADD:   Result = A + B;                // add/addi: 加法
            `ALU_CTRL_SUB:   Result = A - B;                // sub: 减法
            `ALU_CTRL_SLT:   Result = ($signed(A) < $signed(B)) ? 32'd1 : 32'd0;  // slt/slti: 有符号比较
            `ALU_CTRL_SLTU:  Result = (A < B) ? 32'd1 : 32'd0;                    // sltu/sltiu: 无符号比较
            
            // 逻辑运算
            `ALU_CTRL_XOR:   Result = A ^ B;                // xor/xori: 异或
            `ALU_CTRL_OR:    Result = A | B;                // or/ori: 或
            `ALU_CTRL_AND:   Result = A & B;                // and/andi: 与
            
            // 移位运算
            `ALU_CTRL_SLL:   Result = A << shamt;           // sll/slli: 逻辑左移
            `ALU_CTRL_SRL:   Result = A >> shamt;           // srl/srli: 逻辑右移
            `ALU_CTRL_SRA:   Result = $signed(A) >>> shamt; // sra/srai: 算术右移
            
            // 分支比较
            `ALU_CTRL_BEQ:   Result = (A == B) ? 32'd1 : 32'd0;  // beq: 相等比较
            `ALU_CTRL_BNE:   Result = (A != B) ? 32'd1 : 32'd0;  // bne: 不等比较
            `ALU_CTRL_BLT:   Result = ($signed(A) < $signed(B)) ? 32'd1 : 32'd0;  // blt: 有符号小于
            `ALU_CTRL_BGE:   Result = ($signed(A) >= $signed(B)) ? 32'd1 : 32'd0; // bge: 有符号大于等于
            `ALU_CTRL_BLTU:  Result = (A < B) ? 32'd1 : 32'd0;                    // bltu: 无符号小于
            `ALU_CTRL_BGEU:  Result = (A >= B) ? 32'd1 : 32'd0;                   // bgeu: 无符号大于等于
            
            // 地址计算
            `ALU_CTRL_ADDR:  Result = A + B;                // load/store: 地址计算
            
            default:         Result = 32'h0;
        endcase
    end
    
    assign Zero = (Result == 32'h0);  // 零标志位
    
endmodule
```

- 该模块实现了RISC-V RV32I指令集的所有ALU操作
- 支持算术运算（加、减、比较）、逻辑运算（与、或、异或）、移位运算（左移、逻辑右移、算术右移）
- 支持分支指令的比较操作
- 支持地址计算（用于load/store指令）

### ForwardingUnit

```verilog
// 前递单元 - 检测和处理数据冒险
// 当检测到数据冒险时，从流水线后端前递数据到EX阶段
module ForwardingUnit(
    input [4:0] rs1_EX, rs2_EX,      // EX阶段的源寄存器地址
    input [4:0] rd_MEM, rd_WB,       // MEM和WB阶段的目标寄存器地址
    input RegWrite_MEM, RegWrite_WB, // MEM和WB阶段是否写寄存器
    output reg [1:0] ForwardA,       // rs1的前递选择信号
    output reg [1:0] ForwardB        // rs2的前递选择信号
);
    always @(*) begin
        // 前递A（rs1）的逻辑
        if (RegWrite_MEM && (rd_MEM != 5'b0) && (rd_MEM == rs1_EX)) begin
            ForwardA = 2'b10;  // 从MEM阶段前递
        end
        else if (RegWrite_WB && (rd_WB != 5'b0) && (rd_WB == rs1_EX)) begin
            ForwardA = 2'b01;  // 从WB阶段前递
        end
        else begin
            ForwardA = 2'b00;  // 不使用前递
        end
        
        // 前递B（rs2）的逻辑
        if (RegWrite_MEM && (rd_MEM != 5'b0) && (rd_MEM == rs2_EX)) begin
            ForwardB = 2'b10;  // 从MEM阶段前递
        end
        else if (RegWrite_WB && (rd_WB != 5'b0) && (rd_WB == rs2_EX)) begin
            ForwardB = 2'b01;  // 从WB阶段前递
        end
        else begin
            ForwardB = 2'b00;  // 不使用前递
        end
    end
endmodule
```

- 该模块检测数据冒险并生成前递控制信号
- 当EX阶段的源寄存器与MEM或WB阶段的目标寄存器相同时，会产生数据冒险
- 通过前递机制，可以将最新的数据直接传递给EX阶段，避免流水线暂停

### ID_EX_Reg

```verilog
module ID_EX_Reg(
    input clk,
    input rst,
    input flush,
    // 控制信号
    input RegWrite_ID, MemWrite_ID, MemRead_ID,
    input [4:0] ALUOp_ID,
    input ALUSrc_ID,
    input [2:0] DMType_ID,
    input [1:0] GPRSel_ID, WDSel_ID,
    // 数据信号
    input [31:0] PC_ID, rs1_data_ID, rs2_data_ID, imm_ID,
    input [4:0] rs1_ID, rs2_ID, rd_ID,
    input [6:0] opcode_ID,
    // 输出控制信号
    output reg RegWrite_EX, MemWrite_EX, MemRead_EX,
    output reg [4:0] ALUOp_EX,
    output reg ALUSrc_EX,
    output reg [2:0] DMType_EX,
    output reg [1:0] GPRSel_EX, WDSel_EX,
    // 输出数据信号
    output reg [31:0] PC_EX, rs1_data_EX, rs2_data_EX, imm_EX,
    output reg [4:0] rs1_EX, rs2_EX, rd_EX,
    output reg [6:0] opcode_EX
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 控制信号清零
            RegWrite_EX <= 1'b0;
            MemWrite_EX <= 1'b0;
            MemRead_EX <= 1'b0;
            ALUOp_EX <= 5'b0;
            ALUSrc_EX <= 1'b0;
            DMType_EX <= 3'b0;
            GPRSel_EX <= 2'b0;
            WDSel_EX <= 2'b0;
            // 数据信号清零
            PC_EX <= 32'h0;
            rs1_data_EX <= 32'h0;
            rs2_data_EX <= 32'h0;
            imm_EX <= 32'h0;
            rs1_EX <= 5'b0;
            rs2_EX <= 5'b0;
            rd_EX <= 5'b0;
            opcode_EX <= 7'b0;
        end
        else if (flush) begin
            // 清空时清零所有信号
            RegWrite_EX <= 1'b0;
            MemWrite_EX <= 1'b0;
            MemRead_EX <= 1'b0;
            ALUOp_EX <= 5'b0;
            ALUSrc_EX <= 1'b0;
            DMType_EX <= 3'b0;
            GPRSel_EX <= 2'b0;
            WDSel_EX <= 2'b0;
            PC_EX <= 32'h0;
            rs1_data_EX <= 32'h0;
            rs2_data_EX <= 32'h0;
            imm_EX <= 32'h0;
            rs1_EX <= 5'b0;
            rs2_EX <= 5'b0;
            rd_EX <= 5'b0;
            opcode_EX <= 7'b0;
        end
        else begin
            // 正常传递信号
            RegWrite_EX <= RegWrite_ID;
            MemWrite_EX <= MemWrite_ID;
            MemRead_EX <= MemRead_ID;
            ALUOp_EX <= ALUOp_ID;
            ALUSrc_EX <= ALUSrc_ID;
            DMType_EX <= DMType_ID;
            GPRSel_EX <= GPRSel_ID;
            WDSel_EX <= WDSel_ID;
            PC_EX <= PC_ID;
            rs1_data_EX <= rs1_data_ID;
            rs2_data_EX <= rs2_data_ID;
            imm_EX <= imm_ID;
            rs1_EX <= rs1_ID;
            rs2_EX <= rs2_ID;
            rd_EX <= rd_ID;
            opcode_EX <= opcode_ID;
        end
    end
endmodule
```

## 实例化

### ALU

```verilog
// EX阶段
    // 前递单元
    ForwardingUnit forwarding_unit(
        .rs1_EX(rs1_EX),
        .rs2_EX(rs2_EX),
        .rd_MEM(rd_MEM),
        .rd_WB(rd_WB),
        .RegWrite_MEM(RegWrite_MEM),
        .RegWrite_WB(RegWrite_WB),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );
    
    // 前递数据选择
    wire [31:0] rs1_data_forward, rs2_data_forward;
    assign rs1_data_forward = (ForwardA == 2'b10) ? alu_result_MEM :
                              (ForwardA == 2'b01) ? wb_data :
                              rs1_data_EX;
    assign rs2_data_forward = (ForwardB == 2'b10) ? alu_result_MEM :
                              (ForwardB == 2'b01) ? wb_data :
                              rs2_data_EX;
    
    // ALU操作数选择
    wire [31:0] alu_operand_a, alu_operand_b;
    assign alu_operand_a = rs1_data_forward;
    assign alu_operand_b = ALUSrc_EX ? imm_EX : rs2_data_forward;
    
    // ALU
    alu alu_unit(
        .A(alu_operand_a),
        .B(alu_operand_b),
        .ALUOp(ALUOp_EX),
        .Result(alu_result_EX),
        .Zero(Zero_EX)
    );
```

### ForwardingUnit

### ID_EX_Reg

```verilog
// ID-EX流水线寄存器
    ID_EX_Reg id_ex_reg(
        .clk(clk),
        .rst(rst),
        .flush(flush_ID),
        .RegWrite_ID(RegWrite_ID),
        .MemWrite_ID(MemWrite_ID),
        .MemRead_ID(MemRead_ID),
        .ALUOp_ID(ALUOp_ID),
        .ALUSrc_ID(ALUSrc_ID),
        .DMType_ID(DMType_ID),
        .GPRSel_ID(GPRSel_ID),
        .WDSel_ID(WDSel_ID),
        .PC_ID(PC_ID),
        .rs1_data_ID(rs1_data_ID),
        .rs2_data_ID(rs2_data_ID),
        .imm_ID(imm_ID),
        .rs1_ID(rs1_ID),
        .rs2_ID(rs2_ID),
        .rd_ID(rd_ID),
        .opcode_ID(opcode_ID),
        .RegWrite_EX(RegWrite_EX),
        .MemWrite_EX(MemWrite_EX),
        .MemRead_EX(MemRead_EX),
        .ALUOp_EX(ALUOp_EX),
        .ALUSrc_EX(ALUSrc_EX),
        .DMType_EX(DMType_EX),
        .GPRSel_EX(GPRSel_EX),
        .WDSel_EX(WDSel_EX),
        .PC_EX(PC_EX),
        .rs1_data_EX(rs1_data_EX),
        .rs2_data_EX(rs2_data_EX),
        .imm_EX(imm_EX),
        .rs1_EX(rs1_EX),
        .rs2_EX(rs2_EX),
        .rd_EX(rd_EX),
        .opcode_EX(opcode_EX)
    );
```

## 理解过程

你的疑惑：

- 前递单元是如何工作的？为什么需要前递？

对应理解：

- 数据冒险：当指令需要使用前面指令的结果时，由于流水线的并行执行，数据可能还没有写回寄存器
- 前递机制：将流水线后端（MEM、WB阶段）的计算结果直接传递给EX阶段，避免等待
- 前递优先级：MEM阶段的结果优先于WB阶段，因为MEM阶段的数据更新
- 前递条件：只有当目标寄存器不为x0且确实要写寄存器时才进行前递

# MEM

这个部分的全部硬件为：

- 数据存储器（dm）
- EX-MEM流水线寄存器

## 定义

### dm

```verilog
`include "ctrl_encode_def.v"

module dm( input         clk, rst,
           input         MemWrite, MemRead,
           input  [31:0] addr, wdata,
           input  [2:0]  DMType,
           output [31:0] rdata);

    reg [31:0] dm[1023:0];  // 1KB数据存储器
    integer i;
    
    // 复位时清零存储器
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            for (i = 0; i < 1024; i = i + 1) begin
                dm[i] <= 32'h0;
            end
        end
        else if (MemWrite) begin
            case (DMType)
                `DM_CTRL_WORD:  dm[addr[11:2]] <= wdata;                    // sw: 写32位
                `DM_CTRL_HALF:  dm[addr[11:2]][15:0] <= wdata[15:0];       // sh: 写16位
                `DM_CTRL_BYTE:  dm[addr[11:2]][7:0] <= wdata[7:0];         // sb: 写8位
                default:        dm[addr[11:2]] <= wdata;
            endcase
        end
    end
    
    // 读取数据
    wire [31:0] raw_data = dm[addr[11:2]];
    wire [1:0] byte_offset = addr[1:0];
    wire [15:0] half_offset = addr[1];
    
    assign rdata = MemRead ? 
        case (DMType)
            `DM_CTRL_WORD:  raw_data;                    // lw: 读32位
            `DM_CTRL_HALF:  half_offset ? 
                {{16{raw_data[31]}}, raw_data[31:16]} :  // lh: 读高16位
                {{16{raw_data[15]}}, raw_data[15:0]};    // lh: 读低16位
            `DM_CTRL_HALF_U: half_offset ? 
                {16'h0, raw_data[31:16]} :               // lhu: 读高16位（无符号）
                {16'h0, raw_data[15:0]};                 // lhu: 读低16位（无符号）
            `DM_CTRL_BYTE:  byte_offset == 2'b11 ? 
                {{24{raw_data[31]}}, raw_data[31:24]} :  // lb: 读最高8位
                byte_offset == 2'b10 ? 
                {{24{raw_data[23]}}, raw_data[23:16]} :  // lb: 读次高8位
                byte_offset == 2'b01 ? 
                {{24{raw_data[15]}}, raw_data[15:8]} :   // lb: 读次低8位
                {{24{raw_data[7]}}, raw_data[7:0]};      // lb: 读最低8位
            `DM_CTRL_BYTE_U: byte_offset == 2'b11 ? 
                {24'h0, raw_data[31:24]} :               // lbu: 读最高8位（无符号）
                byte_offset == 2'b10 ? 
                {24'h0, raw_data[23:16]} :               // lbu: 读次高8位（无符号）
                byte_offset == 2'b01 ? 
                {24'h0, raw_data[15:8]} :                // lbu: 读次低8位（无符号）
                {24'h0, raw_data[7:0]};                  // lbu: 读最低8位（无符号）
            default: raw_data;
        endcase : 32'h0;

endmodule
```

- 该模块实现了RISC-V的数据存储器
- 支持字节（8位）、半字（16位）、字（32位）的读写操作
- 支持有符号和无符号的加载操作
- 地址对齐：字访问要求4字节对齐，半字访问要求2字节对齐

### EX_MEM_Reg

```verilog
module EX_MEM_Reg(
    input clk,
    input rst,
    input flush,
    // 控制信号
    input RegWrite_EX, MemWrite_EX, MemRead_EX,
    input [2:0] DMType_EX,
    input [1:0] GPRSel_EX, WDSel_EX,
    // 数据信号
    input [31:0] alu_result_EX, rs2_data_EX,
    input [4:0] rd_EX,
    // 输出控制信号
    output reg RegWrite_MEM, MemWrite_MEM, MemRead_MEM,
    output reg [2:0] DMType_MEM,
    output reg [1:0] GPRSel_MEM, WDSel_MEM,
    // 输出数据信号
    output reg [31:0] alu_result_MEM, rs2_data_MEM,
    output reg [4:0] rd_MEM
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 控制信号清零
            RegWrite_MEM <= 1'b0;
            MemWrite_MEM <= 1'b0;
            MemRead_MEM <= 1'b0;
            DMType_MEM <= 3'b0;
            GPRSel_MEM <= 2'b0;
            WDSel_MEM <= 2'b0;
            // 数据信号清零
            alu_result_MEM <= 32'h0;
            rs2_data_MEM <= 32'h0;
            rd_MEM <= 5'b0;
        end
        else if (flush) begin
            // 清空时清零所有信号
            RegWrite_MEM <= 1'b0;
            MemWrite_MEM <= 1'b0;
            MemRead_MEM <= 1'b0;
            DMType_MEM <= 3'b0;
            GPRSel_MEM <= 2'b0;
            WDSel_MEM <= 2'b0;
            alu_result_MEM <= 32'h0;
            rs2_data_MEM <= 32'h0;
            rd_MEM <= 5'b0;
        end
        else begin
            // 正常传递信号
            RegWrite_MEM <= RegWrite_EX;
            MemWrite_MEM <= MemWrite_EX;
            MemRead_MEM <= MemRead_EX;
            DMType_MEM <= DMType_EX;
            GPRSel_MEM <= GPRSel_EX;
            WDSel_MEM <= WDSel_EX;
            alu_result_MEM <= alu_result_EX;
            rs2_data_MEM <= rs2_data_EX;
            rd_MEM <= rd_EX;
        end
    end
endmodule
```

## 实例化

### dm

```verilog
// MEM阶段
    // 数据存储器
    dm dm_unit(
        .clk(clk),
        .rst(rst),
        .MemWrite(MemWrite_MEM),
        .MemRead(MemRead_MEM),
        .addr(alu_result_MEM),
        .wdata(rs2_data_MEM),
        .DMType(DMType_MEM),
        .rdata(dm_data_MEM)
    );
```

### EX_MEM_Reg

```verilog
// EX-MEM流水线寄存器
    EX_MEM_Reg ex_mem_reg(
        .clk(clk),
        .rst(rst),
        .flush(1'b0),  // EX-MEM阶段通常不清空
        .RegWrite_EX(RegWrite_EX),
        .MemWrite_EX(MemWrite_EX),
        .MemRead_EX(MemRead_EX),
        .DMType_EX(DMType_EX),
        .GPRSel_EX(GPRSel_EX),
        .WDSel_EX(WDSel_EX),
        .alu_result_EX(alu_result_EX),
        .rs2_data_EX(rs2_data_EX),
        .rd_EX(rd_EX),
        .RegWrite_MEM(RegWrite_MEM),
        .MemWrite_MEM(MemWrite_MEM),
        .MemRead_MEM(MemRead_MEM),
        .DMType_MEM(DMType_MEM),
        .GPRSel_MEM(GPRSel_MEM),
        .WDSel_MEM(WDSel_MEM),
        .alu_result_MEM(alu_result_MEM),
        .rs2_data_MEM(rs2_data_MEM),
        .rd_MEM(rd_MEM)
    );
```

## 理解过程

你的疑惑：

- 数据存储器是如何处理不同宽度的访问的？

对应理解：

- 字节对齐：RISC-V要求字访问4字节对齐，半字访问2字节对齐
- 地址计算：使用地址的高位作为存储器索引，低位作为字节偏移
- 符号扩展：有符号加载（lb、lh）进行符号扩展，无符号加载（lbu、lhu）进行零扩展
- 写操作：只修改目标字节/半字，保持其他位不变

# WB

这个部分的全部硬件为：

- MEM-WB流水线寄存器
- 写回数据选择逻辑

## 定义

### MEM_WB_Reg

```verilog
module MEM_WB_Reg(
    input clk,
    input rst,
    input flush,
    // 控制信号
    input RegWrite_MEM,
    input [1:0] GPRSel_MEM, WDSel_MEM,
    // 数据信号
    input [31:0] alu_result_MEM, dm_data_MEM, PC_MEM,
    input [4:0] rd_MEM,
    // 输出控制信号
    output reg RegWrite_WB,
    output reg [1:0] GPRSel_WB, WDSel_WB,
    // 输出数据信号
    output reg [31:0] alu_result_WB, dm_data_WB, PC_WB,
    output reg [4:0] rd_WB
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // 控制信号清零
            RegWrite_WB <= 1'b0;
            GPRSel_WB <= 2'b0;
            WDSel_WB <= 2'b0;
            // 数据信号清零
            alu_result_WB <= 32'h0;
            dm_data_WB <= 32'h0;
            PC_WB <= 32'h0;
            rd_WB <= 5'b0;
        end
        else if (flush) begin
            // 清空时清零所有信号
            RegWrite_WB <= 1'b0;
            GPRSel_WB <= 2'b0;
            WDSel_WB <= 2'b0;
            alu_result_WB <= 32'h0;
            dm_data_WB <= 32'h0;
            PC_WB <= 32'h0;
            rd_WB <= 5'b0;
        end
        else begin
            // 正常传递信号
            RegWrite_WB <= RegWrite_MEM;
            GPRSel_WB <= GPRSel_MEM;
            WDSel_WB <= WDSel_MEM;
            alu_result_WB <= alu_result_MEM;
            dm_data_WB <= dm_data_MEM;
            PC_WB <= PC_MEM;
            rd_WB <= rd_MEM;
        end
    end
endmodule
```

## 实例化

### MEM_WB_Reg

```verilog
// MEM-WB流水线寄存器
    MEM_WB_Reg mem_wb_reg(
        .clk(clk),
        .rst(rst),
        .flush(1'b0),  // MEM-WB阶段通常不清空
        .RegWrite_MEM(RegWrite_MEM),
        .GPRSel_MEM(GPRSel_MEM),
        .WDSel_MEM(WDSel_MEM),
        .alu_result_MEM(alu_result_MEM),
        .dm_data_MEM(dm_data_MEM),
        .PC_MEM(PC_MEM),
        .rd_MEM(rd_MEM),
        .RegWrite_WB(RegWrite_WB),
        .GPRSel_WB(GPRSel_WB),
        .WDSel_WB(WDSel_WB),
        .alu_result_WB(alu_result_WB),
        .dm_data_WB(dm_data_WB),
        .PC_WB(PC_WB),
        .rd_WB(rd_WB)
    );
    
    // 写回数据选择
    wire [31:0] wb_data;
    assign wb_data = (WDSel_WB == 2'b00) ? alu_result_WB :  // ALU结果
                     (WDSel_WB == 2'b01) ? dm_data_WB :      // 存储器数据
                     (WDSel_WB == 2'b10) ? PC_WB :           // 跳转返回地址
                     32'h0;                                   // 默认值
```

## 理解过程

你的疑惑：

- 写回阶段是如何选择正确的数据的？

对应理解：

- 写回数据来源：ALU计算结果、存储器读取数据、跳转返回地址
- 选择信号：WDSel控制信号决定使用哪个数据源
- 寄存器写使能：RegWrite信号控制是否真正写入寄存器
- 目标寄存器：rd指定要写入的寄存器地址

# IF-ID流水线寄存器

```verilog
module IF_ID_Reg(
    input clk,
    input rst,
    input flush,
    input stall,
    input [31:0] PC_in,
    input [31:0] instr_in,
    output reg [31:0] PC_out,
    output reg [31:0] instr_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC_out <= 32'h0;
            instr_out <= 32'h0;
        end
        else if (flush) begin
            PC_out <= 32'h0;
            instr_out <= 32'h0;
        end
        else if (!stall) begin
            PC_out <= PC_in;
            instr_out <= instr_in;
        end
    end
endmodule
```

## 实例化

```verilog
// IF-ID流水线寄存器
    IF_ID_Reg if_id_reg(
        .clk(clk),
        .rst(rst),
        .flush(flush_IF),
        .stall(stall_IF),
        .PC_in(PC_IF),
        .instr_in(instr_IF),
        .PC_out(PC_ID),
        .instr_out(instr_ID)
    );
```

## 理解过程

你的疑惑：

- 流水线寄存器是如何处理暂停和清空的？

对应理解：

- 暂停（stall）：当stall=1时，保持当前值不变，防止新数据进入
- 清空（flush）：当flush=1时，将输出清零，插入气泡
- 正常传递：当stall=0且flush=0时，正常传递输入到输出
- 优先级：rst > flush > stall > 正常传递

# 总结

## 流水线阶段总结

| 阶段 | 主要功能 | 关键模块 | 控制信号 |
|------|----------|----------|----------|
| IF | 取指令 | PC, NPC | NPCOp |
| ID | 指令解码 | CTRL, RF, EXT | RegWrite, MemWrite, ALUOp |
| EX | 执行 | ALU, ForwardingUnit | ALUSrc, ForwardA/B |
| MEM | 存储器访问 | dm | MemRead, MemWrite, DMType |
| WB | 写回 | - | WDSel, RegWrite |

## 冒险处理总结

| 冒险类型 | 检测单元 | 处理方法 | 影响阶段 |
|----------|----------|----------|----------|
| 数据冒险 | ForwardingUnit | 前递 | EX |
| Load-Use冒险 | HazardDetectionUnit | 暂停 | IF, ID |
| 控制冒险 | CTRL | 清空 | IF, ID |

## 关键设计要点

1. **模块化设计**：每个流水线阶段独立设计，便于理解和调试
2. **冒险处理**：完整的数据冒险和控制冒险处理机制
3. **控制信号传递**：控制信号随数据在流水线中传递
4. **前递机制**：有效减少数据冒险导致的性能损失
5. **存储器对齐**：正确处理不同宽度的存储器访问

这个五级流水线CPU实现了RISC-V RV32I指令集的完整功能，通过流水线技术提高了指令执行效率，通过冒险处理机制保证了执行正确性。
