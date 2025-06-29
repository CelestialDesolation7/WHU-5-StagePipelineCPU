# RISC-V Pipeline CPU 项目更新说明

## 更新概述

本次更新主要解决了以下问题：
1. 删除了重复的冒险检测单元文件
2. 修改项目以使用ROM IP核而不是im.v模块
3. 重写了所有测试文件以适应当前设计
4. 更新了Makefile以反映新的项目结构

## 主要修改

### 1. 删除重复文件
- 删除了 `Srcs/hazard_units.v`，因为冒险检测和转发单元已经集成在 `PipelineCPU.v` 中

### 2. ROM IP核集成
- 修改了 `Srcs/sccomp.v` 以使用ROM IP核 `dist_mem_gen_0`
- 替换了原来的 `im.v` 模块实例化
- ROM IP核接口：
  - 输入：`a[6:0]` - 7位地址（对应PC[8:2]）
  - 输出：`spo[31:0]` - 32位指令数据

### 3. 测试文件重写

#### Test/simple_test.v
- 简化了测试逻辑
- 添加了指令和PC输出监控
- 测试基本CPU功能

#### Test/pipeline_test.v
- 专门测试流水线冒险处理
- 检查数据冒险和Load-Use冒险的正确处理
- 验证转发和暂停机制

#### Test/vivado_testbench.v
- 测试顶层模块的所有功能
- 包括时钟分频、暂停控制、数据显示等
- 验证开关控制的各种模式

### 4. Makefile更新
- 更新了源文件路径以反映新的目录结构
- 添加了测试文件编译步骤
- 更新了帮助信息以包含ROM IP核说明

## 项目结构

```
Project/
├── Srcs/                    # 源代码文件
│   ├── PipelineCPU.v        # 五级流水线CPU主模块
│   ├── sccomp.v            # 顶层CPU模块（使用ROM IP核）
│   ├── top_module.v        # FPGA顶层模块
│   ├── pipeline_regs.v     # 流水线寄存器
│   ├── ctrl.v              # 控制单元
│   ├── alu.v               # ALU单元
│   ├── dm.v                # 数据存储器
│   ├── RF.v                # 寄存器文件
│   ├── PC.v                # 程序计数器
│   ├── NPC.v               # 下一条PC计算
│   ├── EXT.v               # 立即数扩展
│   └── ctrl_encode_def.v   # 控制信号定义
├── Test/                   # 测试文件
│   ├── simple_test.v       # 简单功能测试
│   ├── pipeline_test.v     # 流水线冒险测试
│   └── vivado_testbench.v  # Vivado仿真测试
├── Makefile                # 编译和仿真脚本
└── PROJECT_UPDATE.md       # 本文件
```

## ROM IP核配置

### 创建ROM IP核步骤：
1. 在Vivado中打开IP Catalog
2. 搜索并选择 "Distributed Memory Generator"
3. 配置参数：
   - Component Name: `dist_mem_gen_0`
   - Memory Type: ROM
   - Data Width: 32
   - Address Width: 7
   - 加载包含RISC-V指令的COE文件

### COE文件格式示例：
```
MEMORY_INITIALIZATION_RADIX=16;
MEMORY_INITIALIZATION_VECTOR=
00000093,  // addi x1, x0, 0
00100113,  // addi x2, x0, 1
00208193,  // addi x3, x1, 2
...
```

## 使用方法

### 1. 编译和仿真
```bash
# 编译所有源文件
make compile

# 运行简单测试
make test_simple

# 运行流水线测试
make test_pipeline

# 运行所有测试
make test_all
```

### 2. Vivado项目设置
1. 创建新的Vivado项目
2. 添加所有 `Srcs/` 目录下的Verilog文件
3. 添加ROM IP核 `dist_mem_gen_0`
4. 设置顶层模块为 `top_module`
5. 添加约束文件 `Nexys4DDR_CPU.xdc`

### 3. FPGA测试
- 使用开关控制CPU运行模式
- 观察LED和七段数码管显示
- 验证流水线CPU的正确执行

## 注意事项

1. **ROM IP核依赖**：确保在Vivado项目中正确添加和配置ROM IP核
2. **指令内容**：ROM中需要包含有效的RISC-V RV32I指令
3. **地址对齐**：PC地址需要正确对齐到ROM地址空间
4. **仿真限制**：在ModelSim等仿真器中，ROM IP核可能需要特殊处理

## 故障排除

### 常见问题：
1. **ROM IP核未找到**：确保在Vivado项目中正确实例化IP核
2. **指令读取错误**：检查ROM地址映射和指令格式
3. **仿真失败**：确保所有依赖文件都已正确编译

### 调试建议：
1. 使用ILA核监控关键信号
2. 检查PC和指令读取路径
3. 验证流水线各阶段的信号传递 