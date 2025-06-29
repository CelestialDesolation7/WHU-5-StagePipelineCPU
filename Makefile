# RISC-V Pipeline CPU Makefile
# 用于编译、仿真和测试

# 编译器设置
VLOG = vlog
VSIM = vsim
VCOM = vcom

# 源文件列表 (Srcs目录)
VERILOG_SOURCES = \
	Srcs/ctrl_encode_def.v \
	Srcs/PC.v \
	Srcs/NPC.v \
	Srcs/RF.v \
	Srcs/alu.v \
	Srcs/EXT.v \
	Srcs/dm.v \
	Srcs/pipeline_regs.v \
	Srcs/ctrl.v \
	Srcs/PipelineCPU.v \
	Srcs/sccomp.v \
	Srcs/top_module.v

# 测试文件 (Test目录)
TEST_FILES = \
	Test/simple_test.v \
	Test/pipeline_test.v \
	Test/vivado_testbench.v

# 默认目标
all: compile

# 编译所有Verilog文件
compile:
	@echo "Compiling Verilog sources..."
	$(VLOG) $(VERILOG_SOURCES)
	@echo "Compilation completed."

# 编译测试文件
compile_tests: compile
	@echo "Compiling test files..."
	$(VLOG) $(TEST_FILES)
	@echo "Test compilation completed."

# 运行简单测试
test_simple: compile_tests
	@echo "Running simple test..."
	$(VSIM) -c simple_test -do "run -all; quit"

# 运行流水线测试
test_pipeline: compile_tests
	@echo "Running pipeline test..."
	$(VSIM) -c pipeline_test -do "run -all; quit"

# 运行Vivado测试
test_vivado: compile_tests
	@echo "Running Vivado testbench..."
	$(VSIM) -c vivado_testbench -do "run -all; quit"

# 运行所有测试
test_all: test_simple test_pipeline test_vivado

# 交互式仿真
sim_interactive: compile_tests
	@echo "Starting interactive simulation..."
	$(VSIM) top_module

# 清理生成的文件
clean:
	@echo "Cleaning generated files..."
	rm -rf work/
	rm -f transcript
	rm -f vsim.wlf
	rm -f *.log

# 帮助信息
help:
	@echo "Available targets:"
	@echo "  compile        - Compile all Verilog sources from Srcs/"
	@echo "  compile_tests  - Compile sources and test files"
	@echo "  test_simple    - Run simple functional test"
	@echo "  test_pipeline  - Run pipeline functionality test"
	@echo "  test_vivado    - Run Vivado testbench"
	@echo "  test_all       - Run all tests"
	@echo "  sim_interactive- Start interactive simulation"
	@echo "  clean          - Clean generated files"
	@echo "  help           - Show this help message"
	@echo ""
	@echo "Note: This project uses ROM IP core (dist_mem_gen_0) for instruction memory"
	@echo "Make sure to add the IP core to your Vivado project"

.PHONY: all compile compile_tests test_simple test_pipeline test_vivado test_all sim_interactive clean help 