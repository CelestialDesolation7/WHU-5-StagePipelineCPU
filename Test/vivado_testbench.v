`timescale 1ns / 1ps

// Vivado仿真测试文件 - 测试修改后的顶层模块
module vivado_testbench();

    // 测试信号
    reg clk;
    reg rstn;
    reg [15:0] sw_i;
    wire [15:0] led_o;
    wire [7:0] disp_seg_o;
    wire [7:0] disp_an_o;
    
    // 实例化顶层模块
    top_module dut(
        .clk(clk),
        .rstn(rstn),
        .sw_i(sw_i),
        .led_o(led_o),
        .disp_seg_o(disp_seg_o),
        .disp_an_o(disp_an_o)
    );
    
    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz时钟
    end
    
    // 测试激励
    initial begin
        // 初始化
        rstn = 0;
        sw_i = 16'b0;
        
        $display("=== RISC-V Pipeline CPU Vivado Test ===");
        
        // 等待一段时间后释放复位
        #100;
        rstn = 1;
        
        // 测试1: 高速运行模式 (sw_i[15]=0)
        sw_i[15] = 0;
        sw_i[1] = 0;  // 不暂停
        $display("Test 1: High-speed mode");
        #500;
        
        // 测试2: 低速运行模式 (sw_i[15]=1)
        sw_i[15] = 1;
        $display("Test 2: Low-speed mode");
        #1000;
        
        // 测试3: 暂停模式 (sw_i[1]=1)
        sw_i[1] = 1;
        $display("Test 3: Pause mode");
        #500;
        
        // 测试4: 恢复运行
        sw_i[1] = 0;
        $display("Test 4: Resume execution");
        #500;
        
        // 测试5: 显示寄存器数据 (sw_i[13]=1)
        sw_i[13] = 1;
        sw_i[14:11] = 4'b0100;  // 选择寄存器数据显示
        $display("Test 5: Register data display");
        #1000;
        
        // 测试6: 显示ALU数据 (sw_i[12]=1)
        sw_i[12] = 1;
        sw_i[13] = 0;
        sw_i[14:11] = 4'b0010;  // 选择ALU数据显示
        $display("Test 6: ALU data display");
        #1000;
        
        // 测试7: 显示数据存储器数据 (sw_i[11]=1)
        sw_i[11] = 1;
        sw_i[12] = 0;
        sw_i[14:11] = 4'b0001;  // 选择数据存储器数据显示
        $display("Test 7: Data memory display");
        #1000;
        
        // 测试8: 显示指令 (sw_i[14:11]=1000)
        sw_i[11] = 0;
        sw_i[14:11] = 4'b1000;  // 选择指令显示
        $display("Test 8: Instruction display");
        #1000;
        
        // 测试9: 显示LED数据 (sw_i[0]=1)
        sw_i[0] = 1;
        $display("Test 9: LED data display");
        #500;
        
        $display("=== RISC-V Pipeline CPU Test Results ===");
        $display("Clock Mode: %s", (sw_i[15]) ? "Slow" : "Fast");
        $display("CPU Status: %s", (sw_i[1]) ? "Paused" : "Running");
        $display("Display Mode: %b", sw_i[14:11]);
        $display("LED Data: 0x%h", led_o);
        $display("=== Test Completed ===");
        $finish;
    end
    
    // 监控信号变化
    initial begin
        $monitor("Time=%0t PC=0x%h LED=0x%h sw_i=0x%h", $time, dut.cpu.PC_out, led_o, sw_i);
    end

endmodule 