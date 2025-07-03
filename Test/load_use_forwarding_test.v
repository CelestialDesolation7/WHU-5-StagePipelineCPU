`timescale 1ns / 1ps

module load_use_forwarding_test();
    reg clk;
    reg rst;
    reg [31:0] instr_in;
    reg [31:0] Data_in;
    reg [4:0] reg_sel;
    
    wire [31:0] PC_out;
    wire [31:0] Addr_out;
    wire [31:0] Data_out;
    wire mem_w;
    wire [2:0] DMType_out;
    wire [31:0] debug_data;
    wire [31:0] reg_data;
    
    // 实例化被测试模块
    PipelineCPU cpu(
        .clk(clk),
        .rst(rst),
        .instr_in(instr_in),
        .Data_in(Data_in),
        .reg_sel(reg_sel),
        .PC_out(PC_out),
        .Addr_out(Addr_out),
        .Data_out(Data_out),
        .mem_w(mem_w),
        .DMType_out(DMType_out),
        .debug_data(debug_data),
        .reg_data(reg_data)
    );
    
    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // 测试用例
    initial begin
        // 初始化
        rst = 1;
        instr_in = 32'h00000000;
        Data_in = 32'h00000000;
        reg_sel = 5'b0;
        
        // 等待几个时钟周期
        #20;
        rst = 0;
        
        // 测试序列：复现load-use冒险场景
        // 0x28: lw x15, 0(x0)  - 从内存地址0加载数据到x15
        #10;
        instr_in = 32'h00002783; // lw x15, 0(x0)
        Data_in = 32'h87654321;  // 模拟从内存读取的数据
        
        // 0x2c: add x2, x0, x0  - 空操作
        #10;
        instr_in = 32'h00000133; // add x2, x0, x0
        Data_in = 32'h00000000;
        
        // 0x30: addi x4, x0, 15  - 立即数加法
        #10;
        instr_in = 32'h00F00213; // addi x4, x0, 15
        Data_in = 32'h00000000;
        
        // 0x34: and x7, x15, x4  - 关键的AND指令
        #10;
        instr_in = 32'h0047F3B3; // and x7, x15, x4
        Data_in = 32'h00000000;
        
        // 继续执行几个周期
        #10;
        instr_in = 32'h00000000; // NOP
        
        // 等待指令完成
        #50;
        
        // 检查结果
        $display("=== Load-Use Forwarding Test Results ===");
        $display("Time: %t", $time);
        
        // 检查x15寄存器的值
        reg_sel = 5'd15;
        #1;
        $display("x15 (should be 0x87654321): 0x%h", reg_data);
        
        // 检查x4寄存器的值
        reg_sel = 5'd4;
        #1;
        $display("x4 (should be 0x0000000F): 0x%h", reg_data);
        
        // 检查x7寄存器的值 (0x87654321 & 0x0000000F = 0x00000001)
        reg_sel = 5'd7;
        #1;
        $display("x7 (should be 0x00000001): 0x%h", reg_data);
        
        if (reg_data == 32'h00000001) begin
            $display("TEST PASSED: AND instruction executed correctly with forwarding!");
        end else begin
            $display("TEST FAILED: AND instruction result is incorrect");
        end
        
        $display("=== Test Complete ===");
        $finish;
    end
    
    // 监控信号
    initial begin
        $monitor("Time: %t, PC: 0x%h, Instr: 0x%h, x15: 0x%h, x4: 0x%h, x7: 0x%h", 
                 $time, PC_out, instr_in, 
                 (reg_sel == 5'd15) ? reg_data : 32'hxxxxxxxx,
                 (reg_sel == 5'd4) ? reg_data : 32'hxxxxxxxx,
                 (reg_sel == 5'd7) ? reg_data : 32'hxxxxxxxx);
    end
    
endmodule 