`timescale 1ns / 1ps

// 简单功能测试 - 测试RISC-V流水线CPU的基本功能
module simple_test();

    // 测试信号
    reg clk;
    reg rstn;
    reg [4:0] reg_sel;
    wire [31:0] reg_data;
    wire [31:0] instr;
    wire [31:0] PC_out;
    
    // 实例化CPU
    sccomp cpu(
        .clk(clk),
        .rstn(rstn),
        .reg_sel(reg_sel),
        .reg_data(reg_data),
        .instr(instr),
        .PC_out(PC_out)
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
        reg_sel = 5'b0;
        
        // 等待一段时间后释放复位
        #100;
        rstn = 1;
        
        // 运行CPU一段时间
        #1000;
        
        // 测试不同寄存器的值
        $display("=== RISC-V Pipeline CPU Simple Test Results ===");
        
        // 检查寄存器x1
        reg_sel = 5'b00001;
        #10;
        $display("x1 = 0x%h", reg_data);
        
        // 检查寄存器x2
        reg_sel = 5'b00010;
        #10;
        $display("x2 = 0x%h", reg_data);
        
        // 检查寄存器x3
        reg_sel = 5'b00011;
        #10;
        $display("x3 = 0x%h", reg_data);
        
        // 检查寄存器x4
        reg_sel = 5'b00100;
        #10;
        $display("x4 = 0x%h", reg_data);
        
        // 检查寄存器x5
        reg_sel = 5'b00101;
        #10;
        $display("x5 = 0x%h", reg_data);
        
        $display("=== Test Completed ===");
        $finish;
    end
    
    // 监控信号变化
    initial begin
        $monitor("Time=%0t PC=0x%h instr=0x%h", $time, PC_out, instr);
    end

endmodule 