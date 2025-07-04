`timescale 1ns / 1ps

// Load-Use冒险检测测试模块
module load_use_hazard_test();

    // 测试信号
    reg clk;
    reg rst;
    reg [31:0] instr_in;
    reg [31:0] Data_in;
    reg [4:0] reg_sel;
    wire [31:0] reg_data;
    wire [31:0] instr;
    wire [31:0] PC_out;
    wire [31:0] Addr_out;
    wire [31:0] Data_out;
    wire [2:0] DMType_out;
    wire [31:0] debug_data;
    
    // 实例化CPU
    sccomp cpu(
        .clk(clk),
        .rstn(~rst),
        .reg_sel(reg_sel),
        .reg_data(reg_data),
        .instr(instr),
        .PC_out(PC_out),
        .Addr_out(Addr_out),
        .Data_out(Data_out),
        .DMType_out(DMType_out),
        .debug_data(debug_data)
    );
    
    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz时钟
    end
    
    // 测试过程
    initial begin
        // 初始化
        rst = 1;
        reg_sel = 5'd2;  // 选择x2寄存器进行观察
        
        // 等待几个时钟周期
        #20;
        rst = 0;
        
        // 等待CPU执行
        #1000;
        
        // 检查x2寄存器的值
        $display("=== Load-Use冒险检测测试 ===");
        $display("时间 %0t: x2寄存器值 = 0x%h", $time, reg_data);
        
        // 检查x7寄存器的值（Load指令的目标寄存器）
        reg_sel = 5'd7;
        #10;
        $display("时间 %0t: x7寄存器值 = 0x%h", $time, reg_data);
        
        // 检查x15寄存器的值（Load指令的目标寄存器）
        reg_sel = 5'd15;
        #10;
        $display("时间 %0t: x15寄存器值 = 0x%h", $time, reg_data);
        
        // 检查x4寄存器的值
        reg_sel = 5'd4;
        #10;
        $display("时间 %0t: x4寄存器值 = 0x%h", $time, reg_data);
        
        $display("=== 测试完成 ===");
        $finish;
    end
    
    // 监控关键信号
    always @(posedge clk) begin
        if (!rst) begin
            $display("时间 %0t: PC=0x%h, 指令=0x%h", $time, PC_out, instr);
        end
    end

endmodule 