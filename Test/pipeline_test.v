`timescale 1ns / 1ps

// 流水线功能测试 - 测试数据冒险和Load-Use冒险处理
module pipeline_test();

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
        
        $display("=== RISC-V Pipeline CPU Hazard Test ===");
        $display("Testing data hazards and load-use hazards...");
        
        // 等待一段时间后释放复位
        #100;
        rstn = 1;
        
        // 运行CPU执行指令
        #2000;
        
        // 检查流水线冒险处理结果
        $display("=== Pipeline Hazard Test Results ===");
        
        // 检查关键寄存器的值
        reg_sel = 5'b00001;  // x1
        #10;
        $display("x1 = 0x%h", reg_data);
        
        reg_sel = 5'b00010;  // x2
        #10;
        $display("x2 = 0x%h", reg_data);
        
        reg_sel = 5'b00011;  // x3
        #10;
        $display("x3 = 0x%h", reg_data);
        
        reg_sel = 5'b00100;  // x4
        #10;
        $display("x4 = 0x%h", reg_data);
        
        reg_sel = 5'b00101;  // x5
        #10;
        $display("x5 = 0x%h", reg_data);
        
        reg_sel = 5'b00110;  // x6
        #10;
        $display("x6 = 0x%h", reg_data);
        
        reg_sel = 5'b00111;  // x7
        #10;
        $display("x7 = 0x%h", reg_data);
        
        reg_sel = 5'b01000;  // x8
        #10;
        $display("x8 = 0x%h", reg_data);
        
        reg_sel = 5'b01001;  // x9
        #10;
        $display("x9 = 0x%h", reg_data);
        
        reg_sel = 5'b01010;  // x10
        #10;
        $display("x10 = 0x%h", reg_data);
        
        $display("=== Pipeline Test Completed ===");
        $display("If all registers show correct values, hazard handling is working properly.");
        $finish;
    end
    
    // 监控流水线状态
    initial begin
        $monitor("Time=%0t PC=0x%h instr=0x%h", $time, PC_out, instr);
    end

endmodule 