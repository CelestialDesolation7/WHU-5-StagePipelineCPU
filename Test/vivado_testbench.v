`timescale 1ns/1ps
`include "ctrl_encode_def.v"
module vivado_testbench();
    reg clk, rst;
    reg [31:0] instr_mem[0:15]; // 简单指令存储器
    reg [31:0] data_mem[0:127]; // 数据存储器镜像
    wire mem_w;
    wire [31:0] PC_out, Addr_out, Data_out;
    wire [2:0] DMType_out;
    wire [31:0] debug_data;
    reg [31:0] instr_in;
    wire [31:0] Data_in;
    reg [4:0] reg_sel;
    wire [31:0] reg_data;

    // 实例化数据存储器
    dm u_dm(
        .clk(clk),
        .DMWr(mem_w),
        .DMType(DMType_out),
        .addr(Addr_out),
        .din(Data_out),
        .dout(Data_in)
    );

    // 实例化CPU
    PipelineCPU uut(
        .clk(clk),
        .rst(rst),
        .instr_in(instr_in),
        .Data_in(Data_in),
        .mem_w(mem_w),
        .PC_out(PC_out),
        .Addr_out(Addr_out),
        .Data_out(Data_out),
        .reg_sel(reg_sel),
        .reg_data(reg_data),
        .DMType_out(DMType_out),
        .debug_data(debug_data)
    );

    // 时钟生成
    initial clk = 0;
    always #5 clk = ~clk;

    // 指令ROM仿真
    always @(*) begin
        if (PC_out[31:2] < 16)
            instr_in = instr_mem[PC_out[5:2]];
        else
            instr_in = 32'h00000013; // NOP
    end

    // 初始化和测试流程
    initial begin
        rst = 1;
        reg_sel = 0;
        // 指令：x1=5, x2=10, x3=x1+x2, mem[0]=x3, x4=mem[0]
        instr_mem[0] = 32'b000000000101_00000_000_00001_0010011; // ADDI x1,x0,5
        instr_mem[1] = 32'b000000001010_00000_000_00010_0010011; // ADDI x2,x0,10
        instr_mem[2] = 32'b0000000_00010_00001_000_00011_0110011; // ADD x3,x1,x2
        instr_mem[3] = 32'b0000000_00011_00000_010_00000_0100011; // SW x3,0(x0)
        instr_mem[4] = 32'b0000000_00000_00000_010_00100_0000011; // LW x4,0(x0)
        instr_mem[5] = 32'b000000000000_00000_000_00000_0010011; // NOP
        // 其余填充为NOP
        for(integer i=6;i<16;i=i+1) instr_mem[i]=32'h00000013;
        // 数据存储器初始化
        for(integer i=0;i<128;i=i+1) data_mem[i]=0;
        #20 rst = 0;
        // 等待指令执行
        #200;
        // 检查结果
        reg_sel = 5'd1; #10; $display("x1 = %d", reg_data);
        reg_sel = 5'd2; #10; $display("x2 = %d", reg_data);
        reg_sel = 5'd3; #10; $display("x3 = %d", reg_data);
        reg_sel = 5'd4; #10; $display("x4 = %d", reg_data);
        $display("mem[0] = %d", u_dm.dmem[0]);
        $finish;
    end
endmodule 