`timescale 1ns/1ps
module sccomp_rom_testbench();
    reg clk, rstn;
    wire [4:0] reg_sel = 5'd2;
    wire [31:0] reg_data;
    wire [31:0] instr;
    wire [31:0] PC_out;
    wire [31:0] mem_addr_out;
    wire [31:0] mem_data_out;
    wire [31:0] debug_data;

    // 实例化sccomp
    sccomp uut(
        .clk(clk),
        .rstn(rstn),
        .reg_sel(reg_sel),
        .reg_data(reg_data),
        .instr(instr),
        .PC_out(PC_out),
        .mem_addr_out(mem_addr_out),
        .mem_data_out(mem_data_out),
        .debug_data(debug_data)
    );

    // 时钟生成
    initial clk = 0;
    always #5 clk = ~clk;

    // 测试流程
    initial begin
        rstn = 0;
        #20 rstn = 1;
        // 运行足够多的周期
        #30_000;
        $stop;
    end
endmodule 