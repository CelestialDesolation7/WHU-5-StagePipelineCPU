`timescale 1ns/1ps
`include "ctrl_encode_def.v"
module hal_pipeline_test();
    reg clk, rst;
    reg [31:0] instr_mem[0:7];
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
        if (PC_out[31:2] < 8)
            instr_in = instr_mem[PC_out[5:2]];
        else
            instr_in = 32'h00000013; // NOP
    end

    // 初始化和测试流程
    initial begin
        rst = 1;
        reg_sel = 0;
        // 指令序列：x1=1, x2=2, x3=x1+x2, jal x4, +12, x5=5, x6=6, x7=x5+x6, addi x8, x0, 8 (跳转目标)
        instr_mem[0] = 32'h00100093; // ADDI x1,x0,1            0x00
        instr_mem[1] = 32'h00200113; // ADDI x2,x0,2            0x04
        instr_mem[2] = 32'h002081b3; // ADD x3,x1,x2            0x08
        instr_mem[3] = 32'h00c0026f; // JAL x4, +12 (PC+12)     0x0C
        instr_mem[4] = 32'h00500293; // ADDI x5,x0,5 (跳过)      0x10
        instr_mem[5] = 32'h00600313; // ADDI x6,x0,6 (跳过)      0x14
        instr_mem[6] = 32'h006283b3; // ADD x7,x5,x6 (跳转到这里)      0x18
        instr_mem[7] = 32'h00400413; // ADDI x8,x0,4 0x1C
        for(integer i=8;i<16;i=i+1) instr_mem[i]=32'h00000013;
        #20 rst = 0;
        // 逐周期监控流水线关键信号
        $display("Cycle | PC_IF | NPC_IF | instr_IF | PC_ID | instr_ID | PC_EX | instr_EX | PC_MEM | instr_MEM | PC_WB | instr_WB | RegWrite_ID | RegWrite_EX | RegWrite_MEM | RegWrite_WB | JAL Control | x2 (jal rd) | x1 | x4");
        repeat(12) begin
            reg_sel = 5'd2; #1; // x2 (jal目标寄存器)
            $write("%2d | %8h | %8h | %8h | %8h | %8h | %8h | %8h | %8h | %8h | %8h | %8h | ",
                $time/10,
                uut.PC_IF, uut.NPC_IF, uut.instr_IF,
                uut.PC_IF_ID, uut.instr_IF_ID,
                uut.PC_ID_EX, uut.instr_ID_EX,
                uut.PC_EX_MEM, uut.instr_EX_MEM,
                uut.PC_MEM_WB, uut.instr_MEM_WB);
            $write("%1b | %1b | %1b | %1b | %1b | %8h | ",
                uut.RegWrite_ID, uut.RegWrite_EX, uut.RegWrite_MEM, uut.RegWrite_WB,
                uut.opcode_ID == `OPCODE_JAL,
                reg_data);
            reg_sel = 5'd1; #1; $write("%8h | ", reg_data);
            reg_sel = 5'd4; #1; $write("%8h\n", reg_data);
            #10;
        end
        $finish;
    end
endmodule 