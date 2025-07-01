module MEM_stage(
    input clk,
    input rst,
    input [31:0] alu_result_MEM,
    input [31:0] rs2_data_MEM,
    input [4:0] rd_addr_MEM,
    input RegWrite_MEM,
    input MemWrite_MEM,
    input MemRead_MEM,
    input [1:0] WDSel_MEM,
    input [2:0] DMType_MEM,
    input [31:0] Data_in, // 外部dm的读出数据
    output [31:0] mem_data_MEM // 直通输出
);
    assign mem_data_MEM = Data_in;
endmodule 