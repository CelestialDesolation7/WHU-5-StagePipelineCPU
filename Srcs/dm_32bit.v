`include "ctrl_encode_def.v"
// 32位地址空间数据存储器模块 - 使用Vivado IP核接口
// 该模块实现了RISC-V RV32I指令集中的所有存储器访问指令
// 支持32位地址空间，但实际内存大小由IP核配置决定
module dm_32bit(clk, DMWr, DMType, addr, din, dout);
   input          clk;        // 时钟信号
   input          DMWr;       // 存储器写使能信号 (1=写, 0=读)
   input  [2:0]   DMType;     // 存储器访问类型控制信号
   input  [31:0]  addr;       // 存储器地址 (完整32位地址)
   input  [31:0]  din;        // 写入数据 (32位)
   output [31:0]  dout;       // 读出数据 (32位)
     
   // 内部信号
   wire [31:0] mem_data;      // 从IP核读取的原始数据
   wire [1:0] byte_offset;    // 字节偏移量 (地址的低2位)
   wire [31:0] word_addr;     // 字地址 (完整32位地址)
   wire [31:0] write_data;    // 写入IP核的数据
   wire [31:0] read_data;     // 从IP核读取的数据
   wire write_enable;         // IP核写使能
   wire [3:0] byte_enable;    // 字节使能信号
   
   // 计算字地址和字节偏移量
   assign word_addr = {addr[31:2], 2'b00};  // 字对齐地址
   assign byte_offset = addr[1:0];          // 字节偏移
   
   // 生成字节使能信号
   assign byte_enable = (DMType == `DM_WORD) ? 4'b1111 :
                       (DMType == `DM_HALFWORD) ? 
                         (byte_offset[1] ? 4'b1100 : 4'b0011) :
                       (DMType == `DM_BYTE) ?
                         (byte_offset == 2'b00 ? 4'b0001 :
                          byte_offset == 2'b01 ? 4'b0010 :
                          byte_offset == 2'b10 ? 4'b0100 : 4'b1000) : 4'b0000;
   
   // 准备写入数据（根据访问类型和字节偏移）
   assign write_data = (DMType == `DM_WORD) ? din :
                      (DMType == `DM_HALFWORD) ? 
                        (byte_offset[1] ? {din[15:0], 16'b0} : {16'b0, din[15:0]}) :
                      (DMType == `DM_BYTE) ?
                        (byte_offset == 2'b00 ? {24'b0, din[7:0]} :
                         byte_offset == 2'b01 ? {16'b0, din[7:0], 8'b0} :
                         byte_offset == 2'b10 ? {8'b0, din[7:0], 16'b0} : {din[7:0], 24'b0}) : din;
   
   // IP核写使能
   assign write_enable = DMWr;
   
   // 实例化Block Memory Generator IP核
   // 注意：您需要在Vivado中创建这个IP核
   blk_mem_gen_0 data_memory (
       .clka(clk),                    // 时钟
       .ena(1'b1),                    // 使能信号
       .wea(write_enable ? byte_enable : 4'b0000), // 写使能（字节级别）
       .addra(word_addr[31:2]),       // 地址（需要根据IP核配置调整）
       .dina(write_data),             // 写入数据
       .douta(read_data)              // 读出数据
   );
   
   // 处理读取数据（根据访问类型和字节偏移）
   assign dout = (DMType == `DM_WORD) ? read_data :
                (DMType == `DM_HALFWORD) ? 
                  (byte_offset[1] ? {{16{read_data[31]}}, read_data[31:16]} : 
                                   {{16{read_data[15]}}, read_data[15:0]}) :
                (DMType == `DM_HALFWORD_UNSIGNED) ? 
                  (byte_offset[1] ? {16'b0, read_data[31:16]} : {16'b0, read_data[15:0]}) :
                (DMType == `DM_BYTE) ? 
                  (byte_offset == 2'b00 ? {{24{read_data[7]}}, read_data[7:0]} :
                   byte_offset == 2'b01 ? {{24{read_data[15]}}, read_data[15:8]} :
                   byte_offset == 2'b10 ? {{24{read_data[23]}}, read_data[23:16]} :
                   {{24{read_data[31]}}, read_data[31:24]}) :
                (DMType == `DM_BYTE_UNSIGNED) ? 
                  (byte_offset == 2'b00 ? {24'b0, read_data[7:0]} :
                   byte_offset == 2'b01 ? {24'b0, read_data[15:8]} :
                   byte_offset == 2'b10 ? {24'b0, read_data[23:16]} :
                   {24'b0, read_data[31:24]}) : read_data;
   
endmodule 