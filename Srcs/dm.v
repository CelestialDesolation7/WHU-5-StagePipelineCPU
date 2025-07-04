`include "ctrl_encode_def.v"
// 数据存储器模块 - 支持字节、半字、字的读写操作
// 该模块实现了RISC-V RV32I指令集中的所有存储器访问指令
module dm(clk, DMWr, DMType, addr, din, dout);
   input          clk;        // 时钟信号
   input          DMWr;       // 存储器写使能信号 (1=写, 0=读)
   input  [2:0]   DMType;     // 存储器访问类型控制信号
   input  [31:0]  addr;       // 存储器地址 (完整32位地址)
   input  [31:0]  din;        // 写入数据 (32位)
   output [31:0]  dout;       // 读出数据 (32位)
     
   reg [31:0] dmem[127:0];    // 数据存储器数组，128个字，每个字32位
   wire [31:0] mem_data;      // 从存储器读取的原始数据
   wire [1:0] byte_offset;    // 字节偏移量 (地址的低2位)
   wire [6:0] word_addr;      // 字地址 (地址的高7位)
   integer i;
   
   initial begin
      for (i = 0; i < 128; i = i + 1)
          dmem[i] = 32'b0;
   end
   
   // 计算字地址和字节偏移量
   assign word_addr = addr[8:2];     // 字地址：地址的高7位
   assign byte_offset = addr[1:0];   // 字节偏移：地址的低2位
   
   // 从存储器读取原始数据 (字对齐访问)
   // 当地址超出物理内存空间时（addr[31:9]不为0）返回0
   assign mem_data = (addr[31:9] != 23'b0) ? 32'b0 : dmem[word_addr];
   
   // 读操作 - 根据访问类型和字节偏移量返回适当的数据
   assign dout = (DMType == `DM_WORD) ? mem_data :                    // 字访问：直接返回32位数据
                 (DMType == `DM_HALFWORD) ?                           // 半字访问：有符号扩展
                   (byte_offset[1] ? {16'b0, mem_data[31:16]} : {16'b0, mem_data[15:0]}) :
                 (DMType == `DM_HALFWORD_UNSIGNED) ?                  // 半字访问：无符号扩展
                   (byte_offset[1] ? {16'b0, mem_data[31:16]} : {16'b0, mem_data[15:0]}) :
                 (DMType == `DM_BYTE) ?                               // 字节访问：有符号扩展
                   (byte_offset == 2'b00 ? {24'b0, mem_data[7:0]} :   // 字节0
                    byte_offset == 2'b01 ? {24'b0, mem_data[15:8]} :  // 字节1
                    byte_offset == 2'b10 ? {24'b0, mem_data[23:16]} : // 字节2
                    {24'b0, mem_data[31:24]}) :                       // 字节3
                 (DMType == `DM_BYTE_UNSIGNED) ?                      // 字节访问：无符号扩展
                   (byte_offset == 2'b00 ? {24'b0, mem_data[7:0]} :   // 字节0
                    byte_offset == 2'b01 ? {24'b0, mem_data[15:8]} :  // 字节1
                    byte_offset == 2'b10 ? {24'b0, mem_data[23:16]} : // 字节2
                    {24'b0, mem_data[31:24]}) :                       // 字节3
                 mem_data;                                             // 默认返回字数据
   
   // 写操作 - 在时钟上升沿执行
   always @(posedge clk) begin
      if (DMWr && (addr[31:9] == 23'b0)) begin  // 当写使能有效且地址在有效范围内时
         case (DMType)
            `DM_WORD: begin  // 字写入：直接写入32位数据
               dmem[word_addr] <= din;  // 写入整个字
            end
            `DM_HALFWORD: begin  // 半字写入：写入16位数据
               if (byte_offset[1])  // 如果偏移量是2或3，写入高16位
                  dmem[word_addr][31:16] <= din[15:0];
               else                 // 如果偏移量是0或1，写入低16位
                  dmem[word_addr][15:0] <= din[15:0];
            end
            `DM_BYTE: begin  // 字节写入：写入8位数据
               case (byte_offset)  // 根据字节偏移量选择写入位置
                  2'b00: dmem[word_addr][7:0] <= din[7:0];    // 字节0
                  2'b01: dmem[word_addr][15:8] <= din[7:0];   // 字节1
                  2'b10: dmem[word_addr][23:16] <= din[7:0];  // 字节2
                  2'b11: dmem[word_addr][31:24] <= din[7:0];  // 字节3
               endcase
            end
         endcase
      end
   end
    
endmodule    
