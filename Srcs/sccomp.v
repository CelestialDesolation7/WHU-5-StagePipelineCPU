module sccomp(clk, rstn, reg_sel, reg_data, instr, PC_out, mem_addr_out, mem_data_out, debug_data);
   input          clk;
   input          rstn;
   input [4:0]    reg_sel;
   output [31:0]  reg_data;
   output [31:0]  instr;
   output [31:0]  PC_out;
   output [31:0]  mem_addr_out;      // 内存访问地址输出
   output [31:0]  mem_data_out;      // 内存访问数据输出
   output [31:0]  debug_data;
   
   wire [31:0]    PC;
   wire           MemWrite;
   wire [31:0]    dm_addr, dm_din, dm_dout;
   wire [2:0]     DMType;
   wire [31:0]    debug_data_wire;
   
   wire rst = ~rstn;
   
   // 输出PC
   assign PC_out = PC;
   
   // 输出内存访问地址
   assign mem_addr_out = dm_addr;
   
   // 根据读写状态输出相应数据
   // 如果CPU正在向内存写数据，输出写入的数据
   // 如果内存正在被读，输出读到的数据
   assign mem_data_out = MemWrite ? dm_din : dm_dout;
       
  // instantiation of 5-stage pipeline CPU   
   PipelineCPU U_PipelineCPU(
         .clk(clk),                 // input:  cpu clock
         .rst(rst),                 // input:  reset
         .instr_in(instr),          // input:  instruction
         .Data_in(dm_dout),         // input:  data to cpu  
         .mem_w(MemWrite),          // output: memory write signal
         .PC_out(PC),               // output: PC
         .Addr_out(dm_addr),        // output: address from cpu to memory
         .Data_out(dm_din),         // output: data from cpu to memory
         .reg_sel(reg_sel),         // input:  register selection
         .reg_data(reg_data),        // output: register data
         .DMType_out(DMType),        // output: memory access type
         .debug_data(debug_data_wire) // output: debug data
         );
         
  // instantiation of data memory  
   dm    U_dm(
         .clk(clk),           // input:  cpu clock
         .DMWr(MemWrite),     // input:  ram write
         .DMType(DMType),      // input:  memory access type
         .addr(dm_addr),      // input:  ram address (full 32-bit address)
         .din(dm_din),        // input:  data to ram
         .dout(dm_dout)       // output: data from ram
         );
         
  // instantiation of ROM IP core (dist_mem_gen_0)
   dist_mem_gen_0 U_ROM ( 
      .a(PC[8:2]),     // input:  rom address (7-bit)
      .spo(instr)      // output: instruction (32-bit)
   );
   
   assign debug_data = debug_data_wire;
        
endmodule

