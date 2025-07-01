module sccomp(clk, rstn, reg_sel, reg_data, instr, PC_out, Addr_out, Data_out);
   input          clk;
   input          rstn;
   input [4:0]    reg_sel;
   output [31:0]  reg_data;
   output [31:0]  instr;
   output [31:0]  PC_out;
   output [31:0]  Addr_out;
   output [31:0]  Data_out;
   
   wire [31:0]    PC;
   wire           MemWrite_MEM;
   wire [2:0]     DMType_MEM;
   wire [31:0]    alu_result_MEM, rs2_data_MEM, dm_dout;
   wire           mem_w;
   wire [31:0]    dm_addr, dm_din;
   
   wire rst = ~rstn;
   
   // 输出PC
   assign PC_out = PC;
   // 输出地址和数据
   assign Addr_out = dm_addr;
   assign Data_out = dm_din;
       
  // instantiation of 5-stage pipeline CPU   
   PipelineCPU U_PipelineCPU(
         .clk(clk),                 // input:  cpu clock
         .rst(rst),                 // input:  reset
         .instr_in(instr),          // input:  instruction
         .Data_in(dm_dout),         // input:  data to cpu  
         .mem_w(mem_w),             // output: memory write signal
         .PC_out(PC),               // output: PC
         .Addr_out(dm_addr),        // output: address from cpu to memory
         .Data_out(dm_din),         // output: data from cpu to memory
         .MemWrite_MEM(MemWrite_MEM),
         .DMType_MEM(DMType_MEM),
         .alu_result_MEM(alu_result_MEM),
         .rs2_data_MEM(rs2_data_MEM),
         .reg_sel(reg_sel),         // input:  register selection
         .reg_data(reg_data)        // output: register data
         );
         
  // instantiation of data memory  
   dm    U_dm(
         .clk(clk),           // input:  cpu clock
         .DMWr(MemWrite_MEM),     // input:  ram write
         .DMType(DMType_MEM),     // input:  memory access type
         .addr(alu_result_MEM),      // input:  ram address (full 32-bit address)
         .din(rs2_data_MEM),        // input:  data to ram
         .dout(dm_dout)       // output: data from ram
         );
         
  // instantiation of ROM IP core (dist_mem_gen_0)
   dist_mem_gen_0 U_ROM ( 
      .a(PC[8:2]),     // input:  rom address (7-bit)
      .spo(instr)      // output: instruction (32-bit)
   );
        
endmodule

