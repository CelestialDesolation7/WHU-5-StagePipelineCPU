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
   wire           MemWrite;
   wire [31:0]    dm_addr, dm_din, dm_dout;
   
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
         .mem_w(MemWrite),          // output: memory write signal
         .PC_out(PC),               // output: PC
         .Addr_out(dm_addr),        // output: address from cpu to memory
         .Data_out(dm_din),         // output: data from cpu to memory
         .reg_sel(reg_sel),         // input:  register selection
         .reg_data(reg_data)        // output: register data
         );
         
  // instantiation of data memory  
   dm    U_dm(
         .clk(clk),           // input:  cpu clock
         .DMWr(MemWrite),     // input:  ram write
         .DMType(3'b000),     // input:  memory access type (word for now)
         .addr(dm_addr),      // input:  ram address (full 32-bit address)
         .din(dm_din),        // input:  data to ram
         .dout(dm_dout)       // output: data from ram
         );
         
  // instantiation of ROM IP core (dist_mem_gen_0)
   dist_mem_gen_0 U_ROM ( 
      .a(PC[8:2]),     // input:  rom address (7-bit)
      .spo(instr)      // output: instruction (32-bit)
   );
        
endmodule

