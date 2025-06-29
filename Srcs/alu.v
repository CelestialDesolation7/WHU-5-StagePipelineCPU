`include "ctrl_encode_def.v"

module alu(A, B, ALUOp, C, Zero, PC);
           
   input  signed [31:0] A, B;
   input         [4:0]  ALUOp;
   input [31:0] PC;
   output signed [31:0] C;
   output Zero;
   
   reg [31:0] C;
   integer    i;
       
   always @( * ) begin
      case ( ALUOp )
         `ALU_NOP:   C = A;
         `ALU_LUI:   C = B;
         `ALU_AUIPC: C = PC + B;
         `ALU_ADD:   C = A + B;
         `ALU_SUB:   C = A - B;
         `ALU_AND:   C = A & B;
         `ALU_OR:    C = A | B;
         `ALU_XOR:   C = A ^ B;
         `ALU_SLL:   C = A << B[4:0];
         `ALU_SRL:   C = A >> B[4:0];
         `ALU_SRA:   C = A >>> B[4:0];
         `ALU_SLT:   C = {31'b0, (A < B)};
         `ALU_SLTU:  C = {31'b0, ($unsigned(A) < $unsigned(B))};
         `ALU_BEQ:   C = {31'b0, (A == B)};
         `ALU_BNE:   C = {31'b0, (A != B)};
         `ALU_BLT:   C = {31'b0, (A < B)};
         `ALU_BGE:   C = {31'b0, (A >= B)};
         `ALU_BLTU:  C = {31'b0, ($unsigned(A) < $unsigned(B))};
         `ALU_BGEU:  C = {31'b0, ($unsigned(A) >= $unsigned(B))};
         default:    C = A;
      endcase
   end // end always
   
   assign Zero = (C == 32'b0);

endmodule
    
