`include "ctrl_encode_def.v"

module alu(A, B, ALUOp, C, Zero, Sign, Overflow, Carry, PC);
           
   input  signed [31:0] A, B;
   input         [4:0]  ALUOp;
   input [31:0] PC;
   output signed [31:0] C;
   output Zero, Sign, Overflow, Carry;
   
   reg [31:0] C;
   reg Zero, Sign, Overflow, Carry;
   reg [32:0] temp_result;  // 33位用于检测溢出和进位
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
         `ALU_BEQ:   C = A - B;  // 减法，用于比较
         `ALU_BNE:   C = A - B;  // 减法，用于比较
         `ALU_BLT:   C = A - B;  // 减法，用于比较
         `ALU_BGE:   C = A - B;  // 减法，用于比较
         `ALU_BLTU:  C = $unsigned(A) - $unsigned(B);  // 无符号减法
         `ALU_BGEU:  C = $unsigned(A) - $unsigned(B);  // 无符号减法
         default:    C = A;
      endcase
      
      // 计算标志位
      case ( ALUOp )
         `ALU_ADD: begin
            temp_result = {1'b0, A} + {1'b0, B};
            Zero = (C == 32'b0);
            Sign = C[31];
            Overflow = (A[31] == B[31]) && (C[31] != A[31]);
            Carry = temp_result[32];
         end
         `ALU_SUB, `ALU_BEQ, `ALU_BNE, `ALU_BLT, `ALU_BGE: begin
            temp_result = {1'b0, A} - {1'b0, B};
            Zero = (C == 32'b0);
            Sign = C[31];
            Overflow = (A[31] != B[31]) && (C[31] == B[31]);
            Carry = temp_result[32];
         end
         `ALU_BLTU, `ALU_BGEU: begin
            temp_result = {1'b0, $unsigned(A)} - {1'b0, $unsigned(B)};
            Zero = (C == 32'b0);
            Sign = C[31];
            Overflow = 1'b0;  // 无符号运算无溢出
            Carry = temp_result[32];
         end
         default: begin
            Zero = (C == 32'b0);
            Sign = C[31];
            Overflow = 1'b0;
            Carry = 1'b0;
         end
      endcase
   end // end always

endmodule
    
