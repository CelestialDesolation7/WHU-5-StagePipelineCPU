`include "ctrl_encode_def.v"

module PC_NPC(
  input clk,
  input rst,
  input stall,
  input [31:0] base_PC,
  input [2:0] NPCOp,
  input [31:0] IMM,
  input [31:0] aluout,
  output reg [31:0] PC
);

  // 组合逻辑：计算下一个PC
  wire [31:0] PCPLUS4 = PC + 4;
  reg [31:0] next_PC;

  always @(*) begin
    case (NPCOp)
      `NPC_PLUS4:  next_PC = PCPLUS4;
      `NPC_BRANCH: next_PC = base_PC + IMM;
      `NPC_JUMP:   next_PC = base_PC + IMM;
      `NPC_JALR:   next_PC = aluout;
      default:     next_PC = PCPLUS4;
    endcase
  end

  // 时序逻辑：PC寄存器
  always @(posedge clk or posedge rst) begin
    if (rst)
      PC <= 32'h0000_0000;
    else if (!stall)
      PC <= next_PC;
  end

endmodule

