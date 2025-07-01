`include "ctrl_encode_def.v"
module IF_stage(
    input clk,
    input rst,
    input stall,
    input [31:0] instr_in,
    input [2:0] npc_op,
    input [31:0] npc_imm,
    input [31:0] npc_pc,
    output [31:0] PC,
    output [31:0] instr,
    output [31:0] NPC
);
    wire [31:0] next_PC;
    PC pc_unit(.clk(clk), .rst(rst), .NPC(next_PC), .PC(PC), .stall(stall));
    NPC npc_unit(.PC(npc_pc), .NPCOp(npc_op), .IMM(npc_imm), .NPC(next_PC), .aluout());
    assign instr = instr_in;
    assign NPC = next_PC;
endmodule 