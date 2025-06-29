module PC( clk, rst, NPC, PC, stall );

  input              clk;
  input              rst;
  input       [31:0] NPC;
  input              stall;
  output reg  [31:0] PC;

  always @(posedge clk, posedge rst) begin
    if (rst) 
      PC <= 32'h0000_0000;
    else if (!stall)
      PC <= NPC;
  end
endmodule

