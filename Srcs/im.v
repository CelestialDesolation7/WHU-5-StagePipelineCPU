// instruction memory with test program initialization
module im(input  [8:2]  addr,
          output [31:0] dout );

  reg  [31:0] ROM[127:0];

  // Initialize ROM with test program
  initial begin
    // Initialize all memory to 0
    for (integer i = 0; i < 128; i = i + 1) begin
      ROM[i] = 32'h00000000;
    end
    
    // Test program for RISC-V RV32I pipeline CPU
    // LUI x1, 0x12345     // Load upper immediate
    ROM[0] = 32'h12345037;
    
    // ADDI x2, x0, 10     // Add immediate
    ROM[1] = 32'h00A00113;
    
    // ADDI x3, x0, 20     // Add immediate
    ROM[2] = 32'h01400193;
    
    // ADD x4, x2, x3      // Add registers
    ROM[3] = 32'h00310233;
    
    // SUB x5, x4, x2      // Subtract registers
    ROM[4] = 32'h40220293;
    
    // AND x6, x4, x2      // AND registers
    ROM[5] = 32'h00223333;
    
    // OR x7, x4, x2       // OR registers
    ROM[6] = 32'h002243B3;
    
    // XOR x8, x4, x2      // XOR registers
    ROM[7] = 32'h00225433;
    
    // SLT x9, x2, x3      // Set if less than
    ROM[8] = 32'h003124B3;
    
    // SW x4, 0(x0)        // Store word
    ROM[9] = 32'h00402023;
    
    // LW x10, 0(x0)       // Load word
    ROM[10] = 32'h00002503;
    
    // BEQ x2, x3, 8       // Branch if equal
    ROM[11] = 32'h00310263;
    
    // JAL x11, 16         // Jump and link
    ROM[12] = 32'h01000597;
    
    // AUIPC x12, 0x1000   // Add upper immediate to PC
    ROM[13] = 32'h00010617;
    
    // SLTI x13, x2, 15    // Set if less than immediate
    ROM[14] = 32'h00F12693;
    
    // SLTIU x14, x2, 15   // Set if less than immediate unsigned
    ROM[15] = 32'h00F13713;
    
    // XORI x15, x2, 0xFF  // XOR immediate
    ROM[16] = 32'h0FF14793;
    
    // ORI x16, x2, 0xFF   // OR immediate
    ROM[17] = 32'h0FF16813;
    
    // ANDI x17, x2, 0xFF  // AND immediate
    ROM[18] = 32'h0FF17893;
    
    // SLLI x18, x2, 2     // Shift left logical immediate
    ROM[19] = 32'h00211913;
    
    // SRLI x19, x2, 1     // Shift right logical immediate
    ROM[20] = 32'h00115993;
    
    // SRAI x20, x2, 1     // Shift right arithmetic immediate
    ROM[21] = 32'h40115913;
    
    // SLTU x21, x2, x3    // Set if less than unsigned
    ROM[22] = 32'h00313AB3;
    
    // SLL x22, x2, x3     // Shift left logical
    ROM[23] = 32'h00311B33;
    
    // SRL x23, x2, x3     // Shift right logical
    ROM[24] = 32'h00315BB3;
    
    // SRA x24, x2, x3     // Shift right arithmetic
    ROM[25] = 32'h40315C33;
    
    // BNE x2, x3, 8       // Branch if not equal
    ROM[26] = 32'h02311263;
    
    // BLT x2, x3, 8       // Branch if less than
    ROM[27] = 32'h02314263;
    
    // BGE x2, x3, 8       // Branch if greater than or equal
    ROM[28] = 32'h02315263;
    
    // BLTU x2, x3, 8      // Branch if less than unsigned
    ROM[29] = 32'h02316263;
    
    // BGEU x2, x3, 8      // Branch if greater than or equal unsigned
    ROM[30] = 32'h02317263;
    
    // JALR x25, x0, 0     // Jump and link register
    ROM[31] = 32'h00000C67;
    
    // LB x26, 0(x0)       // Load byte
    ROM[32] = 32'h00000D03;
    
    // LH x27, 0(x0)       // Load halfword
    ROM[33] = 32'h00001D83;
    
    // LBU x28, 0(x0)      // Load byte unsigned
    ROM[34] = 32'h00000E03;
    
    // LHU x29, 0(x0)      // Load halfword unsigned
    ROM[35] = 32'h00001E83;
    
    // SB x30, 0(x0)       // Store byte
    ROM[36] = 32'h01E00F23;
    
    // SH x31, 0(x0)       // Store halfword
    ROM[37] = 32'h01F01F23;
    
    // Infinite loop to stop execution
    ROM[38] = 32'h0000006F;  // JAL x0, 0 (infinite loop)
  end

  assign dout = ROM[addr]; // word aligned

endmodule  
