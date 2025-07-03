`timescale 1ns / 1ps

module simple_and_test();

    // Clock and reset signals
    reg clk;
    reg rst;
    
    // CPU interface signals
    wire [31:0] PC_out;
    wire [31:0] Addr_out;
    wire [31:0] Data_out;
    wire mem_w;
    wire [2:0] DMType_out;
    wire [31:0] debug_data;
    
    // Memory interface
    reg [31:0] Data_in;
    reg [31:0] instr_in;
    
    // Register monitoring
    wire [31:0] reg_data;
    reg [4:0] reg_sel;
    
    // Instantiate the pipeline CPU
    PipelineCPU cpu(
        .clk(clk),
        .rst(rst),
        .Data_in(Data_in),
        .instr_in(instr_in),
        .PC_out(PC_out),
        .Addr_out(Addr_out),
        .Data_out(Data_out),
        .mem_w(mem_w),
        .DMType_out(DMType_out),
        .debug_data(debug_data),
        .reg_sel(reg_sel),
        .reg_data(reg_data)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test stimulus
    initial begin
        // Initialize signals
        rst = 1;
        Data_in = 32'h0;
        instr_in = 32'h00000013; // NOP
        reg_sel = 5'h0;
        
        // Release reset after 10 cycles
        #50;
        rst = 0;
        
        // Wait for reset to complete
        #20;
        
        $display("=== Simple AND Test (No Load-Use Hazard) ===");
        
        // Test 1: Set x15 to 0x87654321 using lui + addi
        $display("Time %0t: Setting x15 to 0x87654321", $time);
        instr_in = 32'h87654000; // lui x15 0x87654
        #10;
        instr_in = 32'h32178793; // addi x15 x15 0x321
        #10;
        
        // Test 2: Set x4 to 15 using addi
        $display("Time %0t: Setting x4 to 15", $time);
        instr_in = 32'h00F00213; // addi x4 x0 15
        #10;
        
        // Test 3: AND x7 = x15 & x4
        $display("Time %0t: Executing AND x7 x15 x4", $time);
        instr_in = 32'h0047F3B3; // and x7 x15 x4
        #10;
        
        // Continue with NOPs to let pipeline complete
        instr_in = 32'h00000013; // NOP
        #100;
        
        // Check results
        $display("=== Results Check ===");
        
        // Check x15
        reg_sel = 5'h0F;
        #10;
        $display("x15 = 0x%h", reg_data);
        
        // Check x4
        reg_sel = 5'h04;
        #10;
        $display("x4 = 0x%h", reg_data);
        
        // Check x7
        reg_sel = 5'h07;
        #10;
        $display("x7 = 0x%h (expected: 0x00000001)", reg_data);
        
        // Test 2: Direct AND with immediate (andi)
        $display("=== Testing ANDI instruction ===");
        instr_in = 32'h00F7F393; // andi x7 x15 15
        #10;
        instr_in = 32'h00000013; // NOP
        #50;
        
        reg_sel = 5'h07;
        #10;
        $display("x7 (after ANDI) = 0x%h (expected: 0x00000001)", reg_data);
        
        // End simulation
        #50;
        $finish;
    end
    
    // Monitor pipeline behavior
    always @(posedge clk) begin
        if (!rst) begin
            $display("Time %0t: PC=0x%h, Instr=0x%h", $time, PC_out, instr_in);
        end
    end

endmodule 