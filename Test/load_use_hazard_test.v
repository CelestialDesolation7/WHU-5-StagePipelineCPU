`timescale 1ns / 1ps

module load_use_hazard_test();

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
        
        $display("=== Load-Use Hazard Test ===");
        $display("Testing: lw x15 0(x0) -> addi x4 x0 15 -> and x7 x15 x4");
        
        // Step 1: Load x15 with 0x87654321
        $display("Time %0t: Loading x15 with 0x87654321", $time);
        instr_in = 32'h00002783; // lw x15 0(x0)
        #10;
        
        // Step 2: Set x4 to 15 (0x0F) - This should be stalled due to load-use hazard
        $display("Time %0t: Setting x4 to 15 (may be stalled)", $time);
        instr_in = 32'h00F00213; // addi x4 x0 15
        #10;
        
        // Step 3: AND x7 = x15 & x4 - This should also be stalled
        $display("Time %0t: Executing AND x7 x15 x4 (may be stalled)", $time);
        instr_in = 32'h0047F3B3; // and x7 x15 x4
        #10;
        
        // Continue with NOPs to let pipeline complete
        instr_in = 32'h00000013; // NOP
        #200; // Give more time for pipeline to complete
        
        // Check results
        $display("=== Final Results Check ===");
        
        // Check x15
        reg_sel = 5'h0F;
        #10;
        $display("x15 = 0x%h (expected: 0x87654321)", reg_data);
        
        // Check x4
        reg_sel = 5'h04;
        #10;
        $display("x4 = 0x%h (expected: 0x0000000F)", reg_data);
        
        // Check x7
        reg_sel = 5'h07;
        #10;
        $display("x7 = 0x%h (expected: 0x00000001)", reg_data);
        
        // Manual verification
        if (reg_data == 32'h00000001) begin
            $display("SUCCESS: AND instruction working correctly!");
        end else begin
            $display("FAILURE: AND instruction not working!");
            $display("Expected: 0x87654321 & 0x0000000F = 0x00000001");
            $display("Got: 0x%h", reg_data);
        end
        
        // End simulation
        #50;
        $finish;
    end
    
    // Memory simulation for load instruction
    always @(posedge clk) begin
        if (!rst && Addr_out == 32'h0 && !mem_w) begin
            // Simulate memory read at address 0
            Data_in = 32'h87654321; // The value that should be loaded into x15
            $display("Time %0t: Memory read at addr 0x0, returning 0x%h", $time, Data_in);
        end
    end
    
    // Detailed pipeline monitoring
    always @(posedge clk) begin
        if (!rst) begin
            $display("Time %0t: PC=0x%h, Instr=0x%h, MemW=%b, Addr=0x%h, DataOut=0x%h", 
                     $time, PC_out, instr_in, mem_w, Addr_out, Data_out);
        end
    end

endmodule 