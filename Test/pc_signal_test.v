`timescale 1ns / 1ps

module pc_signal_test();

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
        
        $display("=== PC Signal Test ===");
        $display("Time %0t: Starting test", $time);
        
        // Hold reset for 20 cycles
        #100;
        rst = 0;
        
        $display("Time %0t: Reset released", $time);
        
        // Send a simple instruction
        instr_in = 32'h00000013; // NOP
        #10;
        
        // Send another instruction
        instr_in = 32'h00100093; // addi x1 x0 1
        #10;
        
        // Send another instruction
        instr_in = 32'h00200113; // addi x2 x0 2
        #10;
        
        // Continue with NOPs
        instr_in = 32'h00000013; // NOP
        #100;
        
        $display("Time %0t: Test completed", $time);
        $finish;
    end
    
    // Monitor PC signals
    always @(posedge clk) begin
        if (!rst) begin
            $display("Time %0t: PC_out=0x%h, PC_IF=0x%h", $time, PC_out, debug_data);
        end
    end

endmodule 