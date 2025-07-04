// FPGA顶层模块 - 用于在FPGA上实现RISC-V流水线CPU
// 符合Nexys4DDR约束文件的接口定义
module top_module(
    input clk,                    // 100MHz系统时钟
    input rstn,                   // 复位信号，低电平有效
    input [15:0] sw_i,           // 16个开关输入
    // output [15:0] led_o,         // 16个LED输出
    output [7:0] disp_seg_o,     // 七段数码管段选信号
    output [7:0] disp_an_o       // 七段数码管位选信号
);

    // 参数定义
    parameter REG_DATA_NUM = 5'd31;   // 寄存器数据数量
    parameter DMEM_DATA_NUM = 4'd15;  // 数据存储器数据数量

    // 内部信号定义
    reg [27:0] clkdiv;           // 时钟分频计数器
    wire Clk_CPU;                // CPU时钟
    wire Clk_instr;              // 指令执行时钟
    wire Clk_display;            // 数码管显示时钟（独立高频时钟）
    
    // CPU接口信号
    wire [31:0] PC_out;          // 程序计数器输出
    wire [31:0] instr;           // 当前指令
    wire [31:0] reg_data;        // 寄存器数据
    wire [31:0] mem_addr_out;    // 内存访问地址输出
    wire [31:0] mem_data_out;    // 内存访问数据输出
    wire [31:0] debug_data;      // 调试数据
    
    // 统一显示控制信号
    reg [4:0] reg_addr;          // 寄存器地址
    reg [3:0] dmem_addr;         // 数据存储器地址
    reg sw3_last, sw4_last;      // 开关状态记录
    
    // 显示数据信号
    reg [31:0] reg_data_from_cpu; // 从CPU读取的寄存器数据
    reg [31:0] dmem_data;        // 数据存储器数据
    reg [31:0] access_addr_data; // 访问地址数据
    reg [31:0] display_data;     // 最终显示数据

    // 时钟分频逻辑
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            clkdiv <= 0;
        else
            clkdiv <= clkdiv + 1'b1;
    end

    // 根据 sw_i[15] 控制时钟分频速率
    assign Clk_CPU = (sw_i[15]) ? clkdiv[27] : clkdiv[20];
    assign Clk_instr = Clk_CPU & ~sw_i[1]; // CPU工作时钟与控制信号 sw_i[1] 结合
    
    // 数码管独立时钟 - 使用较高频率确保刷新速度足够快
    assign Clk_display = clkdiv[16];
    
    // 实例化RISC-V流水线CPU
    sccomp cpu(
        .clk(Clk_instr),         // 使用分频后的时钟
        .rstn(rstn),
        .reg_sel(reg_addr),      // 寄存器选择
        .reg_data(reg_data),
        .instr(instr),           // 当前指令
        .PC_out(PC_out),         // 程序计数器
        .mem_addr_out(mem_addr_out),  // 内存访问地址
        .mem_data_out(mem_data_out),  // 内存访问数据
        .debug_data(debug_data)  // 调试数据输出
    );
    
    // 统一显示控制逻辑 - 使用开关234控制，不受CPU时钟影响
    always @ (posedge clk or negedge rstn) begin
        if (!rstn) begin
            reg_addr <= 5'b0;
            dmem_addr <= 4'b0;
            sw3_last <= 1'b0;
            sw4_last <= 1'b0;
        end else begin
            sw3_last <= sw_i[3];
            sw4_last <= sw_i[4];
            
            // 开关2：重置所有地址到0
            if (sw_i[2]) begin
                reg_addr <= 5'b0;
                dmem_addr <= 4'b0;
            end 
            // 开关3：地址递增
            else if (sw_i[3] && !sw3_last) begin
                // 寄存器地址递增
                if (reg_addr == REG_DATA_NUM)
                    reg_addr <= 5'd0;
                else
                    reg_addr <= reg_addr + 1'b1;
                
                // 数据存储器地址递增
                if (dmem_addr == DMEM_DATA_NUM)
                    dmem_addr <= 4'd0;
                else
                    dmem_addr <= dmem_addr + 1'b1;
            end 
            // 开关4：地址递减
            else if (sw_i[4] && !sw4_last) begin
                // 寄存器地址递减
                if (reg_addr == 5'd0)
                    reg_addr <= REG_DATA_NUM;
                else
                    reg_addr <= reg_addr - 1'b1;
                
                // 数据存储器地址递减
                if (dmem_addr == 4'd0)
                    dmem_addr <= DMEM_DATA_NUM;
                else
                    dmem_addr <= dmem_addr - 1'b1;
            end
        end
    end
    
    // 寄存器数据读取
    always @(*) begin
        reg_data_from_cpu = reg_data;
    end
    
    // 数据存储器数据
    always @(*) begin
        dmem_data = mem_data_out;  // 显示内存访问数据
    end
    
    // 访问地址数据 - 如果没有发生读取或写入，输出FFFFFFFF
    always @(*) begin
        // 检查是否有内存访问（地址不为0或者有实际的内存操作）
        if (mem_addr_out != 32'h0 || mem_data_out != 32'h0) begin
            access_addr_data = mem_addr_out;
        end else begin
            access_addr_data = 32'hFFFFFFFF;  // 没有涉及地址时输出FFFFFFFF
        end
    end
    
    // 根据开关输入选择显示的数据
    always @(*) begin
        if(sw_i[10] == 1'b1) begin
            // 调试显示模式
            display_data = debug_data;
        end else begin
            // 正常显示模式
            case(sw_i[14:11])
                4'b1000: display_data = instr;           // 指令
                4'b0100: display_data = reg_data_from_cpu; // 寄存器数据
                4'b0010: display_data = access_addr_data; // 访问地址数据
                4'b0001: display_data = dmem_data;       // 数据存储器数据
                4'b1001: display_data = PC_out;          // 程序计数器
                default: display_data = PC_out;          // 默认显示PC
            endcase
        end
    end
    
    // 七段数码管控制器
    seg7_controller seg7_ctrl(
        .clk(Clk_display),
        .rstn(rstn),
        .data(display_data),
        .seg(disp_seg_o),
        .an(disp_an_o)
    );

endmodule

// 七段数码管控制器模块
module seg7_controller(
    input clk,
    input rstn,
    input [31:0] data,
    output reg [7:0] seg,
    output reg [7:0] an
);
    
    reg [2:0] digit_sel;
    reg [3:0] digit_data;
    
    // 数码管选择计数器
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            digit_sel <= 3'b000;
        else
            digit_sel <= digit_sel + 1;
    end
    
    // 根据选择信号输出对应的数字
    always @(*) begin
        case (digit_sel)
            3'b000: digit_data = data[3:0];   // 第0位
            3'b001: digit_data = data[7:4];   // 第1位
            3'b010: digit_data = data[11:8];  // 第2位
            3'b011: digit_data = data[15:12]; // 第3位
            3'b100: digit_data = data[19:16]; // 第4位
            3'b101: digit_data = data[23:20]; // 第5位
            3'b110: digit_data = data[27:24]; // 第6位
            3'b111: digit_data = data[31:28]; // 第7位
        endcase
    end
    
    // 位选信号（8位）- 共阴极，低电平有效
    always @(*) begin
        case (digit_sel)
            3'b000: an = 8'b11111110; // 选择第0位
            3'b001: an = 8'b11111101; // 选择第1位
            3'b010: an = 8'b11111011; // 选择第2位
            3'b011: an = 8'b11110111; // 选择第3位
            3'b100: an = 8'b11101111; // 选择第4位
            3'b101: an = 8'b11011111; // 选择第5位
            3'b110: an = 8'b10111111; // 选择第6位
            3'b111: an = 8'b01111111; // 选择第7位
        endcase
    end
    
    // 段选信号（共阴极）- 低电平点亮对应段
    always @(*) begin
        case (digit_data)
            4'h0: seg = 8'b11000000;  // 0 - abcdef点亮
            4'h1: seg = 8'b11111001;  // 1 - bc点亮
            4'h2: seg = 8'b10100100;  // 2 - abdeg点亮
            4'h3: seg = 8'b10110000;  // 3 - abcdg点亮
            4'h4: seg = 8'b10011001;  // 4 - bcfg点亮
            4'h5: seg = 8'b10010010;  // 5 - acdfg点亮
            4'h6: seg = 8'b10000010;  // 6 - acdefg点亮
            4'h7: seg = 8'b11111000;  // 7 - abc点亮
            4'h8: seg = 8'b10000000;  // 8 - abcdefg点亮
            4'h9: seg = 8'b10010000;  // 9 - abcdfg点亮
            4'hA: seg = 8'b10001000;  // A - abcefg点亮
            4'hB: seg = 8'b10000011;  // b - cdefg点亮
            4'hC: seg = 8'b11000110;  // C - adef点亮
            4'hD: seg = 8'b10100001;  // d - bcdeg点亮
            4'hE: seg = 8'b10000110;  // E - adefg点亮
            4'hF: seg = 8'b10001110;  // F - aefg点亮
            default: seg = 8'b11111111; // 熄灭 - 所有段都不点亮
        endcase
    end

endmodule 