// global_color_core.v
// -------------------------------------------------
// 全局色彩校正核心
// 仅实现 R' = R * GainR, B' = B * GainB
// 这是一个3级流水线模块，以匹配DSP的乘法延迟。
// -------------------------------------------------

module global_color_core #(
    // 使用您校准脚本输出的参数
    parameter GAIN_WIDTH = 12,
    parameter Q_SHIFT    = 8,
    parameter R_GAIN_FX  = 12'd468, // 1.8293
    parameter B_GAIN_FX  = 12'd492, // 1.9201
    // 内部处理位宽
    parameter DATA_WIDTH = 8
)(
    input  wire                         clk,
    input  wire                         rst_n,
    
    // Data Input (RGB888)
    input  wire                         i_valid,
    input  wire  [DATA_WIDTH-1:0]       i_r,
    input  wire  [DATA_WIDTH-1:0]       i_g,
    input  wire  [DATA_WIDTH-1:0]       i_b,
    
    // Data Output (RGB888)
    output reg                          o_valid,
    output reg   [DATA_WIDTH-1:0]       o_r,
    output reg   [DATA_WIDTH-1:0]       o_g,
    output reg   [DATA_WIDTH-1:0]       o_b
);

    // --- 流水线寄存器 ---

    // S1: 输入寄存器
    reg  [DATA_WIDTH-1:0]       r_s1, g_s1, b_s1;
    reg                         valid_s1;
    
    // S2: 乘法器输出 (DSP)
    // 8bit * 12bit = 20bit
    reg  [DATA_WIDTH+GAIN_WIDTH-1:0] r_mult_s2;
    reg  [DATA_WIDTH+GAIN_WIDTH-1:0] b_mult_s2;
    reg  [DATA_WIDTH-1:0]            g_s2;
    reg                              valid_s2;
    
    // S3: 移位和饱和 (输出)
    // 在 S2 的组合逻辑中实现，S3 即为 o_r, o_g, o_b

    // --- 流水线实现 ---

    // S1: 寄存输入
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= i_valid;
            r_s1     <= i_r;
            g_s1     <= i_g;
            b_s1     <= i_b;
        end
    end

    // S2: 执行乘法 (映射到 DSP)
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s2 <= 1'b0;
        end else begin
            valid_s2    <= valid_s1;
            g_s2        <= g_s1; // G通道直通
            r_mult_s2   <= r_s1 * R_GAIN_FX;
            b_mult_s2   <= b_s1 * B_GAIN_FX;
        end
    end
    
    // S3: 移位、饱和与输出
    wire [DATA_WIDTH+GAIN_WIDTH-1:0] r_shifted, b_shifted;
    wire [DATA_WIDTH-1:0]            r_saturated, b_saturated;
    
    // 移位 (丢弃小数部分)
    assign r_shifted = r_mult_s2 >> Q_SHIFT;
    assign b_shifted = b_mult_s2 >> Q_SHIFT;
    
    // 饱和截断 (防止 > 255)
    assign r_saturated = (r_shifted > {{(GAIN_WIDTH){1'b0}}, {DATA_WIDTH{1'b1}}}) ? {DATA_WIDTH{1'b1}} : r_shifted[DATA_WIDTH-1:0];
    assign b_saturated = (b_shifted > {{(GAIN_WIDTH){1'b0}}, {DATA_WIDTH{1'b1}}}) ? {DATA_WIDTH{1'b1}} : b_shifted[DATA_WIDTH-1:0];
    
    always @(posedge clk) begin
        if (!rst_n) begin
            o_valid <= 1'b0;
            o_r     <= 0;
            o_g     <= 0;
            o_b     <= 0;
        end else begin
            o_valid <= valid_s2;
            o_r     <= r_saturated;
            o_g     <= g_s2;
            o_b     <= b_saturated;
        end
    end

endmodule
