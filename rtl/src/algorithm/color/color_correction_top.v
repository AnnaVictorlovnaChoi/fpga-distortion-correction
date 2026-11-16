// color_correction_top.v
// -------------------------------------------------
// 色彩校正顶层模块
// 1. 接收 1080P DVP (RGB565) 输入
// 2. 将 RGB565 转换为 RGB888
// 3. 实例化 global_color_core 进行校正
// 4. 将校正后的 RGB888 转换为 RGB565
// 5. 将同步信号 (vs, hs, de) 延迟匹配
// 6. 输出 DVP (RGB565) 格式的数据流，用于【后续模块】
// -------------------------------------------------

module color_correction_top #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080
)(
    input  wire                         clk,          // 148MHz
    input  wire                         rst_n,
    
    // DVP Camera Input Interface (RGB565)
    input  wire                         i_vs,
    input  wire                         i_hs,
    input  wire                         i_de,
    input  wire  [15:0]                 i_data_rgb565,
    
    // Corrected Video Output (DVP-like, to next module)
    output wire                         o_corrected_vs,
    output wire                         o_corrected_hs,
    output wire                         o_corrected_de,
    output wire  [15:0]                 o_corrected_data_rgb565
);

    // --- 内部信号 ---
    
    // 核心模块流水线延迟 (S1 -> S2 -> S3 = 2拍)
    // S1(reg), S2(reg), S3(reg_out) = 3拍
    // (在 global_color_core 中, i_valid -> valid_s1 -> valid_s2 -> o_valid)
    // 总延迟为 3 个时钟周期
    localparam CORE_LATENCY = 3; 

    // 位置计数器 (用于调试，非必须)
    reg [10:0] x_cnt; // 0-1919
    reg [10:0] y_cnt; // 0-1079
    
    // 格式转换 (RGB565 -> RGB888)
    wire [7:0] r_in_8b, g_in_8b, b_in_8b;
    
    // Core 输出 (RGB888)
    wire [7:0] r_out_8b, g_out_8b, b_out_8b;
    wire       core_out_valid;
    
    // 同步信号延迟线
    reg  [CORE_LATENCY-1:0] vs_pipe, hs_pipe, de_pipe;
    
    
    // --- 1. DVP 输入同步与位置计数 ---
    always @(posedge clk) begin
        if (!rst_n) begin
            x_cnt <= 0;
            y_cnt <= 0;
        end else if (i_de) begin
            if (x_cnt == IMG_WIDTH - 1) begin
                x_cnt <= 0;
                y_cnt <= (y_cnt == IMG_HEIGHT - 1) ? 0 : y_cnt + 1;
            end else begin
                x_cnt <= x_cnt + 1;
            end
        end else if (~i_vs) begin // Reset on vsync low
             x_cnt <= 0;
             y_cnt <= 0;
        end
    end

    // --- 2. 输入格式转换: RGB565 -> RGB888 ---
    // (R[4:0] -> R[7:0] = {R[4:0], R[4:2]})
    assign r_in_8b = {i_data_rgb565[15:11], i_data_rgb565[15:13]};
    assign g_in_8b = {i_data_rgb565[10:5],  i_data_rgb565[10:9]};
    assign b_in_8b = {i_data_rgb565[4:0],   i_data_rgb565[4:2]};


    // --- 3. 核心模块实例化 ---
    // (使用您校准的参数)
    global_color_core #(
        .R_GAIN_FX  (12'd468),
        .B_GAIN_FX  (12'd492)
    ) u_core (
        .clk      (clk),
        .rst_n    (rst_n),
        
        .i_valid  (i_de),
        .i_r      (r_in_8b),
        .i_g      (g_in_8b),
        .i_b      (b_in_8b),
        
        .o_valid  (core_out_valid),
        .o_r      (r_out_8b),
        .o_g      (g_out_8b),
        .o_b      (b_out_8b)
    );

    // --- 4. 同步信号流水线 (匹配核心延迟) ---
    always @(posedge clk) begin
        if (!rst_n) begin
            vs_pipe <= 0;
            hs_pipe <= 0;
            de_pipe <= 0;
        end else begin
            vs_pipe <= {vs_pipe[CORE_LATENCY-2:0], i_vs};
            hs_pipe <= {hs_pipe[CORE_LATENCY-2:0], i_hs};
            de_pipe <= {de_pipe[CORE_LATENCY-2:0], i_de};
        end
    end

    // --- 5. 输出格式转换: RGB888 -> RGB565 ---
    // R(8bit)->R(5bit), G(8bit)->G(6bit), B(8bit)->B(5bit)
    assign o_corrected_data_rgb565 = {r_out_8b[7:3], g_out_8b[7:2], b_out_8b[7:3]};
    
    // --- 6. 连接到输出接口 ---
    // 确保 o_valid (来自core) 与延迟后的 de (来自pipeline) 保持一致
    // (注意: u_core 内部的 o_valid 已经是 i_de 延迟 CORE_LATENCY 拍的结果)
    assign o_corrected_vs = vs_pipe[CORE_LATENCY-1];
    assign o_corrected_hs = hs_pipe[CORE_LATENCY-1];
    assign o_corrected_de = core_out_valid; // de_pipe[CORE_LATENCY-1];

endmodule
