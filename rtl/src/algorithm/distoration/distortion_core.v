// distortion_core.v

// -----------------------------------------------------------------------------
// 畸变校正核心 (V6.7 - "过暗" & 截断 Bug 最终修复版)
// 模块名称：distortion_core (Distortion Correction Core)
// 基本功能：实现图像畸变校正的核心处理模块，接收输出像素坐标，通过查找表（LUT）获取畸变校正映射坐标，结合行缓存读取的源像素数据，经双线性插值计算输出校正后的像素，适配 1920x1080 分辨率图像。
// 基本介绍：
// 采用流水线设计，包含多阶段处理（S1 至 S10），实现从输入坐标到输出校正像素的完整流程；
// 关键参数配置：支持坐标小数位（Q_COORD_FRAC=5）、权重小数位（Q_WEIGHT_FRAC=4），适配网格下采样（MAP_DOWNSAMPLE_LOG2=4）；
// 核心处理逻辑：通过 LUT 获取 4 个角点映射坐标，经双线性插值（L1 和 L2 两级权重计算）生成最终校正像素，权重计算采用 "减一法" 避免截断误差；
// 适配外部接口：输出 LUT 读取地址和行缓存地址，接收 LUT 返回的映射坐标和行缓存返回的源像素数据；
// 修正记录：
// (V6.2) 修正 o_valid 逻辑以匹配 DDR；
// (V6.4) 增加 x_map_a_s3 寄存器，修复 L1 数据 / 权重 1-clock 对齐问题；
// (V6.5) 修复 L2 边界（黑点）问题（addr_valid_comb=1，边缘复制）；
// (V6.7) 关键修复：回退权重扩展，采用 "减一法" 修复 L1 和 L2 截断 Bug，解决 "过暗"、"网格黑线" 和 "零散横线" 问题。
// 设计人员：王博
// 最后修改时间：2025-11-05
// -----------------------------------------------------------------------------

module distortion_core #(
    parameter IMG_WIDTH = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter MAP_DOWNSAMPLE_LOG2 = 4,
    parameter Q_COORD_FRAC = 5,
    parameter Q_WEIGHT_FRAC = 4
)(
    input wire clk,
    input wire rst_n,

    // 当前输出像素的位置 (x_out, y_out)
    input wire i_valid,
    input wire [10:0] i_x, // 代表输出坐标 X
    input wire [10:0] i_y, // 代表输出坐标 Y

    // 从LUT ROM读回的4个角点坐标 (Q11.5)
    input wire signed [15:0] i_x_map_a, i_y_map_a,
    input wire signed [15:0] i_x_map_b, i_y_map_b,
    input wire signed [15:0] i_x_map_c, i_y_map_c,
    input wire signed [15:0] i_x_map_d, i_y_map_d,

    // 从行缓存读回的4个源像素颜色 (RGB565)
    input wire [15:0] i_pixel_a, // 来自 (x_int, y_int)
    input wire [15:0] i_pixel_b, // 来自 (x_int+1, y_int)
    input wire [15:0] i_pixel_c, // 来自 (x_int, y_int+1)
    input wire [15:0] i_pixel_d, // 来自 (x_int+1, y_int+1)

    // 输出到外部模块
    output reg [14:0] o_lut_addr_a, o_lut_addr_b, o_lut_addr_c, o_lut_addr_d,
    output reg [10:0] o_linebuf_addr_x0, o_linebuf_addr_x1,
    output reg [10:0] o_linebuf_addr_y0, o_linebuf_addr_y1,

    // 输出同步坐标 (与 o_pixel_data 同步)
    output reg [10:0] o_x_out, 
    output reg [10:0] o_y_out, 
    // 最终输出
    output reg o_valid,
    output reg [15:0] o_pixel_data
);

    // --- Q格式常量 (V6.7 回退) ---
    localparam L1_FRAC_BITS = Q_COORD_FRAC + Q_WEIGHT_FRAC * 2; // 5 + 4*2 = 13
    localparam L2_WEIGHT_BITS = Q_COORD_FRAC;                 // 5
    localparam L2_FRAC_BITS = L2_WEIGHT_BITS * 2;             // 10

    // --- 流水线寄存器 ---
    reg v_s1, v_s2, v_s3, v_s4, v_s5, v_s6, v_s7, v_s8, v_s9, v_s10;
    reg [10:0] x_s1, y_s1, x_s2, y_s2, x_s3, y_s3, x_s4, y_s4, x_s5, y_s5;
    reg [10:0] x_s6, y_s6, x_s7, y_s7, x_s8, y_s8, x_s9, y_s9, x_s10, y_s10;
    
    // L1 (权重) - (V6.7 回退到 4 bits)
    reg [Q_WEIGHT_FRAC-1:0] cw_x_s2, cw_y_s2, cw_xi_s2, cw_yi_s2;
    reg [Q_WEIGHT_FRAC-1:0] cw_x_s3, cw_y_s3, cw_xi_s3, cw_yi_s3;

    // L1 (LUT 数据) - (V6.4 修复, 保持)
    reg signed [15:0] x_map_a_s3, y_map_a_s3;
    reg signed [15:0] x_map_b_s3, y_map_b_s3;
    reg signed [15:0] x_map_c_s3, y_map_c_s3;
    reg signed [15:0] x_map_d_s3, y_map_d_s3;
    
    // L1 (中间值) - (V6.7 回退到 4-bit 权重数学)
    reg signed [15+Q_WEIGHT_FRAC:0] x_h1_s4, x_h2_s4, y_h1_s4, y_h2_s4; // 20 bits
    reg [Q_WEIGHT_FRAC-1:0]         cw_y_s4, cw_yi_s4; // 4 bits
    
    // L2
    reg signed [15+L1_FRAC_BITS:0] src_x_fx_s5, src_y_fx_s5; // 29 bits
    
    // L2 (权重) - (V6.7 回退到 5 bits)
    reg [L2_WEIGHT_BITS-1:0] pw_x_s6, pw_xi_s6, pw_y_s6, pw_yi_s6;
    reg [L2_WEIGHT_BITS-1:0] pw_x_s7, pw_xi_s7, pw_y_s7, pw_yi_s7;
    reg [4:0] r_a_s8, r_b_s8, r_c_s8, r_d_s8;
    reg [5:0] g_a_s8, g_b_s8, g_c_s8, g_d_s8;
    reg [4:0] b_a_s8, b_b_s8, b_c_s8, b_d_s8;
    reg [L2_WEIGHT_BITS-1:0] pw_x_s8, pw_xi_s8, pw_y_s8, pw_yi_s8;
    
    // L2 (中间值) - (V6.7 回退到 5-bit 权重数学)
    reg [4+L2_WEIGHT_BITS:0] r_h1_s9, r_h2_s9; // 10 bits
    reg [5+L2_WEIGHT_BITS:0] g_h1_s9, g_h2_s9; // 11 bits
    reg [4+L2_WEIGHT_BITS:0] b_h1_s9, b_h2_s9; // 10 bits
    reg [L2_WEIGHT_BITS-1:0] pw_y_s9, pw_yi_s9; // 5 bits
    
    reg addr_valid_pipe [0:4]; 

    // --- 组合逻辑 ---
    // S1: ** 【V6.7 网格裂纹修复】 **
    localparam GRID_ROM_WIDTH = IMG_WIDTH >> MAP_DOWNSAMPLE_LOG2;
    wire [7:0] grid_x = x_s1 >> MAP_DOWNSAMPLE_LOG2; 
    wire [7:0] grid_y = y_s1 >> MAP_DOWNSAMPLE_LOG2;
    
    // 1. 获取 4-bit 权重 (0-15)
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_x = x_s1[Q_WEIGHT_FRAC-1:0];
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_y = y_s1[Q_WEIGHT_FRAC-1:0];
    // 2. 计算 4-bit 反向权重 (0-15)
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_x_inv = (1 << Q_WEIGHT_FRAC) - 1 - coord_weight_x; // 15 - w
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_y_inv = (1 << Q_WEIGHT_FRAC) - 1 - coord_weight_y; // 15 - w
    
    wire [14:0] base_addr = grid_y * GRID_ROM_WIDTH + grid_x;
    
    // S3: ** 【V6.4 修复】: 使用 4-bit 权重 和 V6.4的 _s3 寄存器 **
    wire signed [15+Q_WEIGHT_FRAC:0] x_h1 = x_map_a_s3 * cw_xi_s3 + x_map_b_s3 * cw_x_s3;
    wire signed [15+Q_WEIGHT_FRAC:0] x_h2 = x_map_c_s3 * cw_xi_s3 + x_map_d_s3 * cw_x_s3;
    wire signed [15+Q_WEIGHT_FRAC:0] y_h1 = y_map_a_s3 * cw_yi_s3 + y_map_b_s3 * cw_y_s3;
    wire signed [15+Q_WEIGHT_FRAC:0] y_h2 = y_map_c_s3 * cw_yi_s3 + y_map_d_s3 * cw_y_s3;

    // S4:
    wire signed [15+L1_FRAC_BITS:0] src_x_fx_s4_out = (x_h1_s4 * cw_yi_s4) + (x_h2_s4 * cw_y_s4);
    wire signed [15+L1_FRAC_BITS:0] src_y_fx_s4_out = (y_h1_s4 * cw_yi_s4) + (y_h2_s4 * cw_y_s4);
    
    // S7: (不变)
    wire [4:0] r_a = i_pixel_a[15:11], r_b = i_pixel_b[15:11], r_c = i_pixel_c[15:11], r_d = i_pixel_d[15:11];
    wire [5:0] g_a = i_pixel_a[10:5],  g_b = i_pixel_b[10:5],  g_c = i_pixel_c[10:5],  g_d = i_pixel_d[10:5];
    wire [4:0] b_a = i_pixel_a[4:0],   b_b = i_pixel_b[4:0],   b_c = i_pixel_c[4:0],   b_d = i_pixel_d[4:0];
    
    // S8: ** 【V6.7 回退】: 5-bit 权重 **
    wire [4+L2_WEIGHT_BITS:0] r_h1 = r_a_s8 * pw_xi_s8 + r_b_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] r_h2 = r_c_s8 * pw_xi_s8 + r_d_s8 * pw_x_s8;
    wire [5+L2_WEIGHT_BITS:0] g_h1 = g_a_s8 * pw_xi_s8 + g_b_s8 * pw_x_s8;
    wire [5+L2_WEIGHT_BITS:0] g_h2 = g_c_s8 * pw_xi_s8 + g_d_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] b_h1 = b_a_s8 * pw_xi_s8 + b_b_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] b_h2 = b_c_s8 * pw_xi_s8 + b_d_s8 * pw_x_s8;
    
    // S9: ** 【V6.7 回退】: 10-bit 小数位 **
    wire [4+L2_FRAC_BITS:0] r_temp = (r_h1_s9 * pw_yi_s9) + (r_h2_s9 * pw_y_s9);
    wire [5+L2_FRAC_BITS:0] g_temp = (g_h1_s9 * pw_yi_s9) + (g_h2_s9 * pw_y_s9);
    wire [4+L2_FRAC_BITS:0] b_temp = (b_h1_s9 * pw_yi_s9) + (b_h2_s9 * pw_y_s9);
    // (移位 10 bits)
    wire [4+L2_FRAC_BITS-L2_FRAC_BITS:0] r_shifted = r_temp >> L2_FRAC_BITS;
    wire [5+L2_FRAC_BITS-L2_FRAC_BITS:0] g_shifted = g_temp >> L2_FRAC_BITS;
    wire [4+L2_FRAC_BITS-L2_FRAC_BITS:0] b_shifted = b_temp >> L2_FRAC_BITS;
    // (饱和)
    wire [4:0] r_final = (r_shifted > 5'd31) ? 5'd31 : r_shifted[4:0];
    wire [5:0] g_final = (g_shifted > 6'd63) ? 6'd63 : g_shifted[5:0];
    wire [4:0] b_final = (b_shifted > 5'd31) ? 5'd31 : b_shifted[4:0];
    
    // --- S5: 边界钳位与地址计算 (V6.7 修复) ---
    localparam MAX_X_FX = (IMG_WIDTH - 1) << L1_FRAC_BITS;
    localparam MAX_Y_FX = (IMG_HEIGHT - 1) << L1_FRAC_BITS;
    wire signed [15+L1_FRAC_BITS:0] clamped_src_x_fx_s5 = (src_x_fx_s5 < 0) ? 0 : (src_x_fx_s5 > MAX_X_FX) ? MAX_X_FX : src_x_fx_s5;
    wire signed [15+L1_FRAC_BITS:0] clamped_src_y_fx_s5 = (src_y_fx_s5 < 0) ? 0 : (src_y_fx_s5 > MAX_Y_FX) ? MAX_Y_FX : src_y_fx_s5;
    
    wire [10:0] x_int = clamped_src_x_fx_s5 >> L1_FRAC_BITS;
    wire [10:0] y_int = clamped_src_y_fx_s5 >> L1_FRAC_BITS;

    // (V6.5 黑点修复)
    wire addr_valid_comb = 1'b1;
    wire [10:0] x_int_plus_1 = (x_int >= IMG_WIDTH - 1) ? x_int : x_int + 1;
    wire [10:0] y_int_plus_1 = (y_int >= IMG_HEIGHT - 1) ? y_int : y_int + 1;
    
    wire [L1_FRAC_BITS-1:0] clamped_frac_x = clamped_src_x_fx_s5[L1_FRAC_BITS-1:0];
    wire [L1_FRAC_BITS-1:0] clamped_frac_y = clamped_src_y_fx_s5[L1_FRAC_BITS-1:0];
    
    // ** 【V6.7 零散横线修复】: L2 权重计算 **
    // 1. 从 L1 (13b) 移位到 L2 (5b)
    wire [L2_WEIGHT_BITS-1:0] l2_weight_x = clamped_frac_x >> (L1_FRAC_BITS - L2_WEIGHT_BITS);
    wire [L2_WEIGHT_BITS-1:0] l2_weight_y = clamped_frac_y >> (L1_FRAC_BITS - L2_WEIGHT_BITS);
    // 2. 计算 5-bit 反向权重 (0-31)
    wire [L2_WEIGHT_BITS-1:0] l2_weight_x_inv = (1 << L2_WEIGHT_BITS) - 1 - l2_weight_x; // 31 - w
    wire [L2_WEIGHT_BITS-1:0] l2_weight_y_inv = (1 << L2_WEIGHT_BITS) - 1 - l2_weight_y; // 31 - w
    
    wire final_addr_valid = addr_valid_pipe[4];
    
    integer i; 
    // --- 主时序逻辑 ---
    always @(posedge clk) begin
        if (!rst_n) begin
            v_s1 <= 1'b0; v_s2 <= 1'b0; v_s3 <= 1'b0; v_s4 <= 1'b0; v_s5 <= 1'b0;
            v_s6 <= 1'b0; v_s7 <= 1'b0; v_s8 <= 1'b0; v_s9 <= 1'b0; v_s10 <= 1'b0;
            for (i=0; i<5; i=i+1) begin addr_valid_pipe[i] <= 1'b0; end
            o_valid <= 1'b0; o_pixel_data <= 16'd0;
            o_x_out <= 0; o_y_out <= 0;
            
            // (复位 V6.7)
            cw_x_s2 <= 0; cw_y_s2 <= 0; cw_xi_s2 <= 0; cw_yi_s2 <= 0;
            cw_x_s3 <= 0; cw_y_s3 <= 0; cw_xi_s3 <= 0; cw_yi_s3 <= 0;
            x_map_a_s3 <= 0; y_map_a_s3 <= 0; x_map_b_s3 <= 0; y_map_b_s3 <= 0;
            x_map_c_s3 <= 0; y_map_c_s3 <= 0; x_map_d_s3 <= 0; y_map_d_s3 <= 0;
            x_h1_s4 <= 0; x_h2_s4 <= 0; y_h1_s4 <= 0; y_h2_s4 <= 0;
            cw_y_s4 <= 0; cw_yi_s4 <= 0;
            src_x_fx_s5 <= 0; src_y_fx_s5 <= 0;
            
            pw_x_s6 <= 0; pw_xi_s6 <= 0; pw_y_s6 <= 0; pw_yi_s6 <= 0;
            pw_x_s7 <= 0; pw_xi_s7 <= 0; pw_y_s7 <= 0; pw_yi_s7 <= 0;
            
            r_a_s8 <= 0; r_b_s8 <= 0; r_c_s8 <= 0; r_d_s8 <= 0;
            g_a_s8 <= 0; g_b_s8 <= 0; g_c_s8 <= 0; g_d_s8 <= 0;
            b_a_s8 <= 0; b_b_s8 <= 0; b_c_s8 <= 0; b_d_s8 <= 0;
            pw_x_s8 <= 0; pw_xi_s8 <= 0; pw_y_s8 <= 0; pw_yi_s8 <= 0;
            
            r_h1_s9 <= 0; r_h2_s9 <= 0;
            g_h1_s9 <= 0; g_h2_s9 <= 0;
            b_h1_s9 <= 0; b_h2_s9 <= 0;
            pw_y_s9 <= 0; pw_yi_s9 <= 0;
        end else begin
            // S1: Register Input Valid and Coords
            v_s1 <= i_valid;
            if(i_valid) begin x_s1 <= i_x; y_s1 <= i_y; end

            // S2
            v_s2 <= v_s1;
            if(v_s1) begin
                x_s2 <= x_s1; y_s2 <= y_s1;
                // (V6.7) 锁存 4-bit L1 权重
                cw_x_s2 <= coord_weight_x; cw_y_s2 <= coord_weight_y;
                cw_xi_s2 <= coord_weight_x_inv; cw_yi_s2 <= coord_weight_y_inv;
            end
            
            // S3
            v_s3 <= v_s2;
            if(v_s2) begin
                x_s3 <= x_s2; y_s3 <= y_s2;
                // (V6.7) 锁存 4-bit L1 权重
                cw_x_s3 <= cw_x_s2; cw_y_s3 <= cw_y_s2;
                cw_xi_s3 <= cw_xi_s2; cw_yi_s3 <= cw_yi_s2;
                
                // (V6.4) 锁存 L1 LUT 数据
                x_map_a_s3 <= i_x_map_a; y_map_a_s3 <= i_y_map_a;
                x_map_b_s3 <= i_x_map_b; y_map_b_s3 <= i_y_map_b;
                x_map_c_s3 <= i_x_map_c; y_map_c_s3 <= i_y_map_c;
                x_map_d_s3 <= i_x_map_d; y_map_d_s3 <= i_y_map_d;
            end

            // S4
            v_s4 <= v_s3;
            if(v_s3) begin
                x_s4 <= x_s3; y_s4 <= y_s3;
                x_h1_s4 <= x_h1; x_h2_s4 <= x_h2;
                y_h1_s4 <= y_h1; y_h2_s4 <= y_h2;
                cw_y_s4 <= cw_y_s3; cw_yi_s4 <= cw_yi_s3;
            end
            
            // S5
            v_s5 <= v_s4;
            if(v_s4) begin
                x_s5 <= x_s4; y_s5 <= y_s4;
                src_x_fx_s5 <= src_x_fx_s4_out;
                src_y_fx_s5 <= src_y_fx_s4_out;
                addr_valid_pipe[0] <= addr_valid_comb; // (V6.5: 恒为 1'b1)
            end
            
            // S6
            v_s6 <= v_s5;
            if(v_s5) begin
                x_s6 <= x_s5; y_s6 <= y_s5;
                // (V6.7) 锁存 5-bit L2 权重
                pw_x_s6 <= l2_weight_x; pw_xi_s6 <= l2_weight_x_inv;
                pw_y_s6 <= l2_weight_y; pw_yi_s6 <= l2_weight_y_inv;
                addr_valid_pipe[1] <= addr_valid_pipe[0];
            end
            
            // S7
            v_s7 <= v_s6;
            if(v_s6) begin
                x_s7 <= x_s6; y_s7 <= y_s6;
                // (V6.7) 传递 5-bit L2 权重
                pw_x_s7 <= pw_x_s6; pw_xi_s7 <= pw_xi_s6;
                pw_y_s7 <= pw_y_s6; pw_yi_s7 <= pw_yi_s6;
                addr_valid_pipe[2] <= addr_valid_pipe[1];
            end

            // S8
            v_s8 <= v_s7;
            if(v_s7) begin
                x_s8 <= x_s7; y_s8 <= y_s7;
                r_a_s8 <= r_a; r_b_s8 <= r_b; r_c_s8 <= r_c; r_d_s8 <= r_d;
                g_a_s8 <= g_a; g_b_s8 <= g_b; g_c_s8 <= g_c; g_d_s8 <= g_d;
                b_a_s8 <= b_a; b_b_s8 <= b_b; b_c_s8 <= b_c; b_d_s8 <= b_d;
                // (V6.7) 传递 5-bit L2 权重
                pw_x_s8 <= pw_x_s7; pw_xi_s8 <= pw_xi_s7;
                pw_y_s8 <= pw_y_s7; pw_yi_s8 <= pw_yi_s7;
                addr_valid_pipe[3] <= addr_valid_pipe[2];
            end

            // S9
            v_s9 <= v_s8;
            if(v_s8) begin
                x_s9 <= x_s8; y_s9 <= y_s8;
                // (V6.7) 锁存 L2 中间值
                r_h1_s9 <= r_h1; r_h2_s9 <= r_h2; g_h1_s9 <= g_h1; g_h2_s9 <= g_h2;
                b_h1_s9 <= b_h1; b_h2_s9 <= b_h2; 
                // (V6.7) 传递 5-bit L2 权重
                pw_y_s9 <= pw_y_s8; pw_yi_s9 <= pw_yi_s8;
                addr_valid_pipe[4] <= addr_valid_pipe[3];
            end
            
            // S10: Output Stage
            v_s10 <= v_s9;
            if(v_s9) begin
                 x_s10 <= x_s9; y_s10 <= y_s9;
                 // (V6.5 修复: final_addr_valid 恒为 1'b1)
                 if (final_addr_valid) begin 
                     o_pixel_data <= {r_final, g_final, b_final}; 
                 end else begin 
                     o_pixel_data <= 16'h0000;
                 end
            end else begin
                o_pixel_data <= 16'h0000;
            end

            // Assign final outputs
            o_valid <= v_s10; // (V6.2: 保持流水线同步)
            
            o_x_out <= x_s10;
            o_y_out <= y_s10;
        end
    end
    
    // --- 组合逻辑输出 (地址) ---
    always @(*) begin
        // S1
        o_lut_addr_a = base_addr; o_lut_addr_b = base_addr + 1;
        o_lut_addr_c = base_addr + GRID_ROM_WIDTH; o_lut_addr_d = base_addr + GRID_ROM_WIDTH + 1;
        
        // S5 (V6.5 修复)
        o_linebuf_addr_x0 = x_int;
        o_linebuf_addr_x1 = x_int_plus_1;
        o_linebuf_addr_y0 = y_int; 
        o_linebuf_addr_y1 = y_int_plus_1;
    end
    
endmodule
