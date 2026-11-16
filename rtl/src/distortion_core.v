// distortion_core.v
// -----------------------------------------------------------------------------
// 畸变校正核心 (已修正和完善)
//
// 实现了“双重双线性插值”算法:
// 1. (L1 Interp): 根据输出坐标，插值压缩LUT，得到精确的定点源坐标。
// 2. (L2 Interp): 根据定点源坐标，插值源像素，得到最终的输出颜色。
//
// 采用一个10级流水线来确保时序收敛。
// -----------------------------------------------------------------------------

module distortion_core #(
    parameter IMG_WIDTH = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter MAP_DOWNSAMPLE_LOG2 = 3, // 2^3 = 8x
    parameter Q_COORD_FRAC = 5,       // 坐标定点格式 Q11.5
    parameter Q_WEIGHT_FRAC = 3       // 权重定点格式 Q0.3 (因为是8x8块)
)(
    input wire clk,
    input wire rst_n,

    // 当前输出像素的位置 (x_out, y_out)
    input wire i_valid,
    input wire [10:0] i_x,
    input wire [10:0] i_y,

    // 从LUT ROM读回的4个角点坐标 (Q11.5)
    input wire signed [15:0] i_x_map_a, i_y_map_a,
    input wire signed [15:0] i_x_map_b, i_y_map_b,
    input wire signed [15:0] i_x_map_c, i_y_map_c,
    input wire signed [15:0] i_x_map_d, i_y_map_d,

    // 从行缓存读回的4个源像素颜色 (RGB565)
    input wire [15:0] i_pixel_a, // P(x, y)
    input wire [15:0] i_pixel_b, // P(x+1, y)
    input wire [15:0] i_pixel_c, // P(x, y+1)
    input wire [15:0] i_pixel_d, // P(x+1, y+1)

    // 输出到外部模块
    output reg [14:0] o_lut_addr_a, o_lut_addr_b, o_lut_addr_c, o_lut_addr_d,
    output reg [10:0] o_linebuf_addr_a, o_linebuf_addr_b, o_linebuf_addr_c, o_linebuf_addr_d,

    // 最终输出
    output reg o_valid,
    output reg [15:0] o_pixel_data
);

    // --- Q格式常量 ---
    // L1插值: (Q11.5 * Q0.3) * Q0.3 = Q11.11
    localparam L1_FRAC_BITS = Q_COORD_FRAC + Q_WEIGHT_FRAC * 2; // 5 + 3*2 = 11
    // L2插值: (Q5.0 * Q0.5) * Q0.5 = Q5.10 (使用Q0.5作为像素权重)
    localparam L2_FRAC_BITS = Q_COORD_FRAC * 2; // 5*2 = 10
    localparam L2_WEIGHT_BITS = Q_COORD_FRAC;

    // --- Pipeline Registers ---
    reg valid_pipe [0:9];
    
    // S1 -> S2 (Coordinate Weight Registration)
    reg [Q_WEIGHT_FRAC-1:0] cw_x_s2, cw_y_s2, cw_xi_s2, cw_yi_s2;

    // S2 -> S3 (LUT ROM Read Delay)
    reg [Q_WEIGHT_FRAC-1:0] cw_x_s3, cw_y_s3, cw_xi_s3, cw_yi_s3;

    // S3 -> S4 (L1 Horizontal Interpolation Results)
    reg signed [16+Q_WEIGHT_FRAC:0] x_h1_s4, x_h2_s4, y_h1_s4, y_h2_s4;
    reg [Q_WEIGHT_FRAC-1:0]         cw_y_s4, cw_yi_s4;

    // S4 -> S5 ( Source Coordinate)
    reg signed [15+L1_FRAC_BITS:0] src_x_fx_s5, src_y_fx_s5;

    // S5 -> S6 (Pixel Weight Registration)
    reg [L2_WEIGHT_BITS-1:0] pw_x_s6, pw_xi_s6;
    reg [L2_WEIGHT_BITS-1:0] pw_y_s6, pw_yi_s6;

    // S6 -> S7 (Line Buffer Read Delay)
    reg [L2_WEIGHT_BITS-1:0] pw_x_s7, pw_xi_s7, pw_y_s7, pw_yi_s7;

    // S7 -> S8 (Unpacked Pixel Colors & Weights)
    reg [4:0] r_a_s8, r_b_s8, r_c_s8, r_d_s8;
    reg [5:0] g_a_s8, g_b_s8, g_c_s8, g_d_s8;
    reg [4:0] b_a_s8, b_b_s8, b_c_s8, b_d_s8;
    reg [L2_WEIGHT_BITS-1:0] pw_x_s8, pw_xi_s8, pw_y_s8, pw_yi_s8;

    // S8 -> S9 (L2 Horizontal Interpolation Results)
    reg [4+L2_WEIGHT_BITS:0] r_h1_s9, r_h2_s9;
    reg [5+L2_WEIGHT_BITS:0] g_h1_s9, g_h2_s9;
    reg [4+L2_WEIGHT_BITS:0] b_h1_s9, b_h2_s9;
    reg [L2_WEIGHT_BITS-1:0] pw_y_s9, pw_yi_s9;

    // --- 组合逻辑 ---

    // --- S1: 计算LUT地址和坐标插值权重 (Combinational) ---
    localparam GRID_ROM_WIDTH = IMG_WIDTH >> MAP_DOWNSAMPLE_LOG2;
    wire [7:0] grid_x = i_x >> MAP_DOWNSAMPLE_LOG2;
    wire [7:0] grid_y = i_y >> MAP_DOWNSAMPLE_LOG2;
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_x = i_x[Q_WEIGHT_FRAC-1:0];
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_y = i_y[Q_WEIGHT_FRAC-1:0];
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_x_inv = (1 << Q_WEIGHT_FRAC) - coord_weight_x;
    wire [Q_WEIGHT_FRAC-1:0] coord_weight_y_inv = (1 << Q_WEIGHT_FRAC) - coord_weight_y;
    wire [14:0] base_addr = grid_y * GRID_ROM_WIDTH + grid_x;

    // --- S3: L1 水平插值 (Combinational, feeds S4 registers) ---
    // **修正**: 使用 S3 寄存器 (cw_xi_s3, cw_x_s3) 和 S3 输入 (i_x_map_a)
    wire signed [16+Q_WEIGHT_FRAC:0] x_h1 = i_x_map_a * cw_xi_s3 + i_x_map_b * cw_x_s3;
    wire signed [16+Q_WEIGHT_FRAC:0] x_h2 = i_x_map_c * cw_xi_s3 + i_x_map_d * cw_x_s3;
    wire signed [16+Q_WEIGHT_FRAC:0] y_h1 = i_y_map_a * cw_yi_s3 + i_y_map_b * cw_y_s3;
    wire signed [16+Q_WEIGHT_FRAC:0] y_h2 = i_y_map_c * cw_yi_s3 + i_y_map_d * cw_y_s3;
    
    // --- S4: L1 垂直插值 (Combinational, feeds S5 registers) ---
    // **修正**: 使用 S4 寄存器
    wire signed [15+L1_FRAC_BITS:0] src_x_fx_s4_out = (x_h1_s4 * cw_yi_s4 + x_h2_s4 * cw_y_s4);
    wire signed [15+L1_FRAC_BITS:0] src_y_fx_s4_out = (y_h1_s4 * cw_yi_s4 + y_h2_s4 * cw_y_s4);

    // --- S7: 解包RGB565 (Combinational, feeds S8 registers) ---
    // **修正**: 使用 S7 输入 (i_pixel_a)
    wire [4:0] r_a = i_pixel_a[15:11], r_b = i_pixel_b[15:11], r_c = i_pixel_c[15:11], r_d = i_pixel_d[15:11];
    wire [5:0] g_a = i_pixel_a[10:5],  g_b = i_pixel_b[10:5],  g_c = i_pixel_c[10:5],  g_d = i_pixel_d[10:5];
    wire [4:0] b_a = i_pixel_a[4:0],   b_b = i_pixel_b[4:0],   b_c = i_pixel_c[4:0],   b_d = i_pixel_d[4:0];

    // --- S8: L2 水平插值 (Combinational, feeds S9 registers) ---
    // **修正**: 使用 S8 寄存器
    wire [4+L2_WEIGHT_BITS:0] r_h1 = r_a_s8 * pw_xi_s8 + r_b_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] r_h2 = r_c_s8 * pw_xi_s8 + r_d_s8 * pw_x_s8;
    wire [5+L2_WEIGHT_BITS:0] g_h1 = g_a_s8 * pw_xi_s8 + g_b_s8 * pw_x_s8;
    wire [5+L2_WEIGHT_BITS:0] g_h2 = g_c_s8 * pw_xi_s8 + g_d_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] b_h1 = b_a_s8 * pw_xi_s8 + b_b_s8 * pw_x_s8;
    wire [4+L2_WEIGHT_BITS:0] b_h2 = b_c_s8 * pw_xi_s8 + b_d_s8 * pw_x_s8;
    
    // --- S9: L2 垂直插值 (Combinational, feeds S10 output) ---
    // **修正**: 使用 S9 寄存器
    // (Q5.5 * Q0.5) * Q0.5 = Q5.10 -> 移位10位
    wire [4:0] r_final = (r_h1_s9 * pw_yi_s9 + r_h2_s9 * pw_y_s9) >> L2_FRAC_BITS;
    wire [5:0] g_final = (g_h1_s9 * pw_yi_s9 + g_h2_s9 * pw_y_s9) >> L2_FRAC_BITS;
    wire [4:0] b_final = (b_h1_s9 * pw_yi_s9 + b_h2_s9 * pw_y_s9) >> L2_FRAC_BITS;
    
    wire [L1_FRAC_BITS-1:0] frac_x = src_x_fx_s5[L1_FRAC_BITS-1:0];
    wire [L1_FRAC_BITS-1:0] frac_y = src_y_fx_s5[L1_FRAC_BITS-1:0];

    integer i; 

    // --- 主时序逻辑 (Single Always Block) ---
    always @(posedge clk) begin
        // **修正**: 在 `always` 块内部声明 `integer`
        if (!rst_n) begin
            // **修正**: 使用 `for` 循环初始化流水线
            for (i=0; i<10; i=i+1) begin
                valid_pipe[i] <= 1'b0;
            end
            o_valid <= 1'b0;
            o_pixel_data <= 16'd0;
            
            // 重置所有流水线寄存器
            cw_x_s2 <= 0; cw_y_s2 <= 0; cw_xi_s2 <= 0; cw_yi_s2 <= 0;
            cw_x_s3 <= 0; cw_y_s3 <= 0; cw_xi_s3 <= 0; cw_yi_s3 <= 0;
            x_h1_s4 <= 0; x_h2_s4 <= 0; y_h1_s4 <= 0; y_h2_s4 <= 0;
            cw_y_s4 <= 0; cw_yi_s4 <= 0;
            src_x_fx_s5 <= 0; src_y_fx_s5 <= 0;
            pw_x_s6 <= 0; pw_xi_s6 <= 0; pw_y_s6 <= 0; pw_yi_s6 <= 0;
            pw_x_s7 <= 0; pw_xi_s7 <= 0; pw_y_s7 <= 0; pw_yi_s7 <= 0;
            r_a_s8 <= 0; r_b_s8 <= 0; r_c_s8 <= 0; r_d_s8 <= 0;
            g_a_s8 <= 0; g_b_s8 <= 0; g_c_s8 <= 0; g_d_s8 <= 0;
            b_a_s8 <= 0; b_b_s8 <= 0; b_c_s8 <= 0; b_d_s8 <= 0;
            pw_x_s8 <= 0; pw_xi_s8 <= 0; pw_y_s8 <= 0; pw_yi_s8 <= 0;
            r_h1_s9 <= 0; r_h2_s9 <= 0; g_h1_s9 <= 0; g_h2_s9 <= 0;
            b_h1_s9 <= 0; b_h2_s9 <= 0;
            pw_y_s9 <= 0; pw_yi_s9 <= 0;

        end else begin
            // --- S1: Pipeline Stage Propagation ---
            valid_pipe[0] <= i_valid;
            for (i=1; i<10; i=i+1) begin
                valid_pipe[i] <= valid_pipe[i-1];
            end
            
            // --- S1 -> S2: Register coordinate weights ---
            if(valid_pipe[0]) begin
                cw_x_s2  <= coord_weight_x;
                cw_y_s2  <= coord_weight_y;
                cw_xi_s2 <= coord_weight_x_inv;
                cw_yi_s2 <= coord_weight_y_inv;
            end
            
            // --- S2 -> S3: Register weights again for LUT read delay ---
            if(valid_pipe[1]) begin
                cw_x_s3  <= cw_x_s2;
                cw_y_s3  <= cw_y_s2;
                cw_xi_s3 <= cw_xi_s2;
                cw_yi_s3 <= cw_yi_s2;
            end
            
            // --- S3 -> S4: Register L1 horizontal interpolation results ---
            // **修正**: 使用 S3 的组合逻辑 `x_h1`
            if(valid_pipe[2]) begin
                x_h1_s4 <= x_h1; 
                x_h2_s4 <= x_h2;
                y_h1_s4 <= y_h1; 
                y_h2_s4 <= y_h2;
                cw_y_s4 <= cw_y_s3; 
                cw_yi_s4 <= cw_yi_s3;
            end
            
            // --- S4 -> S5: Register final source coordinate ---
            if(valid_pipe[3]) begin
                src_x_fx_s5 <= src_x_fx_s4_out;
                src_y_fx_s5 <= src_y_fx_s4_out;
            end
            
            // --- S5 -> S6: Register pixel weights ---
            // **修正**: 
            // 1. 使用 `L1_FRAC_BITS` (11) 进行移位
            // 2. 截取小数部分的 *高* `L2_WEIGHT_BITS` (5) 位
            if(valid_pipe[4]) begin                
                pw_x_s6  <= frac_x >> (L1_FRAC_BITS - L2_WEIGHT_BITS);
                pw_xi_s6 <= (1 << L2_WEIGHT_BITS) - (frac_x >> (L1_FRAC_BITS - L2_WEIGHT_BITS));
                pw_y_s6  <= frac_y >> (L1_FRAC_BITS - L2_WEIGHT_BITS);
                pw_yi_s6 <= (1 << L2_WEIGHT_BITS) - (frac_y >> (L1_FRAC_BITS - L2_WEIGHT_BITS));
            end
            
            // --- S6 -> S7: Register for Line Buffer read delay ---
            if(valid_pipe[5]) begin
                pw_x_s7 <= pw_x_s6;
                pw_xi_s7 <= pw_xi_s6;
                pw_y_s7 <= pw_y_s6;
                pw_yi_s7 <= pw_yi_s6;
            end
            
            // --- S7 -> S8: Register unpacked colors and pixel weights ---
            if(valid_pipe[6]) begin
                r_a_s8 <= r_a; r_b_s8 <= r_b; r_c_s8 <= r_c; r_d_s8 <= r_d;
                g_a_s8 <= g_a; g_b_s8 <= g_b; g_c_s8 <= g_c; g_d_s8 <= g_d;
                b_a_s8 <= b_a; b_b_s8 <= b_b; b_c_s8 <= b_c; b_d_s8 <= b_d;
                pw_x_s8 <= pw_x_s7;
                pw_xi_s8 <= pw_xi_s7;
                pw_y_s8 <= pw_y_s7;
                pw_yi_s8 <= pw_yi_s7;
            end
            
            // --- S8 -> S9: Register L2 horizontal interpolation results ---
            if(valid_pipe[7]) begin
                r_h1_s9 <= r_h1; r_h2_s9 <= r_h2;
                g_h1_s9 <= g_h1; g_h2_s9 <= g_h2;
                b_h1_s9 <= b_h1; b_h2_s9 <= b_h2;
                pw_y_s9 <= pw_y_s8;
                pw_yi_s9 <= pw_yi_s8;
            end
            
            // --- S9 -> S10:  Output Stage ---
            if(valid_pipe[8]) begin
                o_pixel_data <= {r_final, g_final, b_final};
            end
            
            // 最终的 o_valid 信号在S9之后产生
            o_valid <= valid_pipe[9];
        end
    end
    
    wire [10:0] x_int = src_x_fx_s5 >> L1_FRAC_BITS;
    wire [10:0] y_int = src_y_fx_s5 >> L1_FRAC_BITS;

    // --- 组合逻辑输出 (地址) ---
    always @(*) begin
        // S1: 输出LUT ROM地址
        o_lut_addr_a = base_addr;
        o_lut_addr_b = base_addr + 1;
        o_lut_addr_c = base_addr + GRID_ROM_WIDTH;
        o_lut_addr_d = base_addr + GRID_ROM_WIDTH + 1;

        // S5: 输出行缓存地址
        // **修正**: 使用 `L1_FRAC_BITS` (11) 作为移位量
        
        // 我们需要 (x_int, y_int) 和 (x_int+1, y_int+1)
        // `distortion_top` 会根据 y_int 和 y_int+1 来选择
        // 到底是读 ping 还是 pong buffer
        o_linebuf_addr_a = x_int;       // For P(x, y)
        o_linebuf_addr_b = x_int + 1;   // For P(x+1, y)
        o_linebuf_addr_c = x_int;       // For P(x, y+1)
        o_linebuf_addr_d = x_int + 1;   // For P(x+1, y+1)
    end
    
endmodule
