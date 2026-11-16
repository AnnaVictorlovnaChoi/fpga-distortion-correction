// distortion_top.v
// -----------------------------------------------------------------------------
// 畸变校正顶层模块 (V10.4 - 时序 & 逻辑 最终修复版)
// 模块名称：distortion_top (Distortion Correction Top Module)
// 基本功能：实现完整的图像畸变校正处理流程，整合畸变校正核心、查找表（LUT）、行缓存、Gamma 校正及 ROI 裁剪功能，输出校正后的 1920x1080 分辨率 RGB565 格式图像。
// 基本介绍：
// 顶层模块架构：例化畸变校正查找表（distortion_lut_rom）、4 个单端口行缓存（line_buffer_1080p_sp）、畸变校正核心（distortion_core）及 Gamma 校正模块（gamma_correction_lut），形成完整处理链路；
// 关键参数配置：适配 1920x1080 分辨率，支持 ROI 区域裁剪（ROI_X/Y/W/H 可配置），总处理延迟（TOTAL_LATENCY）为 12 拍（核心延迟 10 拍 + Gamma 延迟 2 拍）；
// 数据处理流程：输入图像数据经 Ping-Pong Buffer 行缓存存储，畸变校正核心通过 LUT 获取映射坐标并进行双线性插值，校正后数据经 Gamma 校正和 ROI 裁剪输出；
// 时序控制：采用 Ping-Pong Buffer 机制实现行缓存读写分离，通过延迟管线（vs_pipe/hs_pipe）同步控制信号，确保输出时序对齐。
// 修正记录：
// (V10.3) 引入 Gamma 校正模块（gamma_correction_lut），完善图像亮度调整功能；
// (V10.4 - 关键时序修复)
// a. 修正 Gamma 模块延迟参数（GAMMA_LATENCY=2），总延迟（TOTAL_LATENCY）调整为 12 拍；
// b. 同步更新 vs_pipe/hs_pipe 延迟为 12 拍，修复 o_pixel_valid 与 o_frame_vs 的 1-clock 时序错位，解决 "全黑屏" 问题；
// (V10.4 - 关键逻辑修复)
// a. 修复像素源选择（pixel_y1_x0/pixel_y1_x1）的复制 - 粘贴错误，解决 "零散横线" 问题。
// 设计人员：王博
// 最后修改时间：2025-11-05
// -----------------------------------------------------------------------------

module distortion_top #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter ROI_X = 43,  
    parameter ROI_Y = 118,
    parameter ROI_W = 1833,
    parameter ROI_H = 843,
    
    // ** 【V10.4 修正】: 修正 Gamma 延迟 **
    parameter CORE_LATENCY  = 10,
    parameter GAMMA_LATENCY = 2, // <-- 修复: 2 拍延迟
    parameter TOTAL_LATENCY = CORE_LATENCY + GAMMA_LATENCY // 12
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         i_vs,
    input  wire                         i_hs,
    input  wire                         i_de,
    input  wire  [15:0]                 i_data_rgb565,
    
    // (V10 端口)
    output wire                         o_frame_vs,
    output wire                         o_line_hs,
    output wire                         o_pixel_valid,
    output wire  [15:0]                 o_pixel_data
);

    // --- 1. 输入位置计数器 (与 i_de 同步) ---
    reg [10:0] in_x_cnt, in_y_cnt;

    // --- 2. Ping-Pong Buffer 控制 (不变) ---
    reg  ping_pong_sel; // 0: ping=write, pong=read; 1: ping=read, pong=write
    reg  i_hs_d1;
    wire hs_posedge = ~i_hs_d1 & i_hs;

    // --- 3. 模块间连线 ---
    // (不变)
    wire [14:0] lut_addr_a, lut_addr_b, lut_addr_c, lut_addr_d;
    wire signed [15:0] x_map_a, y_map_a, x_map_b, y_map_b, x_map_c, y_map_c, x_map_d, y_map_d;
    wire [10:0] core_addr_x0, core_addr_x1;
    wire [10:0] core_addr_y0, core_addr_y1;
    wire [15:0] pixel_ping_y0_x0, pixel_ping_y0_x1; 
    wire [15:0] pixel_ping_y1_x0, pixel_ping_y1_x1;
    wire [15:0] pixel_pong_y0_x0, pixel_pong_y0_x1; 
    wire [15:0] pixel_pong_y1_x0, pixel_pong_y1_x1;
    
    // Core Output
    wire        core_valid;
    wire [15:0] core_pixel_data_internal; 
    wire [10:0] core_x_out, core_y_out;
    
    // (不变)
    reg [15:0] core_in_pixel_a_reg, core_in_pixel_b_reg;
    reg [15:0] core_in_pixel_c_reg, core_in_pixel_d_reg;

    // (V10.3) Gamma 模块连线
    wire        gamma_valid_out;
    wire [15:0] gamma_data_out;
    wire [15:0] roi_gated_pixel; // ROI 门控后的像素
    wire        in_roi;          // ROI 判断信号

    // --- 4. 例化子模块 ---
    // 4.1. LUT ROM (不变)
    distortion_lut_rom #( .GRID_WIDTH(120), .GRID_HEIGHT(67) ) u_lut_rom (
        .clk(clk), .addr_a(lut_addr_a), .addr_b(lut_addr_b), .addr_c(lut_addr_c), .addr_d(lut_addr_d),
        .x_map_a(x_map_a), .y_map_a(y_map_a), .x_map_b(x_map_b), .y_map_b(y_map_b),
        .x_map_c(x_map_c), .y_map_c(y_map_c), .x_map_d(x_map_d), .y_map_d(y_map_d)
    );

    // 4.2. 4个单端口 Line Buffer (不变)
    line_buffer_1080p_sp u_line_buf_ping_0 ( 
        .clk(clk), .wr_en(i_de & ~ping_pong_sel), .wr_addr(in_x_cnt), .wr_data(i_data_rgb565), 
        .rd_addr(core_addr_x0), .rd_data(pixel_ping_y0_x0)
    );
    line_buffer_1080p_sp u_line_buf_ping_1 ( 
        .clk(clk), .wr_en(i_de & ~ping_pong_sel), .wr_addr(in_x_cnt), .wr_data(i_data_rgb565), 
        .rd_addr(core_addr_x1), .rd_data(pixel_ping_y0_x1)
    );
    line_buffer_1080p_sp u_line_buf_pong_0 ( 
        .clk(clk), .wr_en(i_de &  ping_pong_sel), .wr_addr(in_x_cnt), .wr_data(i_data_rgb565), 
        .rd_addr(core_addr_x0), .rd_data(pixel_pong_y0_x0)
    );
    line_buffer_1080p_sp u_line_buf_pong_1 ( 
        .clk(clk), .wr_en(i_de &  ping_pong_sel), .wr_addr(in_x_cnt), .wr_data(i_data_rgb565), 
        .rd_addr(core_addr_x1), .rd_data(pixel_pong_y0_x1)
    );
    
    // --- 4.3. 像素源选择 (V10.4 "横线" 修正版) ---
    
    // read_buffer_is_ping: 1 = Ping是读缓存, 0 = Pong是读缓存
    wire read_buffer_is_ping = (ping_pong_sel == 1'b1); 
    
    // 来自上一行 ("读缓存") 的像素
    // (Ping=读 ? 选Ping : 选Pong)
    wire [15:0] pixel_y0_x0 = read_buffer_is_ping ? pixel_ping_y0_x0 : pixel_pong_y0_x0;
    wire [15:0] pixel_y0_x1 = read_buffer_is_ping ? pixel_ping_y0_x1 : pixel_pong_y0_x1;
    
    // 来自当前行 ("写缓存") 的像素
    // ** 【V10.4 关键修复】: 修复了 V10.2 的复制-粘贴 BUG **
    // (Ping=读 ? 选Pong : 选Ping)
    wire [15:0] pixel_y1_x0 = read_buffer_is_ping ? pixel_pong_y0_x0 : pixel_ping_y0_x0;
    wire [15:0] pixel_y1_x1 = read_buffer_is_ping ? pixel_pong_y0_x1 : pixel_ping_y0_x1;
    // --- 【V10.4 修复完毕】---

    // (V10.2) 处理 Y=0 时的边界逻辑
    wire [15:0] core_in_pixel_a = (core_addr_y0 == 0) ? pixel_y1_x0 : pixel_y0_x0;
    wire [15:0] core_in_pixel_b = (core_addr_y0 == 0) ? pixel_y1_x1 : pixel_y0_x1;
    wire [15:0] core_in_pixel_c = pixel_y1_x0;
    wire [15:0] core_in_pixel_d = pixel_y1_x1;

    // 4.4. Core (使用 V6.7 接口, 不变)
    distortion_core #( .MAP_DOWNSAMPLE_LOG2(4) ) u_distortion_core (
        .clk(clk), .rst_n(rst_n), .i_valid(i_de), 
        .i_x(in_x_cnt), 
        .i_y(in_y_cnt), 
        .i_x_map_a(x_map_a), .i_y_map_a(y_map_a), .i_x_map_b(x_map_b), .i_y_map_b(y_map_b),
        .i_x_map_c(x_map_c), .i_y_map_c(y_map_c), .i_x_map_d(x_map_d), .i_y_map_d(y_map_d),
        .i_pixel_a(core_in_pixel_a_reg), .i_pixel_b(core_in_pixel_b_reg),
        .i_pixel_c(core_in_pixel_c_reg), .i_pixel_d(core_in_pixel_d_reg),
        .o_lut_addr_a(lut_addr_a), .o_lut_addr_b(lut_addr_b), .o_lut_addr_c(lut_addr_c), .o_lut_addr_d(lut_addr_d),
        .o_linebuf_addr_x0(core_addr_x0), .o_linebuf_addr_x1(core_addr_x1), 
        .o_linebuf_addr_y0(core_addr_y0), .o_linebuf_addr_y1(core_addr_y1),
        .o_valid(core_valid), 
        .o_pixel_data(core_pixel_data_internal),
        .o_x_out(core_x_out), 
        .o_y_out(core_y_out)
    );
    
    // 4.5. (V10.3) Gamma 校正模块 (使用 V1.1)
    gamma_correction_lut u_gamma_correction (
        .clk(clk),
        .rst_n(rst_n),
        .i_valid(core_valid),             // (来自 T+10)
        .i_data_rgb565(roi_gated_pixel),  // (来自 T+10)
        .o_valid(gamma_valid_out),        // (输出 T+12)
        .o_data_rgb565(gamma_data_out)    // (输出 T+12)
    );

    // --- 5. 主时序逻辑 ---
    // 延迟管线
    // ** 【V10.4 修正】: 延迟增加到 12 拍 **
    reg [TOTAL_LATENCY-1:0] vs_pipe, hs_pipe;
    wire out_vs = vs_pipe[TOTAL_LATENCY-1]; // 延迟后的 Vsync
    
    always @(posedge clk) begin
        if (!rst_n) begin
            ping_pong_sel <= 1'b0; i_hs_d1 <= 1'b0;
            vs_pipe <= 0; hs_pipe <= 0;
            in_x_cnt <= 0; in_y_cnt <= 0;
            core_in_pixel_a_reg <= 0; core_in_pixel_b_reg <= 0; 
            core_in_pixel_c_reg <= 0; core_in_pixel_d_reg <= 0;
        end else begin
            // 控制逻辑
            i_hs_d1 <= i_hs;
            if (hs_posedge) begin 
                ping_pong_sel <= ~ping_pong_sel; 
            end
            
            // 输入计数器 (与 i_de 同步)
            if (i_de) begin
                if (in_x_cnt == IMG_WIDTH - 1) begin 
                    in_x_cnt <= 0; 
                    in_y_cnt <= (in_y_cnt == IMG_HEIGHT - 1) ? 0 : in_y_cnt + 1; 
                end else begin 
                    in_x_cnt <= in_x_cnt + 1; 
                end
            end else if (~i_vs) begin // 由输入 vsync 复位
                in_x_cnt <= 0; 
                in_y_cnt <= 0; 
            end
            
            // 延迟管线 (vs, hs)
            // ** 【V10.4 修正】: 延迟增加到 12 拍 **
            vs_pipe <= {vs_pipe[TOTAL_LATENCY-2:0], i_vs};
            hs_pipe <= {hs_pipe[TOTAL_LATENCY-2:0], i_hs};
            
            // 寄存 BRAM 输出 (使用 V10.4 的新逻辑)
            core_in_pixel_a_reg <= core_in_pixel_a;
    
            core_in_pixel_b_reg <= core_in_pixel_b;
            core_in_pixel_c_reg <= core_in_pixel_c;
            core_in_pixel_d_reg <= core_in_pixel_d;
        end
    end

    // --- 6. ROI 裁剪与最终输出 (V10.4 修正) ---
    
    // (V10) ROI 判断 (与 T+10 的 core_valid 同步)
    assign in_roi = (core_x_out >= ROI_X) && (core_x_out < ROI_X + ROI_W) &&
                    (core_y_out >= ROI_Y) && (core_y_out < ROI_Y + ROI_H);

    // (V10.3) 步骤 1: ROI 门控 (在 T+10)
    assign roi_gated_pixel = (in_roi) ? core_pixel_data_internal : 16'h0000;
    
    // (V10.4) 步骤 2: 最终输出 (在 T+12)
    assign o_frame_vs = out_vs; // (已延迟 12 拍)
    assign o_line_hs = hs_pipe[TOTAL_LATENCY-1]; // (已延迟 12 拍)
    assign o_pixel_valid = gamma_valid_out; // (来自 Gamma 模块, T+12)
    assign o_pixel_data = gamma_data_out;  // (来自 Gamma 模块, T+12)

endmodule
