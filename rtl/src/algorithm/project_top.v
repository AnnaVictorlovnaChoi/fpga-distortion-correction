//  * 模块名称：project_top (Complete Video Processing Pipeline Top Module)
//  * 基本功能：实现完整的1920x1080分辨率视频处理流水线，串联色彩校正与畸变校正功能，接收DVP相机原始输入，输出经处理后符合DDR控制器接口要求的视频流。
//  * 基本介绍：
//  *   - 模块架构：通过标准DVP接口串联两个核心处理模块，形成"色彩校正→畸变校正"的完整流水线；
//  *   - 子模块例化：
//  *     - 工位1（色彩校正）：例化`color_correction_top`，对输入的原始RGB565数据进行色彩校正，输出时序同步的校正后DVP流，固定延迟3个时钟周期；
//  *     - 工位2（畸变校正）：例化`distortion_top`，接收色彩校正后的DVP流，进行畸变校正与ROI裁剪，输出最终处理结果，固定延迟10个时钟周期；
//  *   - 接口适配：输入为DVP相机信号（i_vs/i_hs/i_de/i_data_rgb565），输出为适配DDR控制器的信号（o_frame_vs/o_line_hs/o_pixel_valid/o_pixel_data），确保时序严格同步。
//  * 核心畸变校正数学处理逻辑：
//  *   底层通过`distortion_top`集成的畸变校正核心，基于相机内参矩阵与畸变系数（基础模型k1/k2和k3辅助模型），利用查找表（LUT）存储预计算的坐标映射关系；通过加权融合两种模型的校正结果，结合双线性插值算法处理像素数据，同时通过ROI参数（ROI_X/Y/W/H）裁剪有效区域，最终输出精准校正的视频流。
//  * 设计审查结论：
//  *   设计正确且健壮，通过标准DVP接口串联`color_correction_top`和`distortion_top`两个"黑盒"模块；底层模块内部逻辑修改不影响其DVP接口行为约定，因此不影响顶层模块正确性；输出的o_frame_vs与o_pixel_data/o_pixel_valid保持严格同步，可安全用于触发DDR控制器的帧缓冲切换。
//  * 修正记录：
//  *   1. 最终确认版：整合色彩校正与畸变校正流水线，明确各模块延迟特性（色彩校正3拍，畸变校正10拍），固化ROI参数，确保输出时序适配DDR控制器接口要求。
//  * 设计人员：王博
//  * 设计时间：2025-11-05


module project_top #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080
)(
    // --- 1. 时钟与复位 ---
    input  wire                         clk,          // 148MHz 系统时钟
    input  wire                         rst_n,        // 低电平复位
    
    // --- 2. DVP相机输入接口 ---
    input  wire                         i_vs,         // 原始垂直同步
    input  wire                         i_hs,         // 原始水平同步
    input  wire                         i_de,         // 原始数据使能
    input  wire  [15:0]                 i_data_rgb565,// 原始RGB565数据
    
    // --- 3. 输出到DDR内存控制器 ---
    output wire                         o_frame_vs,   // 帧起始信号 (Vsync)
    output wire                         o_line_hs,    // 行起始信号 (Hsync)
    output wire                         o_pixel_valid,// 像素有效 (Data Enable / FIFO Write)
    output wire  [15:0]                 o_pixel_data   // 校正后的RGB565数据
);

    // --- 模块间信号线 (工位1 -> 工位2) ---
    // 这条线承载一个【色彩已校正】的、时序同步的DVP流
    wire                                cc_vs;
    wire                                cc_hs;
    wire                                cc_de;
    wire [15:0]                         cc_data_rgb565;

    // --- 例化 工位1: 色彩校正 ---
    // 作为一个整体，它将输入的DVP流延迟了固定的3个时钟周期
    color_correction_top u_color_correction (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        // 输入: 原始DVP流
        .i_vs                   (i_vs),
        .i_hs                   (i_hs),
        .i_de                   (i_de),
        .i_data_rgb565          (i_data_rgb565),
        
        // 输出: 色彩校正后的DVP流
        .o_corrected_vs         (cc_vs),
        .o_corrected_hs         (cc_hs),
        .o_corrected_de         (cc_de),
        .o_corrected_data_rgb565(cc_data_rgb565)
    );
    
    // --- 例化 工位2: 畸变校正 ---
    // 作为一个整体，它将输入的DVP流(cc_...)延迟了固定的10个时钟周期
    distortion_top #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        // ROI参数从Python脚本的输出中获取
        .ROI_X      (43),
        .ROI_Y      (118),
        .ROI_W      (1833),
        .ROI_H      (843),
        .CORE_LATENCY (10) 
    ) u_distortion (
        .clk            (clk),
        .rst_n          (rst_n),
        
        // 输入: 来自工位1的色彩校正DVP流
        .i_vs           (cc_vs),
        .i_hs           (cc_hs),
        .i_de           (cc_de),
        .i_data_rgb565  (cc_data_rgb565),
        
        // 输出: 最终的、完全校正并裁剪后的视频流
        .o_frame_vs           (o_frame_vs),
        .o_line_hs           (o_line_hs),
        .o_pixel_valid           (o_pixel_valid),
        .o_pixel_data  (o_pixel_data)
    );

endmodule
