// line_buffer_1080p_sp.v  表示 Single Port Interface
// -----------------------------------------------------------------------------

// 模块名称：line_buffer_1080p_sp (Single Port Line Buffer for 1080p)
// 基本功能：实现适用于 1080p 分辨率图像数据处理的单写单读同步 RAM 行缓存，
// 用于临时存储一行图像数据，支持同步写操作和同步读操作。
// 基本介绍：
// 采用单端口接口设计，写端口包含写使能、写地址和写数据信号，读端口通过读地址获取数据；
// 数据宽度默认 16 位（适配 RGB565 格式图像数据），地址宽度默认 11 位（支持 2048 个存储单元，满足 1920 像素行宽需求）；
// 内部存储器设计可被综合工具自动推断为双端口 BRAM 原语，以获得最优的存储性能和时序特性；
// 读写操作均在时钟上升沿同步执行，读数据将在下一个时钟周期输出。
// 设计人员：王博
// 最终修改时间：2025-11-05

// -----------------------------------------------------------------------------

module line_buffer_1080p_sp #(
    parameter DATA_WIDTH = 16,          // RGB565
    parameter ADDR_WIDTH = 11,          // 2^11 = 2048, enough for 1920
    parameter LINE_WIDTH = 1920
)(
    input  wire                    clk,
    // 写端口
    input  wire                    wr_en,
    input  wire [ADDR_WIDTH-1:0]   wr_addr,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    // 读端口 (单端口)
    input  wire [ADDR_WIDTH-1:0]   rd_addr,
    output reg  [DATA_WIDTH-1:0]   rd_data
);
    // 描述一个存储器，让综合工具推断为BRAM
    reg [DATA_WIDTH-1:0] mem [0:LINE_WIDTH-1];

    // 写操作 (同步)
    always @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // 读操作 (同步)
    always @(posedge clk) begin
        // 标准的同步读，数据将在下一个时钟周期出现在 rd_data
        rd_data <= mem[rd_addr]; 
    end
    
endmodule
