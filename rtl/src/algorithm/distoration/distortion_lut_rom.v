// distortion_lut_rom.v
// -----------------------------------------------------------------------------
//  * 模块名称：dist_corr_lut_rom_720p (Distortion Correction LUT ROM for 720p)
//  * 基本功能：实现适用于720p分辨率图像的畸变校正查找表（LUT）只读存储器，
//  *           用于存储畸变校正所需的坐标映射数据，支持4个并发地址的同步读取操作。
//  * 基本介绍：
//  *   - 适配720p网格规格（80x45，由1280/16和720/16计算得出），满足720p图像畸变校正的网格采样需求；
//  *   - 地址宽度为12位（支持4096个存储单元，满足80x45=3600的存储深度需求）；
//  *   - 加载指定LUT文件（"dist_corr_lut_data.mem"），数据宽度为32位（包含16位X坐标和16位Y坐标，均为有符号数，采用Q11.5格式）；
//  *   - 内部包含两个独立内存数组（mem_ac和mem_bd），均初始化自同一LUT文件，分别处理4路并发读取请求（addr_a/addr_c对应mem_ac，addr_b/addr_d对应mem_bd）；
//  *   - 读取操作在时钟上升沿同步执行，输出对应地址的X、Y坐标映射数据。
//  * 设计人员：王博
//  * 最后修改时间：2025-11-05
// -----------------------------------------------------------------------------

module distortion_lut_rom #(
    parameter GRID_WIDTH   = 120, // 1920 / 16
    parameter GRID_HEIGHT  = 67,  // 1080 / 16 (整数)
    parameter ADDR_WIDTH   = 14,  // 2^14=16384, > 120*67=8040
    parameter MAP_WIDTH    = 16   // 坐标位宽 Q11.5
)(
    input  wire                         clk,
    
    // 4个并发的读取地址
    input  wire [ADDR_WIDTH-1:0]        addr_a, // 目标 BRAM 1, Port A
    input  wire [ADDR_WIDTH-1:0]        addr_b, // 目标 BRAM 2, Port A
    input  wire [ADDR_WIDTH-1:0]        addr_c, // 目标 BRAM 1, Port B
    input  wire [ADDR_WIDTH-1:0]        addr_d, // 目标 BRAM 2, Port B
    
    // 4个并发的坐标数据输出 (signed)
    output reg signed [MAP_WIDTH-1:0]   x_map_a, y_map_a,
    output reg signed [MAP_WIDTH-1:0]   x_map_b, y_map_b,
    output reg signed [MAP_WIDTH-1:0]   x_map_c, y_map_c,
    output reg signed [MAP_WIDTH-1:0]   x_map_d, y_map_d
);

    localparam ROM_DEPTH = GRID_WIDTH * GRID_HEIGHT;
    localparam DATA_WIDTH = MAP_WIDTH * 2; // 32 bits

    // 声明两个独立的内存数组
    reg [DATA_WIDTH-1:0] mem_ac [0:ROM_DEPTH-1]; // 用于存储端口 A 和 C 的数据
    reg [DATA_WIDTH-1:0] mem_bd [0:ROM_DEPTH-1]; // 用于存储端口 B 和 D 的数据

    // 使用两个 initial 块分别初始化两个内存
    // 它们加载的是同一个 .hex 文件
    initial begin
        $readmemh("src/algorithm/distoration/dist_map_xy_16x.hex", mem_ac);
    end
    initial begin
        $readmemh("src/algorithm/distoration/dist_map_xy_16x.hex", mem_bd);
    end

    // BRAM 1: 处理 addr_a 和 addr_c 的读取请求
    always @(posedge clk) begin
        {x_map_a, y_map_a} <= mem_ac[addr_a]; // 使用 BRAM 1 的 Port A
        {x_map_c, y_map_c} <= mem_ac[addr_c]; // 使用 BRAM 1 的 Port B
    end
    
    // BRAM 2: 处理 addr_b 和 addr_d 的读取请求
    always @(posedge clk) begin
        {x_map_b, y_map_b} <= mem_bd[addr_b]; // 使用 BRAM 2 的 Port A
        {x_map_d, y_map_d} <= mem_bd[addr_d]; // 使用 BRAM 2 的 Port B
    end

endmodule
