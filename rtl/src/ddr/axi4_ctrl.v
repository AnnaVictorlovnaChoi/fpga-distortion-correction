/*
 * 模块名称：axi4_ctrl (AXI4 Memory Controller with Ping-Pong Buffer)
 * 基本功能：实现基于AXI4协议的内存控制器，支持与DDR等内存设备的高效数据读写操作，具备乒乓缓冲区管理和跨时钟域数据传输能力，适配多通道（ori/algo）帧数据的读写需求。
 * 基本介绍：
 *   - 参数配置：支持可配置的事务ID长度（C_ID_LEN）、数据总线位宽（C_DATA_LEN，默认128bit）、突发传输长度（C_BURST_LEN）、缓冲区基地址（C_BASE_ADDR0/1）及大小（C_BUF_SIZE）等，适配不同内存规格与数据格式；
 *   - 接口说明：
 *     - AXI4总线接口：包含写地址（AW）、写数据（W）、写响应（B）、读地址（AR）、读数据（R）通道，符合AXI4协议规范；
 *     - 帧数据接口：支持两个写通道（ori/algo）和两个读通道（ori/algo），包含像素时钟、垂直同步、数据使能及数据信号，实现跨时钟域帧数据传输；
 *   - 核心模块：
 *     - 写控制器：通过状态机管理写地址与数据传输，使用FIFO进行跨时钟域数据缓冲，采用乒乓缓冲区避免读写冲突，支持突发传输；
 *     - 读控制器：通过仲裁逻辑处理多通道读请求，基于状态机控制读地址与数据接收，FIFO缓冲数据并适配读通道时钟；
 *     - 乒乓缓冲管理：通过帧索引（rc_ori_wframe_index/rc_algo_wframe_index等）切换读写缓冲区，确保数据连续传输。
 * 核心数据传输逻辑：
 *   写操作：外部帧数据（ori/algo）经跨时钟域FIFO缓冲后，写控制器按AXI4协议生成突发写地址与数据，通过乒乓缓冲区管理（基于帧同步信号切换缓冲区）避免覆盖未读取数据；
 *   读操作：读控制器通过仲裁逻辑优先处理高优先级（ori）通道读请求，生成读地址并接收内存返回数据，经FIFO缓冲后适配读通道时钟输出；
 *   跨时钟域处理：通过异步FIFO实现帧数据时钟（ori_wframe_pclk等）与AXI4时钟（axi_clk）的隔离，确保数据稳定传输。
 * 修正记录：
 *   1. 初始版本：实现AXI4协议下的多通道帧数据读写，支持乒乓缓冲区管理与跨时钟域传输，适配128bit数据总线与突发传输机制。
 * 设计人员：王博、张智华、李伟亮
 * 最后修改时间：2025-11-05
 */

module axi4_ctrl #(
    parameter C_ID_LEN      = 8,                         // 事务ID的位宽
              C_DATA_LEN    = 128,                       // 数据总线位宽，一次可传输128bit
              C_DATA_SIZE   = 4,                         // 数据大小编码，与C_DATA_LEN匹配
                                                         // 编码规则：8bit:0, 16bit:1, 32bit:2, 64bit:3, 128bit:4, 256bit:5, 512bit:6, 1024bit:7
              C_BURST_LEN   = 128,                       // 一次突发(Burst)可连续传输的beat数量
              C_STRB_LEN    = C_DATA_LEN / 8,            // 字节选通信号(掩码)的位宽
              C_ADDR_INC    = C_BURST_LEN * C_STRB_LEN,  // 地址增量，下一次传输地址的偏移量
              C_BASE_ADDR0  = 32'h00000000,              // 基地址，默认为0x00000000
              C_BASE_ADDR1  = C_BASE_ADDR0 + 4 * (1 << 22),              // 基地址，默认为0x00000000
              C_BUF_SIZE    = 22,                        // 缓冲区大小，默认分配2^22字节(4MB)
              C_RD_END_ADDR0 = 1280 * 720,               // 读操作的结束地址
              C_RD_END_ADDR1 = 1280 * 720,               // 读操作的结束地址
              C_W_WIDTH0    = 64,                        // 写数据位宽
              C_R_WIDTH0    = 8,                         // 读数据位宽
              C_W_WIDTH1    = 64,                        // 写数据位宽
              C_R_WIDTH1    = 8                          // 读数据位宽
) (
    // AXI4总线时钟与复位信号
    input wire axi_clk,   // AXI4总线时钟
    input wire axi_reset, // AXI4总线复位信号(高有效)

    // AXI4写地址通道信号
    output [C_ID_LEN-1:0] axi_awid,     // 写地址ID
    output [        31:0] axi_awaddr,   // 写地址
    output [         7:0] axi_awlen,    // 突发长度
    output [         2:0] axi_awsize,   // 传输大小
    output [         1:0] axi_awburst,  // 突发类型
    output                axi_awlock,   // 锁定信号
    output [         3:0] axi_awcache,  // 缓存类型
    output [         2:0] axi_awprot,   // 保护类型
    output [         3:0] axi_awqos,    // QoS(服务质量)信号
    output reg            axi_awvalid,  // 写地址有效
    input                 axi_awready,  // 写地址就绪

    // AXI4写数据通道信号
    output [C_DATA_LEN-1:0] axi_wdata,   // 写数据
    output [C_STRB_LEN-1:0] axi_wstrb,   // 写选通信号
    output                  axi_wlast,   // 最后一个写数据标记
    output reg              axi_wvalid,  // 写数据有效
    input                   axi_wready,  // 写数据就绪

    // AXI4写响应通道信号
    input  [C_ID_LEN-1:0] axi_bid,     // 响应ID
    input  [         1:0] axi_bresp,   // 响应状态
    input                 axi_bvalid,  // 响应有效
    output                axi_bready,  // 响应就绪

    // AXI4读地址通道信号
    output      [C_ID_LEN-1:0] axi_arid,     // 读地址ID
    output      [        31:0] axi_araddr,   // 读地址
    output      [         7:0] axi_arlen,    // 突发长度
    output      [         2:0] axi_arsize,   // 传输大小
    output      [         1:0] axi_arburst,  // 突发类型
    output                     axi_arlock,   // 锁定信号
    output      [         3:0] axi_arcache,  // 缓存类型
    output      [         2:0] axi_arprot,   // 保护类型
    output      [         3:0] axi_arqos,    // QoS信号
    output reg                 axi_arvalid,  // 读地址有效
    input  wire                axi_arready,  // 读地址就绪

    // AXI4读数据通道信号
    input  wire [  C_ID_LEN-1:0] axi_rid,     // 读数据ID
    input  wire [C_DATA_LEN-1:0] axi_rdata,   // 读数据
    input  wire [           1:0] axi_rresp,   // 读响应状态
    input  wire                  axi_rlast,   // 最后一个读数据标记
    input  wire                  axi_rvalid,  // 读数据有效
    output                       axi_rready,  // 读数据就绪

    // 写帧接口信号
    input wire                 ori_wframe_pclk   ,  // 写帧像素时钟
    input wire                 ori_wframe_vsync  ,  // 写帧垂直同步信号，下降沿触发刷新
    input wire                 ori_wframe_data_en,  // 写帧数据使能
    input wire [C_W_WIDTH0-1:0] ori_wframe_data   ,  // 写帧数据
    
    input wire                 algo_wframe_pclk   ,  // 写帧像素时钟
    input wire                 algo_wframe_vsync  ,  // 写帧垂直同步信号，下降沿触发刷新
    input wire                 algo_wframe_data_en,  // 写帧数据使能
    input wire [C_W_WIDTH1-1:0] algo_wframe_data   ,  // 写帧数据

    // 读帧接口信号
    // input  wire                 rframe_sel,
    
    input  wire                 ori_rframe_pclk,     // 读帧像素时钟
    input  wire                 ori_rframe_vsync,    // 读帧垂直同步信号，下降沿触发刷新
    input  wire                 ori_rframe_data_en,  // 读帧数据使能
    output wire [C_R_WIDTH0-1:0] ori_rframe_data,     // 读帧数据
    
    input  wire                 algo_rframe_pclk,     // 读帧像素时钟
    input  wire                 algo_rframe_vsync,    // 读帧垂直同步信号，下降沿触发刷新
    input  wire                 algo_rframe_data_en,  // 读帧数据使能
    output wire [C_R_WIDTH1-1:0] algo_rframe_data,     // 读帧数据

    // 调试输出端口
    output [31:0] tp_o
);
    

    // 初始化写地址、写数据和读地址有效信号为0
    initial begin
        axi_awvalid <= 0;
        axi_wvalid  <= 0;
        axi_arvalid <= 0;
    end

    // AXI4写通道信号赋值
    assign axi_awid    = {C_ID_LEN{1'b0}};  // 写地址ID设为全0
    assign axi_awlen   = C_BURST_LEN - 1'b1;  // 突发长度设置为C_BURST_LEN-1
    assign axi_awsize  = C_DATA_SIZE;  // 传输大小设为C_DATA_SIZE
    assign axi_awburst = 2'b01;  // 突发类型设为INCR(增量模式)
    assign axi_awlock  = 1'b0;  // 不锁定，允许被中断或与其他访问交错
    assign axi_awcache = 0;  // 不可高速缓存且不可缓存
    assign axi_awprot  = 3'b0;  // 正常访问，安全访问，数据访问
    assign axi_awqos   = 4'b0;  // QoS设为最低优先级
    assign axi_wstrb   = 32'hFFFFFFFF;  // 写选通信号全1，表示所有字节都有效
    assign axi_bready  = 1;  // 始终准备好接收响应

    // AXI4读通道信号赋值
    assign axi_arid    = {C_ID_LEN{1'b0}};  // 读地址ID设为全0
    assign axi_arlen   = C_BURST_LEN - 1'b1;  // 突发长度设置为C_BURST_LEN-1
    assign axi_arsize  = C_DATA_SIZE;  // 传输大小设为C_DATA_SIZE
    assign axi_arburst = 2'b01;  // 突发类型设为INCR(增量模式)
    assign axi_arlock  = 1'b0;  // 不锁定
    assign axi_arcache = 0;  // 不可高速缓存且不可缓存
    assign axi_arprot  = 3'b0;  // 正常访问，安全访问，数据访问
    assign axi_arqos   = 4'b0;  // QoS设为最低优先级
    assign axi_rready  = 1;  // 始终准备好接收读数据


    //////////////////////////////////////////////////////////////////
    // 读写调度模块 - 实现乒乓缓冲区管理
    reg  [           1:0] rc_ori_wframe_index    ;
    reg  [           1:0] r_ori_wframe_index_last;
    
    reg  [           1:0] rc_algo_wframe_index    ;
    reg  [           1:0] r_algo_wframe_index_last;
    
    reg  [           2:0] rc_ori_rframe_index;  // 当前读帧索引
    reg  [           2:0] rc_algo_rframe_index;  // 当前读帧索引

    // 实际应该写入的下一帧：如果下一帧正在被读，则跳过

    reg                 r_ori_wframe_inc = 0;
    reg                 r_algo_wframe_inc = 0;
    reg                 r_ori_rframe_inc = 0;  // 写帧完成标志和读帧完成标志
    reg                 r_algo_rframe_inc = 0;  // 写帧完成标志和读帧完成标志

    //////////////////////////////////////////////////////////////////
    // AXI写控制器

    wire                    ori_w_wfifo_pempty  ;
    wire                    ori_w_wfifo_empty   ;    // 写FIFO可编程空信号和空信号
    wire                    ori_w_wfifo_ren     ;      // 写FIFO读使能
    wire [C_DATA_LEN-1:0]   ori_w_wfifo_rdata   ;
    
    wire                    algo_w_wfifo_pempty ;
    wire                    algo_w_wfifo_empty  ;    // 写FIFO可编程空信号和空信号
    wire                    algo_w_wfifo_ren    ;      // 写FIFO读使能
    wire [C_DATA_LEN-1:0]   algo_w_wfifo_rdata  ;
    
    // 写控制器状态机定义
    reg  [           3:0] rs_w = 0;  // 当前状态寄存器
    wire [           3:0] ws_w_idle = 0;  // 空闲状态
    wire [           3:0] ws_w_wdata = 1;  // 写数据状态
    wire [           3:0] ws_w_winc = 2;  // 地址增量状态
    wire [           3:0] ws_w_eof = 3;  // 帧结束状态

    // 写指针和EOF标志
    reg  [C_BUF_SIZE-1:0] ori_rc_w_ptr;
    reg  [C_BUF_SIZE-1:0] algo_rc_w_ptr;
    reg  [           0:0] rc_w_eof = 0;  // EOF标志
    reg                   ori_r_w_rst = 3;  // 写FIFO复位信号
    reg                   algo_r_w_rst = 3;  // 写FIFO复位信号

    // 突发传输控制
    reg  [           7:0] rc_burst = 0;  // 当前突发中已发送的beat个数

    // 帧结束检测器
    reg     [1:0]   r_ori_wframe_sync;
    reg             ori_sync;
    reg             r_ori_weof_pending;  // 帧结束待处理标志
    
    reg             r_ori_sync;
    
    reg     [1:0]    r_algo_wframe_sync;
    reg             algo_sync;
    reg             r_algo_weof_pending;  // 帧结束待处理标志
    
    reg             r_algo_sync;

    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            r_ori_wframe_sync <= 2'b0;
            r_algo_wframe_sync <= 2'b0;
            ori_sync <= 1'b0;
            algo_sync <= 1'b0;
        end else begin
            r_ori_wframe_sync <= {r_ori_wframe_sync[0], ori_wframe_vsync};     //ori通道检测帧同步信号下降沿
            r_algo_wframe_sync <= {r_algo_wframe_sync[0], algo_wframe_vsync};     //algo通道检测帧同步信号在下降沿

            ori_sync <= (r_ori_wframe_sync == 2'b10) ? 1'b1 : 1'b0;    
            algo_sync <= (r_algo_wframe_sync == 2'b10) ? 1'b1 : 1'b0;
            
        end
    end
    
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            r_ori_sync  <= 1'b0;
        end else if(r_ori_wframe_sync == 2'b10) begin
            r_ori_sync <= 1'b1;
        end else if(ori_w_wfifo_empty) begin
            r_ori_sync <= 1'b0;
        end
    end
    
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            r_algo_sync  <= 1'b0;
        end else if(r_algo_wframe_sync == 2'b10) begin
            r_algo_sync <= 1'b1;
        end else if(algo_w_wfifo_empty) begin
            r_algo_sync <= 1'b0;
        end
    end

    // 读写帧索引更新逻辑
    always @(posedge axi_clk) begin
    
        rc_ori_wframe_index     <= axi_reset ? 0 : (r_ori_wframe_inc ? rc_ori_wframe_index + 1 : rc_ori_wframe_index);
        r_ori_wframe_index_last <= axi_reset ? 0 : (r_ori_wframe_inc ? rc_ori_wframe_index : r_ori_wframe_index_last);
        
        rc_algo_wframe_index     <= axi_reset ? 0 : (r_algo_wframe_inc ? rc_algo_wframe_index + 1 : rc_algo_wframe_index);
        r_algo_wframe_index_last <= axi_reset ? 0 : (r_algo_wframe_inc ? rc_algo_wframe_index : r_algo_wframe_index_last);
        
        if (axi_reset) rc_ori_rframe_index <= 0;
        else if (r_ori_rframe_inc) rc_ori_rframe_index <= r_ori_wframe_index_last;
        if (axi_reset) rc_algo_rframe_index <= 0;
        else if (r_algo_rframe_inc) rc_algo_rframe_index <= r_algo_wframe_index_last;
    end
    
    reg       wsel;
    reg       wlast;
    reg       wbusy;
    reg       ori_wfinsh;
    reg       algo_wfinsh;
    reg [2:0] wait_cnt;

    // AXI写控制器状态机逻辑
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            wait_cnt <= 0;
            axi_awvalid <= 0;
            wsel <= 0;
            wlast <= 0;
            axi_wvalid <= 1'b0;
            r_ori_weof_pending <= 1'b0;
            r_algo_weof_pending <= 1'b0;
            ori_rc_w_ptr <= 0;
            algo_rc_w_ptr <= 0;
        end else begin
        
        //每次请求间隔计时
            if (axi_awvalid || axi_wvalid) wait_cnt <= 0;
            else if (wait_cnt != 7) wait_cnt <= wait_cnt + 1;
            
            //一旦握手立即拉低awvalid
            if (axi_awready) begin
                axi_awvalid  <= 1'b0;
                wsel  <= wsel;
                wlast <= wlast;
            end else if (!axi_awvalid && !axi_wvalid && wait_cnt == 7) begin    //空闲时等待请求间隔结束
                //ori通道具有更高优先级
                if (~ori_w_wfifo_pempty || r_ori_weof_pending) begin  //如果FIFO仍有大于一个突发的数据（一帧一定没结束）或者一帧结束但还有数据没写完
                    axi_awvalid  <= 1'b1;
                    wsel  <= 1'b0;
                    wlast <= r_ori_weof_pending;
                end else if (~algo_w_wfifo_pempty || r_algo_weof_pending) begin
                    axi_awvalid  <= 1'b1;
                    wsel  <= 1'b1;
                    wlast <= r_algo_weof_pending;
                end
            end
            
            //写地址请求awvalid
            if ((axi_wlast && axi_wready)) axi_wvalid <= 1'b0;
            else if (axi_awvalid && axi_awready) axi_wvalid <= 1'b1;
            
            //一帧彻底结束
            ori_wfinsh <= (r_ori_sync & ori_w_wfifo_empty);
            algo_wfinsh <= (r_algo_sync & algo_w_wfifo_empty);
            // ori_wfinsh <= (~wsel & axi_wvalid & wlast & axi_wready) || (ori_sync & ori_w_wfifo_empty);
            // algo_wfinsh <= (wsel & axi_wvalid & wlast & axi_wready) || (algo_sync & algo_w_wfifo_empty);
            
            r_ori_wframe_inc <= ori_wfinsh;
            r_algo_wframe_inc <= algo_wfinsh;
            
            // 帧结束信号，r_weof_pending表示一帧结束但是FIFO中还有数据没发完
            if (ori_wfinsh) r_ori_weof_pending <= 1'b0;
            else if (ori_sync & ~ori_w_wfifo_empty) r_ori_weof_pending <= 1'b1;
            
            if (algo_wfinsh) r_algo_weof_pending <= 1'b0;
            else if (algo_sync & ~algo_w_wfifo_empty) r_algo_weof_pending <= 1'b1;
            
            // r_ori_weof_pending <= 1'b0;
            // r_algo_weof_pending <= 1'b0;
            
            //一个突发中发送的数据计数
            if (!axi_wvalid) rc_burst <= 0;
            else if (axi_wready) rc_burst <= rc_burst + 1;
            
            //每个缓冲区地址索引
            //ori
            if (ori_wfinsh) ori_rc_w_ptr <= 0;
            else if (~wsel && axi_awvalid && axi_awready) ori_rc_w_ptr <= ori_rc_w_ptr + C_ADDR_INC;
            //algo
            if (algo_wfinsh) algo_rc_w_ptr <= 0;
            else if (wsel && axi_awvalid && axi_awready) algo_rc_w_ptr <= algo_rc_w_ptr + C_ADDR_INC;
            
            //结束后复位FIFO
            ori_r_w_rst     <= ori_wfinsh;
            algo_r_w_rst    <= algo_wfinsh;
        end
    end

    // 写地址计算：基地址 + 帧索引 + 偏移量
    assign axi_awaddr     = ~wsel ? C_BASE_ADDR0 + {rc_ori_wframe_index, ori_rc_w_ptr} : 
                                    C_BASE_ADDR1 + {rc_algo_wframe_index, algo_rc_w_ptr};
    
    // 写数据连接到FIFO输出
    assign axi_wdata      = ~wsel ? ori_w_wfifo_rdata : algo_w_wfifo_rdata;

    // 最后一个beat标志生成
    assign axi_wlast      = (rc_burst >= C_BURST_LEN - 1);

    // FIFO读使能信号：AXI写操作握手成功时读取FIFO
    assign ori_w_wfifo_ren = ~wsel & axi_wvalid & axi_wready;
    assign algo_w_wfifo_ren = wsel & axi_wvalid & axi_wready;


    // 调试端口输出
    // assign tp_o[1:0]      = {algo_r_w_rst, ori_r_w_rst};  // 输出写状态机当前状态
    // assign tp_o[2]        = axi_wvalid;  // 输出写数据有效信号
    // assign tp_o[3]        = wframe_data_en;  // 输出写帧数据使能信号
    
    wire    ori_prog_full_o ;
    wire    ori_full_o      ;
    wire    [31 :0 ]    ori_rd_datacount_o;
    
    wire    algo_prog_full_o ;
    wire    algo_full_o      ;
    
    // 根据写数据位宽选择不同的FIFO实例
    generate
        if (C_W_WIDTH0 == 64) begin : Gen_FIFO_W64
            W0_FIFO_64 u_W0_FIFO_64 (
                .wr_clk_i    (ori_wframe_pclk   ),      // 写FIFO时钟
                .wr_en_i     (ori_wframe_data_en),      // 写FIFO写使能
                .wdata       (ori_wframe_data   ),      // 写FIFO写数据
                .a_rst_i     (ori_r_w_rst       ),      // 异步复位信号
                .rd_clk_i    (axi_clk           ),      // 读FIFO时钟
                .rd_en_i     (ori_w_wfifo_ren   ),      // 读FIFO读使能
                .rdata       (ori_w_wfifo_rdata ),      // 读FIFO读数据
                .prog_empty_o(ori_w_wfifo_pempty),      // 可编程空标志
                .empty_o     (ori_w_wfifo_empty )
            );
        end else if (C_W_WIDTH0 == 32) begin : Gen_FIFO_W32
            W0_FIFO_32 u_W0_FIFO_32 (
                .wr_clk_i    (ori_wframe_pclk   ),
                .wr_en_i     (ori_wframe_data_en),
                .wdata       (ori_wframe_data   ),
                .a_rst_i     (ori_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (ori_w_wfifo_ren   ),
                .rdata       (ori_w_wfifo_rdata ),
                .prog_empty_o(ori_w_wfifo_pempty),
                .empty_o     (ori_w_wfifo_empty )
            );
        end else if (C_W_WIDTH0 == 16) begin : Gen_FIFO_W16
            W0_FIFO_16 u_W0_FIFO_16 (
                .wr_clk_i    (ori_wframe_pclk   ),
                .wr_en_i     (ori_wframe_data_en),
                .wdata       (ori_wframe_data   ),
                .a_rst_i     (ori_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (ori_w_wfifo_ren   ),
                .rdata       (ori_w_wfifo_rdata ),
                .prog_empty_o(ori_w_wfifo_pempty),
                .empty_o     (ori_w_wfifo_empty )
            );
        end else if (C_W_WIDTH0 == 8) begin : Gen_FIFO_W8
            W0_FIFO_8 u_W0_FIFO_8 (
                .wr_clk_i    (ori_wframe_pclk   ),
                .wr_en_i     (ori_wframe_data_en),
                .wdata       (ori_wframe_data   ),
                .a_rst_i     (ori_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (ori_w_wfifo_ren   ),
                .rdata       (ori_w_wfifo_rdata ),
                .prog_empty_o(ori_w_wfifo_pempty),
                .empty_o     (ori_w_wfifo_empty )
            );
        end
    endgenerate

    // 根据写数据位宽选择不同的FIFO实例
    generate
        if (C_W_WIDTH1 == 64) begin : Gen_FIFO_W64_2
            W0_FIFO_64 algo_u_W0_FIFO_64 (
                .wr_clk_i    (algo_wframe_pclk   ),      // 写FIFO时钟
                .wr_en_i     (algo_wframe_data_en),      // 写FIFO写使能
                .wdata       (algo_wframe_data   ),      // 写FIFO写数据
                .a_rst_i     (algo_r_w_rst       ),      // 异步复位信号
                .rd_clk_i    (axi_clk           ),      // 读FIFO时钟
                .rd_en_i     (algo_w_wfifo_ren   ),      // 读FIFO读使能
                .rdata       (algo_w_wfifo_rdata ),      // 读FIFO读数据
                .prog_empty_o(algo_w_wfifo_pempty),      // 可编程空标志
                .empty_o     (algo_w_wfifo_empty )      
            );
        end else if (C_W_WIDTH1 == 32) begin : Gen_FIFO_W32_2
            W0_FIFO_32 algo_u_W0_FIFO_32 (
                .wr_clk_i    (algo_wframe_pclk   ),
                .wr_en_i     (algo_wframe_data_en),
                .wdata       (algo_wframe_data   ),
                .a_rst_i     (algo_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (algo_w_wfifo_ren   ),
                .rdata       (algo_w_wfifo_rdata ),
                .prog_empty_o(algo_w_wfifo_pempty),
                .empty_o     (algo_w_wfifo_empty )   
            );
        end else if (C_W_WIDTH1 == 16) begin : Gen_FIFO_W16_2
            W0_FIFO_16 algo_u_W0_FIFO_16 (
                .wr_clk_i    (algo_wframe_pclk   ),
                .wr_en_i     (algo_wframe_data_en),
                .wdata       (algo_wframe_data   ),
                .a_rst_i     (algo_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (algo_w_wfifo_ren   ),
                .rdata       (algo_w_wfifo_rdata ),
                .prog_empty_o(algo_w_wfifo_pempty),
                .empty_o     (algo_w_wfifo_empty )   
            );
        end else if (C_W_WIDTH1 == 8) begin : Gen_FIFO_W8_2
            W0_FIFO_8 algo_u_W0_FIFO_8 (
                .wr_clk_i    (algo_wframe_pclk   ),
                .wr_en_i     (algo_wframe_data_en),
                .wdata       (algo_wframe_data   ),
                .a_rst_i     (algo_r_w_rst       ),
                .rd_clk_i    (axi_clk           ),
                .rd_en_i     (algo_w_wfifo_ren   ),
                .rdata       (algo_w_wfifo_rdata ),
                .prog_empty_o(algo_w_wfifo_pempty),
                .empty_o     (algo_w_wfifo_empty )   
            );
        end
    endgenerate


    //////////////////////////////////////////////////////////////////
    // AXI读控制器

    // Ori路径信号定义
    reg                             ori_rframe_vsync_dly;
    wire                            ori_rframe_vsync_neg;

    // Algo路径信号定义
    reg                             algo_rframe_vsync_dly;
    wire                            algo_rframe_vsync_neg;

    // Ori路径帧计数器
    reg [4:0]                       ori_rfifo_cnt;
    reg                             ori_rfifo_rst;

    // Algo路径帧计数器
    reg [5:0]                       algo_rfifo_cnt;
    reg                             algo_rfifo_rst;

    // Ori路径FIFO控制
    reg                             ori_rfifo_wenb;
    reg [C_DATA_LEN-1:0]            ori_rfifo_wdata;
    wire [9:0]                      ori_rfifo_wcnt;
    wire                            ori_rfifo_wfull;
    wire                            ori_rfifo_wr_rst_busy;
    wire                            w_ori_rfifo_aempty;
    wire                            w_ori_rfifo_empty;

    // Algo路径FIFO控制
    reg                             algo_rfifo_wenb;
    reg [C_DATA_LEN-1:0]            algo_rfifo_wdata;
    wire [9:0]                      algo_rfifo_wcnt;
    wire                            algo_rfifo_full;
    wire                            algo_rfifo_wr_rst_busy;
    wire                            algo_rfifo_aempty;
    wire                            algo_rfifo_empty;

    // Ori路径复位信号
    wire w_ori_rfifo_rst = axi_reset || ori_rfifo_rst;

    // Algo路径复位信号
    wire w_algo_rfifo_rst = axi_reset || algo_rfifo_rst;

    // Ori路径vsync检测
    always @(posedge ori_rframe_pclk) begin
        if(axi_reset == 1'b1)
            ori_rframe_vsync_dly <= 1'b0;
        else
            ori_rframe_vsync_dly <= ori_rframe_vsync;
    end

    assign ori_rframe_vsync_neg = ori_rframe_vsync_dly & ~ori_rframe_vsync; //ILA有波形

    // Algo路径vsync检测
    always @(posedge algo_rframe_pclk) begin
        if(axi_reset == 1'b1)
            algo_rframe_vsync_dly <= 1'b0;
        else
            algo_rframe_vsync_dly <= algo_rframe_vsync;
    end

    assign algo_rframe_vsync_neg = algo_rframe_vsync_dly & ~algo_rframe_vsync;  //ILA有波形

    // Ori路径FIFO复位计数器
    always @(posedge ori_rframe_pclk) begin
        if(axi_reset == 1'b1)
            ori_rfifo_cnt <= 5'h1f;
        else begin
            if(ori_rframe_vsync_neg == 1'b1)
                ori_rfifo_cnt <= 5'h1f;
            else if(ori_rfifo_cnt > 5'd0)
                ori_rfifo_cnt <= ori_rfifo_cnt - 1'b1;
            else
                ori_rfifo_cnt <= 5'b0;
        end
    end

    // Algo路径FIFO复位计数器
    always @(posedge algo_rframe_pclk) begin
        if(axi_reset == 1'b1)
            algo_rfifo_cnt <= 6'h2f;
        else begin
            if(algo_rframe_vsync_neg == 1'b1)
                algo_rfifo_cnt <= 6'h2f;
            else if(algo_rfifo_cnt > 6'd0)
                algo_rfifo_cnt <= algo_rfifo_cnt - 1'b1;
            else
                algo_rfifo_cnt <= 6'b0;
        end
    end

    // Ori路径FIFO复位信号生成
    always @(posedge ori_rframe_pclk) begin
        if(axi_reset == 1'b1)
            ori_rfifo_rst <= 1'b1;
        else begin
            if(ori_rfifo_cnt == 5'b0)
                ori_rfifo_rst <= 1'b0;
            else
                ori_rfifo_rst <= 1'b1;
        end
    end

    // Algo路径FIFO复位信号生成
    always @(posedge algo_rframe_pclk) begin
        if(axi_reset == 1'b1)
            algo_rfifo_rst <= 1'b1;
        else begin
            if(algo_rfifo_cnt == 5'b0)
                algo_rfifo_rst <= 1'b0;
            else
                algo_rfifo_rst <= 1'b1;
        end
    end

    // 同步复位信号到axi_clk域
    reg r_ori_rfifo_rst = 0;
    reg r_algo_rfifo_rst = 0;

    always @(posedge axi_clk) begin
        r_ori_rfifo_rst <= w_ori_rfifo_rst; 
        r_algo_rfifo_rst <= w_algo_rfifo_rst;
    end

    // FIFO复位延迟寄存器
    reg [15:0] ori_rfifo_wr_rst_busy_dly = 0;
    reg [15:0] algo_rfifo_wr_rst_busy_dly = 0;

    // Ori路径复位延迟
    always @(posedge axi_clk or posedge r_ori_rfifo_rst) begin
        if(r_ori_rfifo_rst == 1'b1)
            ori_rfifo_wr_rst_busy_dly <= 0;
        else
            ori_rfifo_wr_rst_busy_dly <= {ori_rfifo_wr_rst_busy_dly[14:0], 1'b1};   //ILA有ori_rfifo_wr_rst_busy_dly[15] == 1 
    end

    // Algo路径复位延迟
    always @(posedge axi_clk or posedge r_algo_rfifo_rst) begin
        if(r_algo_rfifo_rst == 1'b1)
            algo_rfifo_wr_rst_busy_dly <= 0;
        else
            algo_rfifo_wr_rst_busy_dly <= {algo_rfifo_wr_rst_busy_dly[14:0], 1'b1};
    end

    // 复位结束检测
    wire ori_rfifo_wr_rst_busy_neg = (ori_rfifo_wr_rst_busy_dly[1:0] == 2'b01) ? 1'b1 : 1'b0;
    wire algo_rfifo_wr_rst_busy_neg = (algo_rfifo_wr_rst_busy_dly[1:0] == 2'b01) ? 1'b1 : 1'b0;

    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            r_ori_rframe_inc <= 0; 
            r_algo_rframe_inc <= 0; 
        end else begin
            r_ori_rframe_inc <= ori_rfifo_wr_rst_busy_neg;
            r_algo_rframe_inc <= algo_rfifo_wr_rst_busy_neg;
        end
    end

    // 仲裁状态定义
    localparam ARB_IDLE = 2'd0;
    localparam ARB_WAIT = 2'd1;
    localparam ARB_SELECT = 2'd2;

    // 仲裁控制信号
    reg [1:0] arb_state = ARB_IDLE;
    reg arb_ori_selected = 0;  // 1表示Ori被选中，0表示Algo被选中
    reg arb_request = 0;       // 仲裁请求信号
    // 地址寄存器
    reg [C_BUF_SIZE-1:0] ori_araddr = 0;
    reg [C_BUF_SIZE-1:0] algo_araddr = 0;
    // 仲裁请求逻辑
    always @(*) begin
        arb_request = (ori_rfifo_wr_rst_busy_dly[15] && (ori_araddr < C_RD_END_ADDR0) && (~ori_rfifo_wfull)) ||
                      (algo_rfifo_wr_rst_busy_dly[15] && (algo_araddr < C_RD_END_ADDR1) && (~algo_rfifo_full));
    end

    // 仲裁状态机

    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            arb_state <= ARB_IDLE;
            arb_ori_selected <= 0;
        end else begin
            case(arb_state)
                ARB_IDLE: begin
                    if(arb_request) begin
                        arb_state <= ARB_SELECT;
                    end
                end
                
                ARB_SELECT: begin
                    // 优先选择Ori路径，如果Ori不就绪则选择Algo
                    if(ori_rfifo_wr_rst_busy_dly[15] && (ori_araddr < C_RD_END_ADDR0) && (~ori_rfifo_wfull)) begin
                        arb_ori_selected <= 1;
                        arb_state <= ARB_WAIT;
                    end else if(algo_rfifo_wr_rst_busy_dly[15] && (algo_araddr < C_RD_END_ADDR1) && (~algo_rfifo_full)) begin
                        arb_ori_selected <= 0;
                        arb_state <= ARB_WAIT;
                    end else begin
                        arb_state <= ARB_IDLE;
                    end
                end
                
                ARB_WAIT: begin
                    // 等待当前路径读取完成后再回到IDLE状态
                    if(axi_rvalid && axi_rlast) begin
                        arb_state <= ARB_IDLE;
                    end
                end
                
                default: 
                    arb_state <= ARB_IDLE;
            endcase
        end
    end

    // 状态机定义
    localparam S_READ_IDLE = 2'd0;
    localparam S_READ_ADDR = 2'd1;
    localparam S_READ_DATA = 2'd2;

    // 全局状态机（控制AXI4总线）
    reg [1:0] global_rd_state = S_READ_IDLE;
    reg global_rd_pend = 0;



    // 全局状态机（控制AXI4总线）
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin
            global_rd_state <= S_READ_IDLE;
            axi_arvalid <= 0; 
            global_rd_pend <= 0; 
        end else if(r_ori_rfifo_rst) begin
            global_rd_state <= S_READ_IDLE;
            axi_arvalid <= 0; 
            global_rd_pend <= 0; 
        end else if(r_algo_rfifo_rst) begin
            global_rd_state <= S_READ_IDLE;
            axi_arvalid <= 0; 
            global_rd_pend <= 0; 
        end else begin
            if(axi_arready) begin
                axi_arvalid <= 0; 
            end
            
            if(axi_rvalid && axi_rlast) begin
                global_rd_pend <= 0; 
            end
            
            case(global_rd_state)
                S_READ_IDLE: begin
                    // 根据仲裁结果决定是否开始读取
                    if(arb_state == ARB_WAIT) begin
                        global_rd_state <= S_READ_ADDR;
                    end else
                        global_rd_state <= S_READ_IDLE;
                end
                
                S_READ_ADDR: begin
                    axi_arvalid <= 1;   //ILA无上升沿
                    global_rd_pend <= 1; 
                    global_rd_state <= S_READ_DATA;
                end
                
                S_READ_DATA: begin
                    if((~axi_arvalid) && (~global_rd_pend))
                        global_rd_state <= S_READ_IDLE;
                    else
                        global_rd_state <= S_READ_DATA;
                end
                
                default: 
                    global_rd_state <= S_READ_IDLE;
            endcase
        end
    end

    // 地址选择逻辑
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset == 1'b1) begin
            ori_araddr <= 0;
            algo_araddr <= 0;
        end else if(r_ori_rfifo_rst) begin
            ori_araddr <= 0;
        end else if(r_algo_rfifo_rst) begin
            algo_araddr <= 0;
        end else begin
            // Ori地址更新
            if((axi_arvalid == 1'b1)&&(axi_arready == 1'b1)&&(arb_ori_selected == 1'b1))
                ori_araddr <= ori_araddr + C_ADDR_INC;
            
            // Algo地址更新
            if((axi_arvalid == 1'b1)&&(axi_arready == 1'b1)&&(arb_ori_selected == 1'b0))
                algo_araddr <= algo_araddr + C_ADDR_INC;
        end
    end

    // AXI地址选择
    assign axi_araddr = arb_ori_selected ? C_BASE_ADDR0 + {rc_ori_rframe_index, ori_araddr} : 
                                            C_BASE_ADDR1 + {rc_algo_rframe_index, algo_araddr};

    // AXI读数据准备信号
    assign axi_rready = 1; 

    // FIFO写使能逻辑（根据当前选中的路径）
    always @(posedge axi_clk) begin
        if(axi_reset == 1'b1) begin
            ori_rfifo_wenb <= 1'b0;
            algo_rfifo_wenb <= 1'b0;
        end else begin
            // 根据仲裁结果决定哪个FIFO接收数据
            if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1)) begin
                if(arb_ori_selected == 1'b1) begin
                    ori_rfifo_wenb <= 1'b1;
                    algo_rfifo_wenb <= 1'b0;
                end else begin
                    ori_rfifo_wenb <= 1'b0;
                    algo_rfifo_wenb <= 1'b1;
                end
            end else begin
                ori_rfifo_wenb <= 1'b0;
                algo_rfifo_wenb <= 1'b0;
            end
        end
    end

    // FIFO写数据选择
    always @(posedge axi_clk) begin
            ori_rfifo_wdata <= axi_rdata;
            algo_rfifo_wdata <= axi_rdata;  
    end

    // Ori路径FIFO实例化
    generate
        if(C_R_WIDTH0 == 8) begin: Gen_Ori_RFIFO_8
            R0_FIFO_8 u_Ori_R0_FIFO_8 (
                .wr_clk_i      (axi_clk),
                .wr_en_i       (ori_rfifo_wenb),
                .wdata         (ori_rfifo_wdata),
                .a_rst_i       (w_ori_rfifo_rst), 
                .prog_full_o   (ori_rfifo_wfull),
                .rd_clk_i      (ori_rframe_pclk),
                .rd_en_i       (ori_rframe_data_en),
                .rdata         (ori_rframe_data),
                .empty_o       (w_ori_rfifo_empty),
                .almost_empty_o(w_ori_rfifo_aempty)
            );
        end else if(C_R_WIDTH0 == 16) begin: Gen_Ori_RFIFO_16	
            R0_FIFO_16 u_Ori_R0_FIFO_16 (
                .wr_clk_i      (axi_clk),
                .wr_en_i       (ori_rfifo_wenb),
                .wdata         (ori_rfifo_wdata),
                .a_rst_i       (w_ori_rfifo_rst), 
                .prog_full_o   (ori_rfifo_wfull),
                .rd_clk_i      (ori_rframe_pclk),
                .rd_en_i       (ori_rframe_data_en),
                .rdata         (ori_rframe_data),
                .empty_o       (w_ori_rfifo_empty),
                .almost_empty_o(w_ori_rfifo_aempty)
            );
        end
    endgenerate

    // Algo路径FIFO实例化
    generate
        if(C_R_WIDTH1 == 8) begin: Gen_Algo_RFIFO_8
            R0_FIFO_8 u_Algo_R0_FIFO_8 (
                .wr_clk_i      (axi_clk),
                .wr_en_i       (algo_rfifo_wenb),
                .wdata         (algo_rfifo_wdata),
                .a_rst_i       (w_algo_rfifo_rst), 
                .prog_full_o   (algo_rfifo_full),
                .rd_clk_i      (algo_rframe_pclk),
                .rd_en_i       (algo_rframe_data_en),
                .rdata         (algo_rframe_data),
                .empty_o       (algo_rfifo_empty),
                .almost_empty_o(algo_rfifo_aempty)
            );
        end else if(C_R_WIDTH1 == 16) begin: Gen_Algo_RFIFO_16	
            R0_FIFO_16 u_Algo_R0_FIFO_16 (
                .wr_clk_i      (axi_clk),
                .wr_en_i       (algo_rfifo_wenb),
                .wdata         (algo_rfifo_wdata),
                .a_rst_i       (w_algo_rfifo_rst), 
                .prog_full_o   (algo_rfifo_full),
                .rd_clk_i      (algo_rframe_pclk),
                .rd_en_i       (algo_rframe_data_en),
                .rdata         (algo_rframe_data),
                .empty_o       (algo_rfifo_empty),
                .almost_empty_o(algo_rfifo_aempty)
            );
        end
    endgenerate

endmodule
