
/*
 * @module    主程序
 * @brief     通过dvp和mipi采集，用按钮2切换状态，然后使用按钮1切换hdmi显示
 * @author    王博、张智华、李伟亮
 * @date      2025-09-21
 * @need modules: ddr_ip, axi_ctrl, dvp_init/capture, hdmi, button, MIPI_cpature
 */

module main (
	 input		clk_24m,//外部时钟的进入，用于驱动pll生成系统时钟
	 input		clk_hdmix2,
	// input		clk_sys,//24m生成的
    input       clk_96m_sys,//系统采用经过pll 96m的时钟作为系统时钟,(实际暂时96m)
	input		clk_hdmi,//Vga使用148.5MHZ
	input		clk_hdmix5,
	input		pll_inst2_LOCKED,
	input		clk_50m_sys,
	input		clk_27m,//给xclk用（24m实际）
	//----------------------------------------------
	//	DDR Clock
	output 			ddr_pll_rstn_o, 
	
	input 			tdqss_clk,			
	input 			core_clk,			//	DDR PLL 200MHz
	input 			tac_clk,			
	input 			twd_clk,		




    input       Button_1,//控制原画面/算法处理后画面
    input       Button_2,//控制dvp相机和mipi相机

///////////////////////////debug口


////////////////////////////////////////////////////////////////
	//	DDR Interface ,采用多个fifo，16位数数据存储
	output 	[15:0] 	addr,
	output 	[2:0] 	ba,
	output 			we,
	output 			reset,
	output 			ras,
	output 			cas,
	output 			odt,
	output 			cke,
	output 			cs,
	
	//	DQ I/O
	input 	[15:0] 	i_dq_hi,
	input 	[15:0] 	i_dq_lo,
	
	output 	[15:0] 	o_dq_hi,
	output 	[15:0] 	o_dq_lo,
	output 	[15:0] 	o_dq_oe,
	
	//	DM O
	output 	[1:0] 	o_dm_hi,
	output 	[1:0] 	o_dm_lo,
	
	//	DQS I/O
	input 	[1:0] 	i_dqs_hi,
	input 	[1:0] 	i_dqs_lo,
	
	input 	[1:0] 	i_dqs_n_hi,
	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
	output 	[1:0] 	o_dqs_n_hi,
	output 	[1:0] 	o_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_oe,
	output 	[1:0] 	o_dqs_n_oe,
	
	//	CK
	output 			clk_p_hi, 
	output 			clk_p_lo, 
	output 			clk_n_hi, 
	output 			clk_n_lo, 
	
	
	//input 			ddr_pll_lock,		//	DDR PLL Lock
	
	//-----------------------------------------------
	//	DDR PLL Phase Shift Interface
	output 	[2:0] 	shift,
	output 	[4:0] 	shift_sel,
	output 			shift_ena,

//////////////////////////////////////////////
    //hdmi_interface
	output      	tmds_clk_p_h,
	output      	tmds_clk_n_h,
	output      	tmds_clk_p_l,
	output      	tmds_clk_n_l,
	output      [2:0]	tmds_data_p_h,
	output      [2:0]	tmds_data_p_l,
	output      [2:0]	tmds_data_n_h,
	output      [2:0]	tmds_data_n_l,

// //////////////////////////////////////////////
// 	//ov5640 cmos interface(720p)
// 	output			camera_sclk,
// 	input			camera_vsync,
// 	input			camera_href,
// 	input			camera_pclk,
// 	output			camera_xclk,
// 	input 	[7:0]	camera_data,

// 	input			i2c_sdat_i,
// 	output			i2c_sdat_o,
// 	output			i2c_sdat_oe,//camera_sdat（inout）

// 	output			camera_pwdn,
// 	output			camera_ctl2,
// 	output			camera_ctl3,
	
////////////////////////////////////////////////////////////////
	//	MIPI-CSI Ctl / I2C(SC2210)
	output 			csi_ctl0_o,
	output 			csi_ctl0_oe,
	input 			csi_ctl0_i,
	
	output 			csi_ctl1_o,
	output 			csi_ctl1_oe,
	input 			csi_ctl1_i,
	
	output 			csi_scl_o,
	output 			csi_scl_oe,
	input 			csi_scl_i,
	
	output 			csi_sda_o,
	output 			csi_sda_oe,
	input 			csi_sda_i,
	
	//	MIPI-CSI RXC 
	input 			csi_rxc_lp_p_i,
	input 			csi_rxc_lp_n_i,
	output 			csi_rxc_hs_en_o,
	output 			csi_rxc_hs_term_en_o,
	input 			csi_rxc_i,
	
	//	MIPI-CSI RXD0
	output 			csi_rxd0_rst_o,
	output 			csi_rxd0_hs_en_o,
	output 			csi_rxd0_hs_term_en_o,
	
	input 			csi_rxd0_lp_p_i,
	input 			csi_rxd0_lp_n_i,
	input 	[7:0] 	csi_rxd0_hs_i,
	
	//	MIPI-CSI RXD1
	output 			csi_rxd1_rst_o,
	output 			csi_rxd1_hs_en_o,
	output 			csi_rxd1_hs_term_en_o,
	
	input 			csi_rxd1_lp_n_i,
	input 			csi_rxd1_lp_p_i,
	input 	[7:0] 	csi_rxd1_hs_i,
	
	//	MIPI-CSI RXD2
	output 			csi_rxd2_rst_o,
	output 			csi_rxd2_hs_en_o,
	output 			csi_rxd2_hs_term_en_o,
	
	input 			csi_rxd2_lp_p_i,
	input 			csi_rxd2_lp_n_i,
	input 	[7:0] 	csi_rxd2_hs_i,
	
	//	MIPI-CSI RXD3
	output 			csi_rxd3_rst_o,
	output 			csi_rxd3_hs_en_o,
	output 			csi_rxd3_hs_term_en_o,
	
	input 			csi_rxd3_lp_p_i,
	input 			csi_rxd3_lp_n_i,
	input 	[7:0] 	csi_rxd3_hs_i,
	
    output  [5:0]   led_o


);
	// assign cmos_ctl2=0;
	// assign cmos_ctl3=0;


	wire   rst_n;
	assign  rst_n=pll_inst2_LOCKED;

	///hdmi
	wire  [23:0]data_input;
	wire  [23:0]data_vga;
	wire 	vga_blk;
	wire 	vga_vsync;
	wire 	vga_hsync;

	wire [25:0]hcount;
	wire [15:0]vcount;

	
wire hdmi_hsync;
wire hdmi_vsync;
wire hdmi_de;
///////////////////////////////////////////////////////////////
//MIPI_csi_rx(2210)
wire 			MIPI_rgb_vsync, MIPI_rgb_hsync, MIPI_rgb_href; 
wire 	[7:0] 	MIPI_rgb_r, MIPI_rgb_g, MIPI_rgb_b; 
//算法模块的变量
  wire i_vs=MIPI_rgb_vsync;
  wire i_hs=MIPI_rgb_hsync;
  wire i_de=MIPI_rgb_href;
  wire [15:0]i_data_rgb565={MIPI_rgb_r[7:3], MIPI_rgb_g[7:2], MIPI_rgb_b[7:3]};

///////////////////////////////////////////////////////////////////////////////////////////
	//按键切换状态,以及主程序目前状态

    reg hdmi_display_now_status=0;

    wire key_press_1;//0代表原图像，1代表处理图像


    wire key_flag1;
    wire key_state1;


    assign key_press_1=key_flag1 && !key_state1;

       button  button_inst1 (
        .Clk(clk_hdmi),
        .Rst_n(rst_n),
        .key_in(Button_1),
        .key_flag(key_flag1),
        .key_state(key_state1)
      );
  
    //按键切换最终输出数据
    //上电后默认为ori，led[0]（最右侧）灭：ori，亮：algo
    
    parameter HDMI_DISPLAY_ORI  = 0;
    parameter HDMI_DISPLAY_ALGO = 1;
    
    reg read_mode = HDMI_DISPLAY_ORI;

    wire key_press_2;

    wire key_flag2;
    wire key_state2;


    assign key_press_2 = key_flag2 && !key_state2;

       button  button_inst2 (
        .Clk(clk_hdmi),
        .Rst_n(rst_n),
        .key_in(Button_2),
        .key_flag(key_flag2),
        .key_state(key_state2)
      );

    always @(posedge clk_hdmi or negedge rst_n) begin
        if (!rst_n) begin
            read_mode <= 0;
        end else if(key_press_2) begin
            read_mode <= ~read_mode;
        end
    end
    
    assign led_o[0] = ~read_mode;


always @(posedge clk_hdmi or negedge rst_n) begin
	if (!rst_n) begin
		hdmi_display_now_status<=0;
	end
	else if (key_press_1==1) begin
		hdmi_display_now_status<=~hdmi_display_now_status;
	end
	else
		hdmi_display_now_status<=hdmi_display_now_status;
end



////寄存两个控制信号，分别直到hdmi显示完1帧，和当前摄像头发出帧同步信号
reg hdmi_display_now_status_r=1;//寄存控制信号，防止1帧显示途中变化
always @(posedge clk_hdmi or negedge rst_n)begin
	if (!rst_n) begin
		hdmi_display_now_status_r<=1;
	end
	else if (hdmi_vsync) begin
		hdmi_display_now_status_r<=hdmi_display_now_status;
	end
	else
		hdmi_display_now_status_r<=hdmi_display_now_status_r;
end



////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	//	I2C Config (SC2210)

		
	//  i2c timing controller module of 16Bit
	wire            [ 8:0]          sc2210_i2c_config_index;
	wire            [23:0]          sc2210_i2c_config_data;
	wire            [ 8:0]          sc2210_i2c_config_size;
	wire                            sc2210_i2c_config_done;
	
	i2c_timing_ctrl_reg16_dat8_wronly
	#(
	    .CLK_FREQ           (96_000_000),                              //  100 MHz
	    .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	) u_i2c_timing_ctrl_16bit_sc2210 (
	    //  global clock
	    .clk                (clk_96m_sys                 ),                          //  96MHz
	    .rst_n              (rst_n               ),                          //  system reset

	    //  i2c interface
	    .i2c_sclk           (csi_scl_o               ),                          //  i2c clock
	    .i2c_sdat_IN        (csi_sda_i               ),
	    .i2c_sdat_OUT       (csi_sda_o               ),
	    .i2c_sdat_OE        (csi_sda_oe              ),

	    //  i2c config data
	    .i2c_config_index   (sc2210_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h60, sc2210_i2c_config_data}),                     //  i2c config data
	    .i2c_config_size    (sc2210_i2c_config_size         ),                          //  i2c config data counte
	    .i2c_config_done    (sc2210_i2c_config_done         )                          //  i2c config timing complete
	);

	assign csi_scl_oe = 1; 
	//  I2C Configure Data of SC2210
	I2C_SC2210_19201080_4Lanes_Config u_I2C_SC2210_19201080_4Lanes_Config
	(
	    .LUT_INDEX  (sc2210_i2c_config_index   ),
	    .LUT_DATA   (sc2210_i2c_config_data    ),
	    .LUT_SIZE   (sc2210_i2c_config_size    )
	);
		
	





	//	The CSI RXC shall not be inverted. Data can be inverted with swapped LP data and flipped HS data. 
	localparam 	CSI_RXD_INV 	= 4'b1111; 
	localparam 	CSI_DATA_WIDTH 	= 32; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	//	Current implementation supports RAW8 only. 
	wire 			w_csi_rx_clk; 
	wire 			w_csi_rx_vsync0, w_csi_rx_hsync0, w_csi_rx_dvalid; 
	wire 	[63:0] 	w_csi_rx_data; 
	wire 	[47:0] 	w_csi_rx_data_rel_raw; 
	
	//	AXI Interface
	wire 	[31:0] 	w_csi_axi_rdata; 
	wire 			w_csi_axi_awready, w_csi_axi_wready, w_csi_axi_arready, w_csi_axi_rvalid; 
	
`define PRI_MIPI_IP
	
	
`ifdef PRI_MIPI_IP
	wire 	[7:0] 	w_mipi_d0 = (CSI_RXD_INV[0] ? 8'hFF : 8'h00) ^ csi_rxd0_hs_i; 
	wire 	[7:0] 	w_mipi_d1 = (CSI_RXD_INV[1] ? 8'hFF : 8'h00) ^ csi_rxd1_hs_i; 
	wire 	[7:0] 	w_mipi_d2 = (CSI_RXD_INV[2] ? 8'hFF : 8'h00) ^ csi_rxd2_hs_i; 
	wire 	[7:0] 	w_mipi_d3 = (CSI_RXD_INV[3] ? 8'hFF : 8'h00) ^ csi_rxd3_hs_i; 
	MIPIRx4LaneFre mipi0_rx (
		.sclk_i			(clk_96m_sys), 
		.srst_i			(~rst_n), 
		
		.mipi_rx_byteclk_i	(csi_rxc_i), 
		.mipi_rx_d0_i		({w_mipi_d0[0], w_mipi_d0[1], w_mipi_d0[2], w_mipi_d0[3], w_mipi_d0[4], w_mipi_d0[5], w_mipi_d0[6], w_mipi_d0[7]}), 
		.mipi_rx_d1_i		({w_mipi_d1[0], w_mipi_d1[1], w_mipi_d1[2], w_mipi_d1[3], w_mipi_d1[4], w_mipi_d1[5], w_mipi_d1[6], w_mipi_d1[7]}), 
		.mipi_rx_d2_i		({w_mipi_d2[0], w_mipi_d2[1], w_mipi_d2[2], w_mipi_d2[3], w_mipi_d2[4], w_mipi_d2[5], w_mipi_d2[6], w_mipi_d2[7]}), 
		.mipi_rx_d3_i		({w_mipi_d3[0], w_mipi_d3[1], w_mipi_d3[2], w_mipi_d3[3], w_mipi_d3[4], w_mipi_d3[5], w_mipi_d3[6], w_mipi_d3[7]}), 
		.mipi_rx_clk_lp_p_i	(csi_rxc_lp_p_i), 
		.mipi_rx_clk_lp_n_i	(csi_rxc_lp_n_i), 
		.mipi_rx_d0_lp_p_i	(CSI_RXD_INV[0] ? csi_rxd0_lp_n_i : csi_rxd0_lp_p_i), 
		.mipi_rx_d0_lp_n_i	(CSI_RXD_INV[0] ? csi_rxd0_lp_p_i : csi_rxd0_lp_n_i), 
		.mipi_rx_rst_o		(), 
		.mipi_rx_rst_n_o		(), 
		.mipi_rx_clk_hs_en_o	(csi_rxc_hs_en_o), 
		.mipi_rx_dat_hs_en_o	(csi_rxd0_hs_en_o), 
		.mipi_rx_clk_term_en_o	(csi_rxc_hs_term_en_o), 
		.mipi_rx_dat_term_en_o	(csi_rxd0_hs_term_en_o), 
		
		.CLK				(w_csi_rx_clk), 
		.VSYNC			(w_csi_rx_vsync0), 
		.HSYNC			(w_csi_rx_hsync0), 
		.DE				(w_csi_rx_dvalid), 
		.DAT				(w_csi_rx_data_rel_raw)
	);
	assign w_csi_rx_data = {2{w_csi_rx_data_rel_raw[47:40], w_csi_rx_data_rel_raw[35:28], w_csi_rx_data_rel_raw[23:16], w_csi_rx_data_rel_raw[11:4]}}; 
	assign {csi_rxd3_hs_en_o, csi_rxd2_hs_en_o, csi_rxd1_hs_en_o} = {3{csi_rxd0_hs_en_o}}; 
	assign {csi_rxd3_hs_term_en_o, csi_rxd2_hs_term_en_o, csi_rxd1_hs_term_en_o} = {3{csi_rxd0_hs_term_en_o}}; 
	
`endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//bayer2ram  
wire			cmos_frame_vsync = w_csi_rx_vsync0;                     //  cmos frame data vsync valid signal
wire			cmos_frame_href = w_csi_rx_hsync0 && w_csi_rx_dvalid;	 //  cmos frame data href vaild  signal
wire	[63:0]	cmos_frame_Gray = {w_csi_rx_data[7:0], w_csi_rx_data[15:8], w_csi_rx_data[23:16], w_csi_rx_data[31:24], w_csi_rx_data[39:32], w_csi_rx_data[47:40], w_csi_rx_data[55:48], w_csi_rx_data[63:56]}; 

   reg    ori_cmos_frame_vsync   ;
                                 
   reg    ori_cmos_frame_href    ;
                                 
   reg    [31:0]    ori_cmos_frame_Gray    ;
    
    always @(posedge w_csi_rx_clk or negedge rst_n) begin
        if(!rst_n)
            ori_cmos_frame_vsync <= 1'b0;
        else begin
            ori_cmos_frame_vsync <= cmos_frame_vsync;
        end
    end
    

    
    always @(posedge w_csi_rx_clk or negedge rst_n) begin
        if(!rst_n)
            ori_cmos_frame_href <= 1'b0;
        else begin
            ori_cmos_frame_href <= cmos_frame_href;
        end
    end
    

    
    always @(posedge w_csi_rx_clk or negedge rst_n) begin
        if(!rst_n)
            ori_cmos_frame_Gray <= 32'h00000000FF;
        else begin
            ori_cmos_frame_Gray <= cmos_frame_Gray;
        end
    end
    






/////////////////////////////////////////////////////////////////
	//ddr(双口w_fifo,双口r_fifo)

	//	General AXI Interface 
	wire	[3:0] 	w_ddr3_awid;
	wire	[31:0]	w_ddr3_awaddr;
	wire	[7:0]	w_ddr3_awlen;
	wire			w_ddr3_awvalid;
	wire			w_ddr3_awready;
	
	wire 	[3:0]  	w_ddr3_wid;
	wire 	[127:0] w_ddr3_wdata;
	wire 	[15:0]	w_ddr3_wstrb;
	wire			w_ddr3_wlast;
	wire			w_ddr3_wvalid;
	wire			w_ddr3_wready;
	
	wire 	[3:0] 	w_ddr3_bid;
	wire 	[1:0] 	w_ddr3_bresp;
	wire			w_ddr3_bvalid;
	wire			w_ddr3_bready;
	
	wire	[3:0] 	w_ddr3_arid;
	wire	[31:0]	w_ddr3_araddr;
	wire	[7:0]	w_ddr3_arlen;
	wire			w_ddr3_arvalid;
	wire			w_ddr3_arready;
	
	wire 	[3:0] 	w_ddr3_rid;
	wire 	[127:0] w_ddr3_rdata;
	wire			w_ddr3_rlast;
	wire			w_ddr3_rvalid;
	wire			w_ddr3_rready;
	wire 	[1:0] 	w_ddr3_rresp;
	
	
	//	AXI Interface Request
	wire 	[3:0] 	w_ddr3_aid;
	wire 	[31:0] 	w_ddr3_aaddr;
	wire 	[7:0]  	w_ddr3_alen;
	wire 	[2:0]  	w_ddr3_asize;
	wire 	[1:0]  	w_ddr3_aburst;
	wire 	[1:0]  	w_ddr3_alock;
	wire			w_ddr3_avalid;
	wire			w_ddr3_aready;
	wire			w_ddr3_atype;
	
	wire 			w_ddr3_cal_done, w_ddr3_cal_pass; 
	
	//	Do not issue DDR read / write when ~cal_done. 
	reg 			r_ddr_unlock = 0; 

	//	Hardware Configuration
	assign clk_p_hi = 1'b0;	//	DDR3 Clock requires 180 degree shifted. 
	assign clk_p_lo = 1'b1;
	assign clk_n_hi = 1'b1;
	assign clk_n_lo = 1'b0; 
//-------------------------------------------上面为ddr的大部分wire

	assign ddr_pll_rstn_o=rst_n;

	wire			w_ddr3_ui_clk = clk_96m_sys;
	wire			w_ddr3_ui_rst = ~rst_n;
	wire			w_ddr3_ui_areset = ~rst_n;
	wire			w_ddr3_ui_aresetn = rst_n;
	wire [15:0]		data_vga_ddr;
	wire [7:0]		mipi_green;
    wire [7:0]		ori_mipi_green;
    wire [15:0]		algo_mipi_green;
	wire vga_blk_ddr;
	assign w_ddr3_awid = 0; 
	assign w_ddr3_wid = 0; 
    
	//算法
	wire o_frame_vs;
    wire o_line_hs;
    wire o_pixel_valid;
    wire [15:0]o_pixel_data;


	//-----------下面为ddr整个模块的主要控制，作为链接顶层和ddr内部控制逻辑的桥梁
        
	
	//	Write in 8 bits. Read in 16 bits. 
	axi4_ctrl #(   
        .C_RD_END_ADDR0(1920 * 1080), 
        .C_RD_END_ADDR1(1920 * 1080* 2), 
        .C_W_WIDTH0(32), 
        .C_R_WIDTH0(8),
        .C_W_WIDTH1(16),
        .C_R_WIDTH1(16),
        .C_ID_LEN(4)
    )u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (w_ddr3_ui_rst            ),

		.axi_awaddr     (w_ddr3_awaddr       ),
		.axi_awlen      (w_ddr3_awlen        ),
		.axi_awvalid    (w_ddr3_awvalid      ),
		.axi_awready    (w_ddr3_awready      ),

		.axi_wdata      (w_ddr3_wdata        ),
		.axi_wstrb      (w_ddr3_wstrb        ),
		.axi_wlast      (w_ddr3_wlast        ),
		.axi_wvalid     (w_ddr3_wvalid       ),
		.axi_wready     (w_ddr3_wready       ),

		.axi_bid        (0          ),
		.axi_bresp      (0        ),
		.axi_bvalid     (1       ),

		.axi_arid       (w_ddr3_arid         ),
		.axi_araddr     (w_ddr3_araddr       ),
		.axi_arlen      (w_ddr3_arlen        ),
		.axi_arvalid    (w_ddr3_arvalid      ),
		.axi_arready    (w_ddr3_arready      ),

		.axi_rid        (w_ddr3_rid          ),
		.axi_rdata      (w_ddr3_rdata        ),
		.axi_rresp      (0        ),
		.axi_rlast      (w_ddr3_rlast        ),
		.axi_rvalid     (w_ddr3_rvalid       ),
		.axi_rready     (w_ddr3_rready       ),

		.ori_wframe_pclk    (w_csi_rx_clk          ),
		.ori_wframe_vsync   (ori_cmos_frame_vsync), //w_wframe_vsync   ),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		.ori_wframe_data_en (ori_cmos_frame_href   ),
		.ori_wframe_data    (ori_cmos_frame_Gray),
		
        .algo_wframe_pclk    (clk_hdmi),
		.algo_wframe_vsync   (o_frame_vs),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		.algo_wframe_data_en (o_pixel_valid),
		.algo_wframe_data    (o_pixel_data),

		// .algo_wframe_data    (16'h0FFF),

        // .algo_wframe_pclk    (clk_hdmi          ),
		// .algo_wframe_vsync   (i_vs), //w_wframe_vsync   ),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		// .algo_wframe_data_en (i_de),
		// .algo_wframe_data    (i_data_rgb565),
        
		.ori_rframe_pclk    (clk_hdmi            ),
		.ori_rframe_vsync   (~vga_vsync             ),	//0表示开始，1是结束	//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.ori_rframe_data_en (vga_blk            ),
		.ori_rframe_data    (ori_mipi_green           ),
		
        .algo_rframe_pclk    (clk_hdmi            ),
		.algo_rframe_vsync   (~vga_vsync             ),	//0表示开始，1是结束	//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.algo_rframe_data_en (vga_blk            ),
		.algo_rframe_data    (algo_mipi_green         ),  //RGB565
        
		.tp_o 		()
	);

    assign mipi_green = ori_mipi_green;
    
    //led[0]指示显示模式，灭：ori，亮：algo
    

 		//---- ---------- ---------- ------------上为外部对ddr控制，下为ddr物理接口以及AXI相关选择器   

	always @(posedge w_ddr3_ui_clk or negedge w_ddr3_ui_aresetn) begin
		if(~w_ddr3_ui_aresetn)
			r_ddr_unlock <= 0; 
		else
			r_ddr_unlock <= w_ddr3_cal_done; 
	end
	
	DdrCtrl ddr3_ctl_axi (	
		.core_clk		(core_clk),
		.tac_clk		(tac_clk),
		.twd_clk		(twd_clk),	
		.tdqss_clk		(tdqss_clk),
		
		.reset		(reset),
		.cs			(cs),
		.ras			(ras),
		.cas			(cas),
		.we			(we),
		.cke			(cke),    
		.addr			(addr),
		.ba			(ba),
		.odt			(odt),
		
		.o_dm_hi		(o_dm_hi),
		.o_dm_lo		(o_dm_lo),
		
		.i_dq_hi		(i_dq_hi),
		.i_dq_lo		(i_dq_lo),
		.o_dq_hi		(o_dq_hi),
		.o_dq_lo		(o_dq_lo),
		.o_dq_oe		(o_dq_oe),
		
		.i_dqs_hi		(i_dqs_hi),
		.i_dqs_lo		(i_dqs_lo),
		.i_dqs_n_hi		(i_dqs_n_hi),
		.i_dqs_n_lo		(i_dqs_n_lo),
		.o_dqs_hi		(o_dqs_hi),
		.o_dqs_lo		(o_dqs_lo),
		.o_dqs_n_hi		(o_dqs_n_hi),
		.o_dqs_n_lo		(o_dqs_n_lo),
		.o_dqs_oe		(o_dqs_oe),
		.o_dqs_n_oe		(o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(w_ddr3_avalid),	//	Enable command only when unlocked. 
		.axi_aready		(w_ddr3_aready),
		.axi_aaddr		(w_ddr3_aaddr),
		.axi_aid		(w_ddr3_aid),
		.axi_alen		(w_ddr3_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(w_ddr3_atype),
		
		.axi_wid		(w_ddr3_wid),
		.axi_wvalid		(w_ddr3_wvalid),
		.axi_wready		(w_ddr3_wready),
		.axi_wdata		(w_ddr3_wdata),
		.axi_wstrb		(w_ddr3_wstrb),
		.axi_wlast		(w_ddr3_wlast),
		
		.axi_bvalid		(w_ddr3_bvalid),
		.axi_bready		(w_ddr3_bready),
		.axi_bid		(w_ddr3_bid),
		.axi_bresp		(w_ddr3_bresp),
		
		.axi_rvalid		(w_ddr3_rvalid),
		.axi_rready		(w_ddr3_rready),
		.axi_rdata		(w_ddr3_rdata),
		.axi_rid		(w_ddr3_rid),
		.axi_rresp		(w_ddr3_rresp),
		.axi_rlast		(w_ddr3_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	
	assign w_ddr3_bready = 1'b1; 
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid && r_ddr_unlock),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(w_ddr3_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid && r_ddr_unlock),
		.arready_o			(w_ddr3_arready),
		
		.aid_o				(w_ddr3_aid),
		.aaddr_o			(w_ddr3_aaddr),
		.alen_o				(w_ddr3_alen),
		.atype_o			(w_ddr3_atype),
		.avalid_o			(w_ddr3_avalid),
		.aready_i			(w_ddr3_aready)
	);
	
	assign w_ddr3_asize = 4; 		//	Fixed 128 bits (16 bytes, size = 4)
	assign w_ddr3_aburst = 1; 
	assign w_ddr3_alock = 0; 
	
  reg de;
  always @(posedge clk_hdmi or negedge rst_n) begin
	if (!rst_n) begin
		de<=0;
	end
	else
		de<=vga_blk;
  end

  assign hdmi_hsync=	(hdmi_display_now_status_r)?MIPI_rgb_hsync:vga_hsync;	
  assign hdmi_vsync=	(hdmi_display_now_status_r)?MIPI_rgb_vsync:vga_vsync;
  assign hdmi_de=		(hdmi_display_now_status_r)?MIPI_rgb_href:de;

////////////////////////////////////////////////////////////////////////

VGA_CTRL  VGA_CTRL_inst (
    .clk_hdmi(clk_hdmi),//148.5
    .data_input(algo_mipi_green),
    .rst_n(rst_n),
    .hcount(),
    .vcount(),
    .vga_clk(),
    .vga_blk(vga_blk),
    .vga_data(data_vga),
    .vga_vsync(vga_vsync),
    .vga_hsync(vga_hsync)
  );



	VIP_RAW8_RGB888 #(.IMG_HDISP(1920), .IMG_VDISP(1080)) bayer2rgb (
		.clk			(clk_hdmi),                //cmos video pixel clock
		.rst_n			(rst_n),              //global reset
	    
		.mirror			(0),

		    //CMOS YCbCr444 data output
		.per_frame_vsync		(vga_vsync),    //Prepared Image data vsync valid signal. Rese Reset on falling edge. 
		.per_frame_href		(vga_blk),     //Prepared Image data href vaild  signal
		.per_frame_hsync		(vga_hsync),     //Prepared Image data href vaild  signal
		.per_img_RAW		(mipi_green),        //Prepared Image data 8 Bit RAW Data

		.post_frame_vsync		(MIPI_rgb_vsync),   //Processed Image data vsync valid signal
		.post_frame_href		(MIPI_rgb_href),    //Processed Image data href vaild  signal
		.post_frame_hsync		(MIPI_rgb_hsync),    //Processed Image data href vaild  signal
		.post_img_red		(MIPI_rgb_r),       //Prepared Image green data to be processed 
		.post_img_green		(MIPI_rgb_g),     //Prepared Image green data to be processed
		.post_img_blue		(MIPI_rgb_b)       //Prepared Image blue data to be processed
	);
///////////////////////////////////////////////////////////
	//算法


project_top # (
    .IMG_WIDTH(1920),
    .IMG_HEIGHT(1080)
  )
  project_top_inst (
    .clk(clk_hdmi),
    .rst_n(rst_n),
    .i_vs(i_vs),
    .i_hs(i_hs),
    .i_de(i_de),
    .i_data_rgb565(i_data_rgb565),
    .o_frame_vs(o_frame_vs),
    .o_line_hs(o_line_hs),
    .o_pixel_valid(o_pixel_valid),
    .o_pixel_data(o_pixel_data)
  );



/////////////////////////////////////////////////////////////////////
	//hdmi
///////
	

//hdmi 受press1按键的控制



	wire [25:0]hcount_readram;
	wire [15:0]vcount_readram;
	assign data_input=(hdmi_display_now_status_r)?{MIPI_rgb_r,MIPI_rgb_g,MIPI_rgb_b}:{algo_mipi_green[15:11],3'h0,algo_mipi_green[10:5],2'h0,algo_mipi_green[4:0],3'h0};


dvi_sender  dvi_sender_inst (
    .clk(clk_hdmi),
    .clkx5(clk_hdmix5),
    .rst_n(rst_n),
    .blue_data(data_input[7:0]),
    .green_data({data_input[15:11],3'h0}),
    .red_data(data_input[23:16]),
    .hsync(hdmi_hsync),
    .vsync(hdmi_vsync),
    .de(hdmi_de),
	.tmds_clk_p_h(tmds_clk_p_h),
    .tmds_clk_n_h(tmds_clk_n_h),
    .tmds_clk_p_l(tmds_clk_p_l),
    .tmds_clk_n_l(tmds_clk_n_l),
    .tmds_data_p_h(tmds_data_p_h),    //rgb 
	.tmds_data_p_l(tmds_data_p_l),
    .tmds_data_n_h(tmds_data_n_h),
    .tmds_data_n_l(tmds_data_n_l)
  );




endmodule //main