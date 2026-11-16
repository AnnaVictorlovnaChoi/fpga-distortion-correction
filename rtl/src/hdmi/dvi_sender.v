/**
 * @module    dvi_sender
 * @brief     使用vga时序的hdmi发送模块，使用串行输出
 * @author    HUA
 * @date      2025-09-25
 * @need modules: TMDS_encoder、serdes_4b_10to1
 */
module dvi_sender(
	 input clk,
	 input clkx5,
	 input rst_n,
	 input [7:0]blue_data,
	 input [7:0]green_data,
	 input [7:0]red_data,
	 input hsync,          // hsync data 
    input vsync,          // vsync data 
    input de,             // data enable
	output      tmds_clk_p_h,
    output      tmds_clk_n_h,
    output      tmds_clk_p_l,
    output      tmds_clk_n_l,
    output[2:0] tmds_data_p_h,    //rgb 
	output[2:0] tmds_data_p_l,
    output[2:0] tmds_data_n_h,
    output[2:0] tmds_data_n_l
);

wire [9:0]blue;
wire [9:0]green;
wire [9:0]red;
TMDS_encoder TMDS_encoder_blue(
 .clk(clk),
 .rst_n(rst_n),
 .din(blue_data),
 .c0(hsync),
 .c1(vsync),
 .dout(blue),
 .de(de)
);
TMDS_encoder TMDS_encoder_green(
 .clk(clk),
 .rst_n(rst_n),
 .din(green_data),
 .c0(1'b0),
 .c1(1'b0),
 .dout(green),
 .de(de)
);
TMDS_encoder TMDS_encoder_red(
 .clk(clk),
 .rst_n(rst_n),
 .din(red_data),
 .c0(1'b0),
 .c1(1'b0),
 .dout(red),
 .de(de)
);

 serdes_4b_10to1 serdes_4b_10to1(
	 .clkx5(clkx5),         
    .datain_0(blue),      
    .datain_1(green),      
    .datain_2(red),      
    .datain_3(10'b1111100000),      
    .data_p_h(tmds_data_p_h),
    .data_p_l(tmds_data_p_l),
    .clk_p_h(tmds_clk_p_h),
    .clk_p_l(tmds_clk_p_l),
    .data_n_h(tmds_data_n_h),
    .data_n_l(tmds_data_n_l),
    .clk_n_h(tmds_clk_n_h),
    .clk_n_l(tmds_clk_n_l)
);





endmodule