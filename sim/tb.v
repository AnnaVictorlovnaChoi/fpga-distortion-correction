//tmds没问题，但是serdes有问题
module tb;

  // Parameters

  //Ports
  reg clk;
  reg clkx5;
  reg rst_n;
  reg [7:0] blue_data;
  reg [7:0] green_data;
  reg [7:0] red_data;
  reg hsync;
  reg vsync;
  reg de;
  wire tmds_clk_p_h;
  wire tmds_clk_n_h;
  wire tmds_clk_p_l;
  wire tmds_clk_n_l;
  wire [2:0] tmds_data_p_h;
  wire [2:0] tmds_data_p_l;
  wire [2:0] tmds_data_n_h;
  wire [2:0] tmds_data_n_l;

  dvi_sender  dvi_sender_inst (
    .clk(clk),
    .clkx5(clkx5),
    .rst_n(rst_n),
    .blue_data(blue_data),
    .green_data(green_data),
    .red_data(red_data),
    .hsync(hsync),
    .vsync(vsync),
    .de(de),
    .tmds_clk_p_h(tmds_clk_p_h),
    .tmds_clk_n_h(tmds_clk_n_h),
    .tmds_clk_p_l(tmds_clk_p_l),
    .tmds_clk_n_l(tmds_clk_n_l),
    .tmds_data_p_h(tmds_data_p_h),
    .tmds_data_p_l(tmds_data_p_l),
    .tmds_data_n_h(tmds_data_n_h),
    .tmds_data_n_l(tmds_data_n_l)
  );

initial clkx5=1;
always #1  clkx5 = ! clkx5 ;
initial clk=1;
always #5  clk = ! clk ;
initial begin
    rst_n=0;
    hsync=0;
    vsync=0;
    blue_data=8'hF0;
    green_data=8'h0F;
    red_data=8'h01;
    de=0;
	#201
	rst_n=1;
    de=1;
	#800_000
	$stop;



end

endmodule