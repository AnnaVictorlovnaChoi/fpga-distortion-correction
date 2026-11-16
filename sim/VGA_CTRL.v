module VGA_CTRL(//1920x1080分辨率的vga驱动
clk_hdmi,//148.5
rst_n,
data_input,//对于1个像素的rgb888数据
hcount,
vcount,
vga_blk,//数据使能信号
vga_clk,
vga_hsync,//行同步
vga_vsync,//场同步（类似en）
vga_data
);
input clk_hdmi;
input [23:0]data_input;
input rst_n;
output [25:0]hcount;
output [15:0]vcount;
output vga_clk;
output vga_blk;
output [23:0]vga_data;
output vga_vsync;
output vga_hsync;

//行
localparam HS_END=44-1;
localparam HDAT_BEGIN=192-1;
localparam HDAT_END=2112-1;
localparam HSYNC_END=2200-1;
//场
localparam VS_END=5-1;
localparam VDAT_BEGIN=41-1;
localparam VDAT_END=1121-1;
localparam VSYNC_END=1125-1;

assign vga_clk=clk_hdmi;
assign vga_data=vga_blk?data_input:0;

reg [30:0]h_cnt;
reg [30:0]v_cnt;
always @(posedge clk_hdmi or negedge rst_n)
	if(!rst_n)
		h_cnt<=0;
	else if(h_cnt==HSYNC_END-1)
		h_cnt<=0;
	else
		h_cnt<=h_cnt+1;

assign vga_hsync=(h_cnt<HS_END)?0:1;		
		
always @(posedge clk_hdmi or negedge rst_n)
	if(!rst_n)
		v_cnt<=0;
	else if(h_cnt==HSYNC_END-1)begin
		if(v_cnt==VSYNC_END-1)
			v_cnt<=0;
		else
			v_cnt<=v_cnt+1;
	end
	
	
assign vga_vsync=(v_cnt<VS_END)?0:1;	

assign vga_blk=(h_cnt>=HDAT_BEGIN&&h_cnt<HDAT_END&&
					v_cnt>=VDAT_BEGIN&&v_cnt<VDAT_END)?1:0;
//根据blk是不是在数据输出从而输出当前输入数据位
assign hcount=vga_blk?(h_cnt-HDAT_BEGIN):0;
assign vcount=vga_blk?(v_cnt-VDAT_BEGIN):0;
endmodule 