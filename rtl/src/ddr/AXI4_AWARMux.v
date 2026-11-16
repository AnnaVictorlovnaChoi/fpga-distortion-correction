`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    15:58:46 04/08/2023 
// Module Name:    AXI4_AWARMux 
//////////////////////////////////////////////////////////////////////////////////

//	AXI4 AW / AR Path Mux. 
//	This module works with Efinix AXI4-Modified Interface. 
//	atype_o = 1 for DDR Write, and atype_o = 0 for DDR Read. 

/* 改进工作：
 * 模块名称：AXI4_AWARMux (AXI4 AW/AR Path Multiplexer)
 * 基本功能：实现AXI4协议中写地址（AW）和读地址（AR）路径的多路复用，适配Efinix修改版AXI4接口，用于在读写地址通道之间进行切换，以轮询方式处理读写请求。
 * 基本介绍：
 *   - 参数配置：支持可配置的地址ID长度（AID_LEN）和地址长度（AADDR_LEN），默认分别为8位和32位；
 *   - 接口说明：
 *     - 输入接口：包含AW通道（awid_i/awaddr_i/awlen_i/awvalid_i）和AR通道（arid_i/araddr_i/arlen_i/arvalid_i）的信号，以及时钟（aclk_i）、复位（arst_i）和下游准备信号（aready_i）；
 *     - 输出接口：复用后的地址ID（aid_o）、地址（aaddr_o）、长度（alen_o）、类型标识（atype_o，1表示写，0表示读），以及上游准备信号（awready_o/arready_o）和复用后有效信号（avalid_o）；
 *   - 工作机制：通过状态机（rs_req）实现轮询模式，交替处理AW和AR请求，当检测到对应通道的valid信号且下游ready时，将该通道信号映射到输出，并通过atype_o标识当前传输类型。
 * 核心多路复用逻辑：
 *   采用轮询调度策略，通过状态机在空闲（ws_req_idle）、桥接（ws_req_bridge）和切换（ws_req_inc）状态间切换；通过r_reqtype标识当前处理类型（0为读，1为写），根据类型将AR或AW通道的信号复用至输出，同时生成对应的ready信号，确保读写地址请求有序传输。
 * 修正记录：
 *   1. 初始版本：实现AXI4 AW/AR通道的轮询复用功能，适配Efinix修改版AXI4接口，完成基本的读写地址信号切换与传输控制。
 * 设计人员：王博、张智华、李伟亮
 * 最后修改时间：2025-11-05
 */

module AXI4_AWARMux #(
	parameter 	AID_LEN 	= 8, 
			AADDR_LEN 	= 32
)(
	input 				aclk_i, 
	input 				arst_i, 
	
	//	AW
	input 	[AID_LEN-1:0] 	awid_i,
	input 	[AADDR_LEN-1:0] 	awaddr_i,
	input 	[7:0] 		awlen_i,
	input 				awvalid_i,
	output reg				awready_o,
	
	//	AR
	input 	[AID_LEN-1:0] 	arid_i,
	input 	[AADDR_LEN-1:0] 	araddr_i,
	input 	[7:0] 		arlen_i,
	input 				arvalid_i,
	output reg				arready_o,
	
	//	A
	output 	[AID_LEN-1:0] 	aid_o,
	output 	[AADDR_LEN-1:0] 	aaddr_o,
	output 	[7:0] 		alen_o,
	output 				atype_o,
	output reg				avalid_o,
	input 				aready_i
);

	//	R/W Wrapper. atype_o = 1 for DDR Write, and atype_o = 0 for DDR Read. 
	//	Assert R/W in round robin mode. 
	reg 	[1:0] 	rs_req = 0; 		//	DDR3 Request State Machine
	wire 	[1:0] 	ws_req_idle = 0; 
	wire 	[1:0] 	ws_req_bridge = 1; 
	wire 	[1:0] 	ws_req_inc = 2; 
	
	reg 			r_reqtype = 0; 	//	Type = 0 for read, and type = 1 for write. Toggle when aready. 
	reg 			r_reqen = 0; 	//	Request En
	
	always @(posedge aclk_i) begin
		//	Clear AREADY by default. 
		arready_o <= 0; 
		awready_o <= 0; 
		
		case (rs_req) 
			ws_req_idle: begin
					//	By default toggle reqtype when no commands. 
					rs_req <= ws_req_inc; 
					
					//	Type = 0 for read. 
					if(r_reqtype == 0) begin
						//	When read request, bridge to AR path. 
						if(arvalid_i) begin
							avalid_o <= 1; 
							rs_req <= ws_req_bridge; 
						end else begin
						end
					end else begin
						//	When write request, bridge to AR path. 
						if(awvalid_i) begin
							avalid_o <= 1; 
							rs_req <= ws_req_bridge; 
						end else begin
						end
					end
				end
			ws_req_bridge: begin
					if(aready_i) begin
						avalid_o <= 0; 
						
						//	Respond to AR / AW. 
						arready_o <= (r_reqtype == 0); 
						awready_o <= (r_reqtype == 1); 
						rs_req <= ws_req_inc; 
					end else begin
					end
				end
			ws_req_inc: begin
					r_reqtype <= r_reqtype + 1; 
					rs_req <= ws_req_idle; 
				end
		endcase
		
		if(arst_i) begin
			rs_req <= 0; 
			r_reqtype <= 0; 
			r_reqen <= 0; 
			
			//awready_o <= 0; 
			//arready_o <= 0; 
			avalid_o <= 0; 
		end else begin
		end
	end
	
	//	Type = 0 for read. Bridge to AR path. 
	assign aid_o = (r_reqtype == 0) ? arid_i : awid_i; 
	assign aaddr_o = (r_reqtype == 0) ? araddr_i : awaddr_i; 
	assign alen_o = (r_reqtype == 0) ? arlen_i : awlen_i; 
	assign atype_o = r_reqtype; 

endmodule
