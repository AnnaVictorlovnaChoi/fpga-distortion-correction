module TMDS_encoder(
 clk,
 rst_n,
 din,
 c0,
 c1,
 dout,
 de
);
input clk;
input rst_n;
input [7:0]din;
input c0;
input c1;
input de;
output reg [9:0]dout;

localparam CTL0=10'b1101010100;//{c1,c0=0,0}
localparam CTL1=10'b0010101011;//{c1,c0=0,1}
localparam CTL2=10'b0101010100;//{c1,c0}
localparam CTL3=10'b1010101011;//{c1,c0}

reg [3:0]n1d;//N1
reg [7:0]din_q;//为了使得与N1同步得到数据需要打一拍din，从而形成一级流水线
 always @ (posedge clk) begin 
	din_q<=din;
	n1d<=din[0]+din[1]+din[2]+din[3]+din[4]+din[5]+din[6]+din[7];
 end 
 
 wire decision1;
 assign decision1=(n1d>4)|((n1d==4)&(din_q[0]==0));
 
 //下面是第一次判断处理8->9
 wire [8:0]q_m;
 assign q_m[0]=din_q[0];
 assign q_m[1]=(decision1)?~(q_m[0]^din_q[1]):(q_m[0]^din_q[1]);
 assign q_m[2]=(decision1)?~(q_m[1]^din_q[2]):(q_m[1]^din_q[2]);
 assign q_m[3]=(decision1)?~(q_m[2]^din_q[3]):(q_m[2]^din_q[3]);
 assign q_m[4]=(decision1)?~(q_m[3]^din_q[4]):(q_m[3]^din_q[4]);
 assign q_m[5]=(decision1)?~(q_m[4]^din_q[5]):(q_m[4]^din_q[5]);
 assign q_m[6]=(decision1)?~(q_m[5]^din_q[6]):(q_m[5]^din_q[6]);
 assign q_m[7]=(decision1)?~(q_m[6]^din_q[7]):(q_m[6]^din_q[7]);
 assign q_m[8]=(decision1)? 0:1;
 
 //准备9-》10
 //统计1和0
 reg[3:0]n1q;
 reg[3:0]n0q;
  always @ (posedge clk) begin 
 n1q<=q_m[0]+q_m[1]+q_m[2]+q_m[3]+q_m[4]+q_m[5]+q_m[6]+q_m[7];
 n0q<=8-(q_m[0]+q_m[1]+q_m[2]+q_m[3]+q_m[4]+q_m[5]+q_m[6]+q_m[7]);
 end
 
 reg [4:0]cnt;//4是符号位，统计之前总共多发送了多少1/0，0的符号为1
 
 wire decision2;
 assign decision2=((cnt==0)|(n1q==n0q));
 wire decision3;
 assign decision3=(((~cnt[4]) & (n1q>n0q))|(cnt[4] & (n0q>n1q)));//根据符号位判断
 
  // 流水线对齐(同步寄存器2拍) 
    reg [1:0] de_reg; 
    reg [1:0] c0_reg; 
    reg [1:0] c1_reg; 
    reg [8:0] q_m_reg; 
    always @ (posedge clk) begin 
        de_reg  <=  {de_reg[0], de}; 
        c0_reg  <=  {c0_reg[0], c0}; 
        c1_reg  <=  {c1_reg[0], c1}; 
        q_m_reg <=  q_m;
	end

always @ (posedge clk or negedge rst_n) begin 
	if (!rst_n) begin 
      dout <= 10'h0; 
      cnt <= 5'd0; 
   end 
	else begin
		if(de_reg[1])begin
			if(decision2)begin
				 dout[9]   <=  ~q_m_reg[8];  
             dout[8]   <=  q_m_reg[8];  
             dout[7:0] <=  (q_m_reg[8]) ? q_m_reg[7:0] : ~q_m_reg[7:0];
             cnt <= (~q_m_reg[8]) ? (cnt + n0q - n1q):(cnt + n1q - n0q);
			end 
			else begin 
				if(decision3)begin
				  dout[9]   <=  1;  
              dout[8]   <=  q_m_reg[8];  
              dout[7:0] <=  ~q_m_reg[7:0];
              cnt <= cnt +{q_m_reg[8], 1'b0}+ n0q - n1q;
				end 
				else begin
				  dout[9]   <=  0;  
              dout[8]   <=  q_m_reg[8];  
              dout[7:0] <=  q_m_reg[7:0]; 
              cnt <= cnt - {~q_m_reg[8], 1'b0} + n1q - n0q;
				end
			end
		end 
		else begin
			cnt <= 5'd0; 
			case ({c1_reg[1], c0_reg[1]}) 
				2'b00:   dout <=  CTL0; 
				2'b01:   dout <=  CTL1; 
				2'b10:   dout <=  CTL2; 
				default: dout <=  CTL3;
			endcase
		end
	end
end
endmodule