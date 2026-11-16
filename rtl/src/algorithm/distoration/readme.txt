1.模块功能：
distortion_top (工位2: 畸变校正)

distortion_lut_rom (工位2的蓝图/数据库)

line_buffer_1080p (工位2的缓存, x4)

distortion_core (工位2的核心数学逻辑，利用双线插值，一共10个流水线)

2.详细数据流转关系 (以distortion_top为核心)

（0）色彩校正 (工位1):

color_correction_top 接收原始DVP流 (i_vs, i_hs, i_de, i_data_rgb565)。

它在内部将i_data_rgb565传递给global_color_core。

global_color_core 使用2个DSP将R和B乘以R_GAIN_FX / B_GAIN_FX。

color_correction_top 输出一个色彩已校正、但仍完全同步的DVP流 (cc_vs, cc_hs, cc_de, cc_data)。

（1）数据进入畸变校正 (工位2):

distortion_top 接收工位1的cc_... DVP流。

distortion_top 内部的 in_x_cnt 和 in_y_cnt 跟踪输入像素的位置。

ping_pong_sel 信号决定哪两个line_buffer是“写缓存”。

cc_data (例如第N行像素) 和 in_x_cnt 被送到 "写缓存" (u_line_buf_ping_ac, u_line_buf_ping_bd) 的写端口。

（2）L1插值 (计算源坐标):

distortion_top 内部的 out_x_cnt 和 out_y_cnt 跟踪输出像素的位置。

out_x_cnt, out_y_cnt 和 i_de 被送入 distortion_core 的 i_x, i_y, i_valid。

[S1]: distortion_core 根据 i_x, i_y 计算出压缩网格的基地址 base_addr 和插值权重 coord_weight_x/y。

[S1 -> Top]: distortion_core 输出4个LUT地址 (o_lut_addr_a/b/c/d)。

[Top -> ROM]: distortion_top 将这4个地址路由到 u_lut_rom。

[ROM -> Top -> Core]: u_lut_rom (有1拍延迟) 输出4个定点坐标 ((x,y)_map_a/b/c/d)，它们被distortion_top路由回distortion_core的i_x_map_a/b/c/d...

[S2-S5]: distortion_core 在其内部流水线（S2, S3, S4）中执行L1插值，最终在S5计算出精确的源坐标 (src_x_fx_s5, src_y_fx_s5)。

（4）L2插值 (抓取源像素):

[S5 -> Top]: distortion_core 从src_x_fx_s5中提取整数部分 x_int，并输出4个行缓存地址 (o_linebuf_addr_a/b/c/d)。

[Top -> Buffers]: distortion_top 根据 ping_pong_sel，将这4个地址路由到 "读缓存" (u_line_buf_pong_ac, u_line_buf_pong_bd)。

[Buffers -> Top -> Core]: "读缓存" (存有第N-1行) 输出4个像素 (pixel_from_pong_a/b/c/d)，它们被distortion_top路由回distortion_core的i_pixel_a/b/c/d。

[S6-S10]: distortion_core 在其内部流水线（S6-S9）中执行L2插值（像素颜色插值），最终在S10（o_valid）输出最终的 o_pixel_data。

（5）输出与同步:

distortion_top 将原始的 i_vs, i_hs 信号通过一个CORE_LATENCY (10拍) 的移位寄存器 (vs_pipe, hs_pipe)。

distortion_top 比较延迟后的坐标 final_x, final_y 与 ROI 参数，生成 in_roi 信号。

最终的 o_de 是 core_valid (来自distortion_core) 和 in_roi 的与逻辑。

o_vs (延迟后), o_hs (延迟后), o_de (延迟且裁剪后), o_pixel_data (延迟且校正后) 作为最终的DVP流输出。

