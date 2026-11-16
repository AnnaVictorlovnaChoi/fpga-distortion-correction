1.关于python脚本的作用
（1）config_dist_1080p_k3：存储相机和畸变的相关参数，如果要更改请直接在这个文件修改
（2）generate_lut.py：逻辑代码，这个代码只和硬件参数和畸变参数相关，运行后能生成一个dist_map_xy.hex文件，用于双线插值算法的区块划分，请将这个文件与“畸变校正算法”中的distortion_lut_rom.v放置在同一路径下，verilog中的#read函数会自动读取（应该）

2.关于verilog：
video_pipeline_top.v：
两个模块算法的功能整合，其数据流如下：
1. ->DVP相机 -> project_top (输入)

信号: i_vs, i_hs, i_de, i_data_rgb565

描述: DVP相机在clk的驱动下，逐像素地提供原始的、偏绿的、带畸变的视频流。

2. project_top -> color_correction_top

信号: i_... -> u_color_correction.i_...

描述: project_top将原始DVP信号原封不动地传递给color_correction_top模块的输入端口。

3. color_correction_top (内部)

i_data_rgb565 (16-bit) -> r_in_8b, g_in_8b, `bb_in_8b (24-bit)

i_de -> global_color_core.i_valid

RGB888 -> global_color_core

global_color_core (3拍延迟) -> r_out_8b, g_out_8b, b_out_8b

Corrected RGB888 -> cc_data_rgb565 (16-bit)

i_vs, i_hs, i_de -> vs_pipe, hs_pipe, de_pipe (3拍延迟)

输出: cc_vs, cc_hs, cc_de, cc_data_rgb565

总结: 输出是一个与输入DVP流具有**相同格式和相同格式和时序的、色彩已校正的新DVP流，只是整体存在固定的3拍延迟。

4. color_correction_top -> distortion_top

信号: cc_vs, cc_hs, cc_de, cc_data_rgb565 -> u_distortion.i_...

描述: project_top中的wire信号充当"传送带"，将工位1的输出直接连接到工位2的输入。

5. distortion_top (内部)

i_de (cc_de) -> u_line_buf_ping_ac/bd.wru_line_buf_ping_ac/bd.wr_en (或 pong，由ping_pong_sel决定)

 (cc_i_data_rgb565 (cc_data) -> u_line_buf_ping_ac/bd.wr_data

i_de (cc_de), out_x_cnt, out_y_cnt -> u_distortion_core.i_valid/i_x/i_y

u_distortion_core.o_lut_addr_a/b/c/d -> u_lut_rom.addr_a/b/c/d

u_lut_rom.(x,y)_map_a/b/c/d -> u_distortion_core.i_(x,y)_map_a/b/c/d

u_distortion_core.o_linebuf_addr_a/b/c/d -> "读" line_buffer 实例的 rd_addr

"读" line_buffer.rd_data_a/b -> distortion_core.i_pixel_a/b/c/d

u_distortion_core.o_valid (10拍延迟) -> core_valid

u_distortion_core.o_pixel_data (10拍延迟) -> core_pixel_data

(cc_vs), i_hs (cc_hs) -> vs_pipe, hs_pipe (10拍延迟)

out_x_cnt, out_y_cnt -> out_x/y_pipe (10拍延迟) -> final_x, final_y

in_roi = (final_x, final_y) 与 ROI 参数比较

输出:

o_vs = vs_pipe 末端

o_hs = hs_pipe 末端

o_de = core_valid && in_roi

o_data_rgb565 = core_pixel_data

总结: distortion_top接收一个DVP流，输出一个**畸变已校正畸变已校正、已裁剪、且同步的新DVP流。它在color_correction_top的3拍延迟基础上，又增加了约10拍的延迟。

6. distortion_top -> project_top (输出)

信号: u_distortion.o_... -> o_...

描述: distortion_top的最终输出被直接连接到project_top的输出端口，准备被DDR内存控制器捕获。

DDR控制器操作:

当o_frame_vs变为高电平时，DDR控制器准备切换缓冲区。

当o_pixel_valid为高电平时，DDR控制器将o_pixel_data写入其内部FIFO，并计算DDR的写入地址。

当o_line_hs变为高电平时，DDR控制器将其DDR地址指针增加一个"行步长(stride)"。