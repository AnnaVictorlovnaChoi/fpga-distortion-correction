1.模块功能
（1）global_color_core.v：纯粹的数学计算核心。
（2）color_correction_top.v：顶层模块，负责I/O和例化Core，并为后续畸变校正算法模块留出接口。

2.数据流关联（信号的输入、流转和输出）
（1）DVP相机 (1080p, 60fps, RGB565)
向 color_correction_top 提供 clk (148MHz), rst_n, i_vs,
向 color_correction_top 提供 clk (148MHz), rst_n, i_vs, i_hs, i_de, i_data_rgb565 (16-bit)。

（2）color_correction_top.v (顶层模块)
职责: I/O接口, 格式转换, 同步。
数据流:
接收 i_data_rgb565 (16-bit)。
[转换] 立即将其转换为RGB888 (r_in_8b, g_in_8b, b_in_8b)。
[例化] 将 r_in_8b, g_in_8b, b_in_8b 和 i_de 传递给 global_color_core。
同时，将 i_vs 和 i_hs 压入一个3拍（CORE_LATENCY）的移位寄存器（vs_pipe, hs_pipe）。
[接收] 从 global_color_core 接收3拍延迟后的 o_valid, o_r (8-bit), o_g (8-bit), `o_bo_b (8-bit)。
[转换] 将 o_r, o_g, o_b 转换为 o_corrected_data_rgb565 (16-bit)。
[输出] 将o_corrected_data_rgb565、o_valid (重命名为 o_corrected_de)、以及来自延迟管线（vs_pipe, hs_pipe）的 o_corrected_vs, o_corrected_hs 一起输出。

（3）global_color_core.v (核心)
职责: 数学计算。
数据流:
(S1) 接收 i_r, i_g, i_b, i_valid 并寄存。
(S2) G通道直通；R和B通道数据送入2个DSP，分别乘以 R_GAIN_FX 和B_GAIN_FX。结果被寄存。
(S3) 移位器（>> Q_SHIFT）和饱和逻辑（> 255 ?）处理乘法结果。最终的 o_r, o_g, o_b 被寄存并输出。

（4）畸变校正模块 (后续)
完美接收来自 color_correction_top 的 o_corrected_vs, o_corrected_hs, o_corrected_de, o_corrected_data_rgb565，因为它们是完全同步的DVP视频流。
