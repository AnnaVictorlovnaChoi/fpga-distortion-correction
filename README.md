# FPGA 实时畸变校正 IP（含简易道路识别）

> 2025 全国大学生 FPGA 创新设计竞赛 · **全国二等奖**
> 作品名：《万物归正：AI 视觉领航仪畸变校正》
> 平台：易灵思（Efinix）Titanium **Ti60F225** + SC2210 摄像头
> 语言/工具：Verilog、Python（OpenCV 预计算）、Efinix Efinity

面向无人机 / 机器人视觉 / ADAS 这类**对延迟敏感**的场景，做一个能把广角镜头的桶形畸变
在视频流上**实时校正掉**的 FPGA IP 核。目标吞吐 **1080P@60Hz（148.5 MHz）/ 720P@60Hz（74.25 MHz）**。

---

## 1. 这个项目想解决什么问题

广角镜头是机器视觉的刚需（视场大、信息多），但代价是**画面边缘严重畸变**。
传统做法是软件后处理：**要么先存一整帧到 DDR 再逐像素重映射，要么在 CPU/GPU 上跑**。
两条路都有同一个病根——**帧缓冲带来的 16.7 ms（一帧）级别延迟**，
对 FPV 操控、SLAM 前端、车道线实时检测来说是不可接受的。

本项目把重映射做成**全流水线硬件**，用行缓存替代帧缓存，让校正随视频流"边进边出"。

---

## 2. 核心设计

### 2.1 「离线预计算 + 在线插值」两段式架构

```
离线（Python / OpenCV，PC 端一次性完成）
  ├─ config_dist_1080p_k3.py   配置相机内参 K、畸变系数 D、缩放系数 Alpha、模型混合权重
  ├─ generate_lut.py           生成 1920×1080 (1280×720) 完整映射图
  ├─ 16×16 网格降采样          把 LUT 存储需求从 16 MB 压到 63 KB（256:1）
  └─ Q11.5 定点量化            输出 dist_map_xy_16x.hex，可直接进 BRAM
                    │
                    ▼
在线（Verilog，板上实时）
  ├─ L1 坐标插值               从 BRAM 并发读 4 个网格锚点 → 插值出亚像素源坐标 (x_src, y_src)
  ├─ L2 像素插值              按 (x_src, y_src) 从行缓存并发读 4 邻域像素 → 双线性插值出最终像素
  └─ 1PPC（单像素/时钟）全流水线
```

### 2.2 三个关键优化

| 优化 | 做法 | 效果 |
|---|---|---|
| **超低延迟** | 彻底弃用 DDR 帧缓存（原本要等一整帧 ≈ 16.7 ms），改为 4 组 BRAM 的 **Ping-Pong 行缓存**流式架构 | 核心延迟降到**约 100 ns**（微秒级以下） |
| **访存优化** | 设计地址预计算单元 + DMA，把 DDR 的随机读转换为对 BRAM 的**批量 Burst 读** | 消除 DDR 访问瓶颈 |
| **在线自适应** | FPGA 侧做直线检测、软核侧跑优化器，构成"自愈"反馈回路，可在温漂 / 振动导致参数漂移时在线微调校正参数 | 延长免维护周期 |

### 2.3 修掉的三类硬件 bug（都在 `docs/` 的设计笔记里有记录）

1. **抗截断（V6.7）**：L1/L2 插值器里 `(1<<N)-0` 导致的差一截断，表现为网格黑线与零散横线；
2. **边缘复制（V6.5）**：L2 插值在图像物理边缘（如 X=1919）采样 X+1 时出现黑点；
3. **时序同步（V10.4）**：把总流水线延迟精确参数化为 12 拍，保证 `o_pixel_valid` 与 `o_frame_vs` 对齐，解决全黑屏。

### 2.4 附带功能

- **色彩校正 + Gamma**：`color_correction_top.v` / `global_color_core.v` / `gamma_correction_lut.v` 组成轻量 ISP 通路；
- **简易道路识别**：在校正后的画面上做基础车道/道路检测，作为"校正之后画面可被算法使用"的端到端验证。

---

## 3. 目录结构

```
.
├── rtl/src/
│   ├── algorithm/
│   │   ├── distortion_top.v          畸变校正顶层
│   │   ├── distortion_core.v         L1/L2 两级插值流水
│   │   ├── distortion_lut_rom.v      LUT 存储与并发读取
│   │   ├── line_buffer_1080p.v       Ping-Pong 行缓存
│   │   ├── color_correction_top.v    色彩校正
│   │   ├── global_color_core.v       全局色彩核心
│   │   └── gamma_correction_lut.v    Gamma 查表
│   ├── ov5640/  sc2210/              CMOS 传感器接口与配置
│   ├── mipi/                         MIPI CSI-2 接收
│   ├── raw2rgb/                      Bayer → RGB
│   ├── hdmi/                         HDMI 输出时序
│   ├── ddr/                          DDR 控制器接口
│   └── button/                       板载按键（原图/校正后切换）
├── python/                           LUT 离线生成链（OpenCV）
│   ├── config_dist_1080p_k3.py       内参/畸变系数/Alpha 配置
│   ├── generate_lut.py               生成映射表 + 网格降采样 + Q11.5 量化
│   ├── distortion_corrector_1080P_k3.py  软件参考实现（与硬件对拍用）
│   ├── color_reconstruction.py       色彩重建
│   ├── optimize_distortion_params.py 参数优化
│   └── calculate_verilog_gains.py    反推 Verilog 定点增益
├── sim/                              关键模块 testbench
└── docs/
    ├── 技术文档.pdf                   完整技术文档（33 页）
    ├── 决赛答辩PPT.pptx               决赛展示
    ├── 总体方案与架构.txt              设计总体方案
    ├── 畸变矫正算法研究.pdf            算法调研
    ├── 工作流程图.svg                 数据通路流程图
    └── 作品名称与简介.docx
```

> **未开源**：比特流、Efinity 工程文件、PnR / 时序报告、原始测试视频、获奖证书与参赛材料。

---

## 4. 关键参数速查

| 项 | 值 |
|---|---|
| 目标分辨率 / 时钟 | 1080P@60Hz (148.5 MHz) / 720P@60Hz (74.25 MHz) |
| 处理吞吐 | 1 PPC（1 pixel / clock） |
| LUT 存储 | 16 MB → **63 KB**（16×16 网格，压比 **256:1**） |
| 坐标精度 | Q11.5 定点 |
| 核心延迟 | 弃用帧缓存前 16.7 ms → 行缓存后 **≈ 100 ns** |
| 流水线深度 | 12 拍（参数化，严格同步） |
| 传感器 | SC2210（另支持 OV5640 通路） |

---

## 5. 我负责的部分

- 「离线预计算 + 在线插值」整体架构设计与资源预算；
- LUT 生成链（Python/OpenCV）与网格降采样、定点量化方案；
- L1 坐标插值 + L2 双线性像素插值的 RTL 实现与流水线时序收敛；
- Ping-Pong 行缓存替代帧缓存的延迟优化；
- DMA 地址预计算与 Burst 读改造；
- 三类硬件 bug 的定位与修复（截断、边缘采样、12 拍同步）；
- 简易道路识别的联调验证。

---

## 6. 关联项目

- [**FPGA 2D 图形渲染加速引擎**](https://github.com/AnnaVictorlovnaChoi/fpga-2d-render-accelerator)（2026，全国三等奖）
  —— 同一块 Ti60F225 上的 CPU+GPU 异构渲染架构，本项目的技术延续。
