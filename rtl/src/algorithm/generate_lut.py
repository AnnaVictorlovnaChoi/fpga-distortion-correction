# generate_lut.py
# -----------------------------------------------------------------------------
# 模块名称：generate_lut (Offline Distortion Correction LUT Generator)
# 基本功能：离线生成适用于FPGA的压缩畸变校正查找表（.hex格式），专为1080P分辨率相机设计，通过降采样优化减少BRAM资源占用。
# 基本介绍：
#   - 依赖1080P相机配置文件（config_dist_1080p_k3），获取相机内参、畸变系数、权重等关键参数；
#   - 核心处理流程：导入配置参数→计算两种畸变模型的校正映射→加权融合映射结果→16x降采样压缩→转换为Q11.5定点格式→导出为.hex文件；
#   - 关键配置：降采样因子16（大幅减少存储量），定点格式采用Q11.5（11位整数+5位小数，共16位），输出文件为"dist_map_xy_16x.hex"；
#   - 适配FPGA实现：生成的查找表可直接被畸变校正模块（如distortion_lut_rom）调用，为硬件畸变校正提供坐标映射数据。
# 核心畸变校正数学处理逻辑：
#   基于相机内参矩阵，使用OpenCV的initUndistortRectifyMap分别计算基础模型（k1、k2）和k3辅助模型的畸变校正映射（map_A、map_B）；通过配置文件中的权重（WEIGHT_BASE、WEIGHT_K3_ADDON）对两种映射进行加权融合，得到最终校正映射（map_x、map_y）；对融合后的映射进行16x降采样压缩，再通过Q11.5定点化（乘以32并取整）转换为硬件可直接使用的整数格式，最终导出为.hex文件。
# 修正记录：
#   1. 优化版：将降采样因子从8提高至16，大幅减少BRAM资源占用，同时保持校正精度；调整输出文件名以区分不同降采样版本。
# 设计人员：王博
# 最后修改时间：2025-11-05]
# -----------------------------------------------------------------------------
import cv2
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 关键：导入我们新的 1080P 配置文件 ---
try:
    # 确保您的配置文件名为 config_dist_1080p.py
    import config_dist_1080p_k3 as cfg
except ImportError:
    print("错误：找不到配置文件 'config_dist_1080p.py'。")
    print("请确保它与本脚本在同一目录下。")
    exit()

# --- 配置 ---
# **优化**: 将降采样因子从 8 提高到 16
DOWNSAMPLE_FACTOR = 16
# Verilog中使用的定点格式: Q11.5 (11位整数, 5位小数) -> 总共16位
Q_FRAC_BITS = 5 
OUTPUT_FILENAME = "dist_map_xy_16x.hex" # 新文件名以作区分

def generate_and_export_lut():
    """
    执行完整的LUT计算和导出流程。
    """
    print("正在根据 1080P 相机参数计算完整的畸变映射表...")
    
    # --- 这部分逻辑与您之前的 distortion_corrector_1080p.py 完全相同 ---
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_BASE, (cfg.IMG_WIDTH, cfg.IMG_HEIGHT),
        cfg.UNDISTORT_ALPHA, (cfg.IMG_WIDTH, cfg.IMG_HEIGHT)
    )
    map_A_x, map_A_y = cv2.initUndistortRectifyMap(
        cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_BASE, None, new_camera_matrix,
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT), cv2.CV_32FC1
    )
    map_B_x, map_B_y = cv2.initUndistortRectifyMap(
        cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_K3_ADDON, None, new_camera_matrix,
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT), cv2.CV_32FC1
    )
    map_x = (cfg.WEIGHT_BASE * map_A_x) + (cfg.WEIGHT_K3_ADDON * map_B_x)
    map_y = (cfg.WEIGHT_BASE * map_A_y) + (cfg.WEIGHT_K3_ADDON * map_B_y)
    
    print("全尺寸查找表 (Blended LUT) 生成完毕！")

    # --- 降采样、转换为定点数并导出 ---
    print(f"开始降采样 (因子: {DOWNSAMPLE_FACTOR}) 并导出到 {OUTPUT_FILENAME}...")
    
    # 确保降采样后的尺寸正确
    # (1920/16 = 120, 1080/16 = 67.5 -> 67)
    h, w = map_x.shape
    grid_h = h // DOWNSAMPLE_FACTOR
    grid_w = w // DOWNSAMPLE_FACTOR

    compressed_map_x = map_x[::DOWNSAMPLE_FACTOR, ::DOWNSAMPLE_FACTOR][:grid_h, :grid_w]
    compressed_map_y = map_y[::DOWNSAMPLE_FACTOR, ::DOWNSAMPLE_FACTOR][:grid_h, :grid_w]
    
    print(f"LUT压缩后网格尺寸: {grid_w} x {grid_h}")

    q_factor = 1 << Q_FRAC_BITS
    
    # 转换为 Q11.5 定点格式
    map_x_fx = np.round(compressed_map_x * q_factor).astype(np.uint16)
    map_y_fx = np.round(compressed_map_y * q_factor).astype(np.uint16)
    
    with open(OUTPUT_FILENAME, 'w') as f:
        f.write(f"// Compressed Distortion LUT for 1080p\n")
        f.write(f"// Grid: {grid_w}x{grid_h}, Downsample: {DOWNSAMPLE_FACTOR}x\n")
        f.write(f"// Format: {{x_coord[15:0](Q11.5), y_coord[15:0](Q11.5)}} (32 bits total)\n")
        
        for r in range(grid_h):
            for c in range(grid_w):
                x_val = map_x_fx[r, c]
                y_val = map_y_fx[r, c]
                
                # 拼接成一个32位的数
                combined_val = (x_val << 16) | y_val
                f.write(f"{combined_val:08X}\n") # 32 bits -> 8 hex chars
                
    print(f"成功导出压缩LUT到: {OUTPUT_FILENAME}")
    print(f"请将此文件与您的Verilog工程放在一起，并更新 distortion_lut_rom 模块。")

if __name__ == '__main__':
    generate_and_export_lut()
