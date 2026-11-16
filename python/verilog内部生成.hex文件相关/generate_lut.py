# generate_lut.py
# -----------------------------------------------------------------------------
# 离线工具：生成用于FPGA的压缩畸变校正查找表(.hex)
#
# !! V4 - 最终修复版 !!
# 修正：
# 1. (关键) 解决了Python中因数据类型不当导致的左移位(<<)操作溢出问题。
#    在进行位移操作前，将 x_val 显式转换为支持32位的整数类型。
#    这是解决 .hex 文件高位全为0的根本原因。
# 2. 保留了之前版本的所有健壮性检查和边界钳位逻辑。
# -----------------------------------------------------------------------------
import cv2
import numpy as np
import os

# --- 导入配置文件 ---
try:
    import config_dist_1080p_k3 as cfg
except ImportError:
    print("错误：找不到配置文件 'config_dist_1080p.py'。")
    exit()

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 配置 ---
DOWNSAMPLE_FACTOR = 16
Q_FRAC_BITS = 5
OUTPUT_FILENAME = "dist_map_xy_16x.hex"

def generate_and_export_lut():
    # ... (前面的步骤1到4.5保持不变，此处省略以保持简洁) ...
    # --- 1. 计算 new_camera_matrix ---
    print("\n[步骤 1] 计算新的相机内参矩阵...")
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_BASE, 
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT),
        cfg.UNDISTORT_ALPHA, 
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT)
    )
    print("ROI (Region of Interest): [x, y, w, h] =", roi)
    print("-" * 60)

    # --- 2. 计算两个模型的完整查找表 ---
    print("\n[步骤 2] 计算两个模型的完整查找表...")
    map_A_x, map_A_y = cv2.initUndistortRectifyMap(cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_BASE, None, new_camera_matrix, (cfg.IMG_WIDTH, cfg.IMG_HEIGHT), cv2.CV_32FC1)
    map_B_x, map_B_y = cv2.initUndistortRectifyMap(cfg.CAMERA_MATRIX, cfg.DIST_COEFFS_K3_ADDON, None, new_camera_matrix, (cfg.IMG_WIDTH, cfg.IMG_HEIGHT), cv2.CV_32FC1)
    print("-" * 60)

    # --- 3. 混合查找表 ---
    print("\n[步骤 3] 混合两个模型的查找表...")
    map_x = (cfg.WEIGHT_BASE * map_A_x) + (cfg.WEIGHT_K3_ADDON * map_B_x)
    map_y = (cfg.WEIGHT_BASE * map_A_y) + (cfg.WEIGHT_K3_ADDON * map_B_y)
    print("-" * 60)
    
    # --- 4. 降采样 ---
    print(f"\n[步骤 4] 对混合后的查找表进行降采样 (因子: {DOWNSAMPLE_FACTOR})...")
    h, w = map_x.shape
    grid_h = h // DOWNSAMPLE_FACTOR
    grid_w = w // DOWNSAMPLE_FACTOR
    compressed_map_x = map_x[::DOWNSAMPLE_FACTOR, ::DOWNSAMPLE_FACTOR][:grid_h, :grid_w].copy()
    compressed_map_y = map_y[::DOWNSAMPLE_FACTOR, ::DOWNSAMPLE_FACTOR][:grid_h, :grid_w].copy()
    print(f"压缩后网格尺寸: {grid_w} x {grid_h}")
    
    # --- 4.5 边界钳位 ---
    print("\n[步骤 4.5] 对坐标进行边界钳位...")
    np.clip(compressed_map_x, 0, cfg.IMG_WIDTH - 1, out=compressed_map_x)
    np.clip(compressed_map_y, 0, cfg.IMG_HEIGHT - 1, out=compressed_map_y)
    print("边界钳位完成。")
    print("-" * 60)
    
    # --- 5. 转换为定点数并导出 ---
    print(f"\n[步骤 5] 转换为 Q11.5 定点数并导出到 {OUTPUT_FILENAME}...")
    q_factor = 1 << Q_FRAC_BITS
    
    map_x_fx = np.round(compressed_map_x * q_factor).astype(np.uint16)
    map_y_fx = np.round(compressed_map_y * q_factor).astype(np.uint16)
    
    with open(OUTPUT_FILENAME, 'w') as f:
        f.write(f"// Compressed Distortion LUT for 1080p (v4 - type fixed)\n")
        f.write(f"// Grid: {grid_w}x{grid_h}, Downsample: {DOWNSAMPLE_FACTOR}x\n")
        f.write(f"// Format: {{x_coord[15:0](Q11.5), y_coord[15:0](Q11.5)}} (32 bits total)\n")
        
        for r in range(grid_h):
            for c in range(grid_w):
                x_val = map_x_fx[r, c]
                y_val = map_y_fx[r, c]
                
                # --- 【关键修复】 ---
                # 在进行位移操作前，将 x_val 显式转换为 Python 的 int 或 np.uint32
                # 以确保移位操作在32位空间内进行，而不是在16位空间内溢出。
                combined_val = (int(x_val) << 16) | int(y_val)
                # --- 【修复完毕】---

                f.write(f"{combined_val:08X}\n")
                
    print(f"\n成功导出修复后的压缩LUT到: {OUTPUT_FILENAME}")
    print("="*60)

if __name__ == '__main__':
    generate_and_export_lut()

