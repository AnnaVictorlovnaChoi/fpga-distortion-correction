# optimize_distortion_params.py
# -----------------------------------------------------------------------------
# 使用 Scipy 优化器自动寻找最佳的畸变校正参数 (k1, k2, k3, alpha)
#
# 依赖:
# pip install opencv-python numpy scipy
#
# 使用方法:
# 1. 将此脚本与 'config_dist_1080p_k3.py' 放在同一目录。
# 2. 准备一张 *原始的、畸变的* 1920x1080 图像，命名为 'distorted_grid_image.png'。
#    此图像必须包含清晰的直线 (如棋盘格、网格、建筑边缘)。
# 3. 运行: python optimize_distortion_params.py
# -----------------------------------------------------------------------------
import cv2
import numpy as np
import os
from scipy.optimize import minimize
import config_dist_1080p_k3 as cfg # 导入配置以获取 K 矩阵

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 1. 全局配置 ---
# 图像必须存在!
INPUT_IMAGE_FILE = 'distorted_grid_image.png' 
IMG_WIDTH = cfg.IMG_WIDTH
IMG_HEIGHT = cfg.IMG_HEIGHT
CAMERA_MATRIX = cfg.CAMERA_MATRIX

# 优化器将尝试缩小此图像以加快速度。 0.5 = 960x540
# (如果速度太慢，可以调低；如果精度不足，调高到 1.0)
OPTIMIZE_SCALE = 0.5 

# 加载并缩放输入图像
if not os.path.exists(INPUT_IMAGE_FILE):
    print(f"!! 错误: 未找到输入图像 '{INPUT_IMAGE_FILE}'。")
    print("请提供一张包含直线的原始畸变图像用于优化。")
    exit()

print(f"加载输入图像: {INPUT_IMAGE_FILE}")
full_distorted_img = cv2.imread(INPUT_IMAGE_FILE, cv2.IMREAD_GRAYSCALE)
if full_distorted_img is None:
    print("!! 错误: 无法读取图像。")
    exit()

# 缩放图像和相机内参以加快优化速度
h, w = full_distorted_img.shape
opt_w = int(w * OPTIMIZE_SCALE)
opt_h = int(h * OPTIMIZE_SCALE)
distorted_img = cv2.resize(full_distorted_img, (opt_w, opt_h), interpolation=cv2.INTER_LINEAR)

K_scaled = CAMERA_MATRIX.copy()
K_scaled[0, :] *= OPTIMIZE_SCALE
K_scaled[1, :] *= OPTIMIZE_SCALE

print(f"优化尺寸: {opt_w}x{opt_h}")

# --- 2. 评分/损失函数 ---
# 目标: 找到使校正后图像中 "直线总长度" 最大化的参数
# (Scipy 是一个最小化器, 所以我们返回 "负" 的总得分)
def objective_function(params):
    k1, k2, k3, alpha = params
    
    # 打印当前尝试的参数 (可选, 但有助于观察)
    print(f"尝试: k1={k1:.4f}, k2={k2:.4f}, k3={k3:.4f}, alpha={alpha:.2f}")

    dist_coeffs = np.array([k1, k2, 0.0, 0.0, k3]) # [k1, k2, p1, p2, k3]

    try:
        # 步骤 1: 计算校正映射
        new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(
            K_scaled, dist_coeffs, (opt_w, opt_h), alpha, (opt_w, opt_h)
        )
        map_x, map_y = cv2.initUndistortRectifyMap(
            K_scaled, dist_coeffs, None, new_cam_matrix, (opt_w, opt_h), cv2.CV_32FC1
        )
        
        # 步骤 2: 校正图像
        corrected_img = cv2.remap(distorted_img, map_x, map_y, cv2.INTER_LINEAR)
        
        # 步骤 3: 边缘检测
        # (高斯模糊以减少噪点对直线检测的干扰)
        blurred_img = cv2.GaussianBlur(corrected_img, (5, 5), 0)
        edges = cv2.Canny(blurred_img, 50, 150, apertureSize=3)
        
        # 步骤 4: 直线检测 (HoughLinesP)
        # minLineLength 越长，对"直线"的要求越高
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)
        
        if lines is None:
            return 1e6 # 惩罚 (没有找到直线)

        total_score = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # 我们只奖励接近水平或垂直的线
            # (简单的评分：只计算长度)
            total_score += length

        # 我们要最大化 total_score, 所以返回负值
        return -total_score

    except cv2.error as e:
        # OpenCV 可能会因为无效参数(如过度校正)而崩溃
        print(f"OpenCV 错误: {e}")
        return 1e6 # 惩罚无效参数

# --- 3. 运行优化器 ---

# 初始猜测 (来自您的 config 文件, 混合后的近似值)
# k1_mixed = (-0.2817 * 0.7) + (-0.3217 * 0.3) ≈ -0.293
# k2_mixed = (0.0593 * 0.7) + (0.1093 * 0.3) ≈ 0.074
# k3_mixed = (-0.0225 * 0.3) ≈ -0.0067
# alpha = 0.9
initial_guess = [-0.293, 0.074, -0.0067, 0.9]

# 参数边界 (k1, k2, k3, alpha)
# (限制 k 值在一个合理的范围内)
bounds = [
    (-0.5, 0.5),   # k1
    (-0.2, 0.2),   # k2
    (-0.05, 0.05), # k3
    (0.0, 1.0)     # alpha
]

print("\n--- 开始优化 (这可能需要几分钟) ---")
# 我们使用 'Nelder-Mead'，因为它对噪声/非平滑的损失函数具有鲁棒性
# 'adaptive=True' 使其对高维问题更稳健
result = minimize(
    objective_function, 
    initial_guess, 
    method='Nelder-Mead',
    # bounds=bounds, # Nelder-Mead 在标准 scipy 中不支持 bounds, 
    # 我们依赖 'adaptive' 和良好的初始猜测
    options={'maxiter': 500, 'adaptive': True, 'disp': True}
)

print("\n--- 优化完成 ---")

if result.success:
    optimized_params = result.x
    print("\n找到最佳参数:")
    print(f"  k1 = {optimized_params[0]}")
    print(f"  k2 = {optimized_params[1]}")
    print(f"  k3 = {optimized_params[2]}")
    print(f"  alpha = {optimized_params[3]}")
    
    print("\n建议的 config.py 内容:")
    print("-" * 30)
    print("# --- 自动优化的参数 (单模型) ---")
    print("# 禁用模型混合")
    print(f"DIST_COEFFS_BASE = np.array([{optimized_params[0]:.8f}, {optimized_params[1]:.8f}, 0.0, 0.0, {optimized_params[2]:.8f}])")
    print("WEIGHT_BASE = 1.0")
    print(f"DIST_COEFFS_K3_ADDON = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # (未使用)")
    print("WEIGHT_K3_ADDON = 0.0 # (未使用)")
    print(f"UNDISTORT_ALPHA = {optimized_params[3]:.8f}")
    print("-" * 30)
    
    # --- 可选：使用优化后的参数生成并显示最终图像 ---
    print("\n正在生成最终校正预览...")
    dist_coeffs_final = np.array([optimized_params[0], optimized_params[1], 0.0, 0.0, optimized_params[2]])
    alpha_final = optimized_params[3]
    
    new_cam_matrix_final, _ = cv2.getOptimalNewCameraMatrix(
        CAMERA_MATRIX, dist_coeffs_final, (IMG_WIDTH, IMG_HEIGHT), alpha_final, (IMG_WIDTH, IMG_HEIGHT)
    )
    map_x_final, map_y_final = cv2.initUndistortRectifyMap(
        CAMERA_MATRIX, dist_coeffs_final, None, new_cam_matrix_final, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1
    )
    final_corrected_img = cv2.remap(full_distorted_img, map_x_final, map_y_final, cv2.INTER_LINEAR)
    
    # 调整大小以便于显示
    h_orig, w_orig = final_corrected_img.shape[:2]
    preview_w = 960
    preview_h = int(h_orig * (preview_w / w_orig))
    preview_img = cv2.resize(final_corrected_img, (preview_w, preview_h))
    
    cv2.imshow('Optimized Corrected Image (Preview)', preview_img)
    print("按任意键退出预览。")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
else:
    print("\n!! 优化失败 !!")
    print(result.message)
