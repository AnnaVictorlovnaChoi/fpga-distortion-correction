# distortion_corrector_1080p.py
# -------------------------------------------------
# 1080P 畸变校正主逻辑文件
#
# !! 改进版：支持两种畸变模型的加权混合 !!
# 算法基于 "混合查找表" (Blended LUT)，
# 保持了对 FPGA 硬件实现的友好性。
# -------------------------------------------------
import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog
from tqdm import tqdm

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 关键：导入我们新的 1080P 配置文件 ---
try:
    import config_dist_1080p_k3 as cfg
except ImportError:
    print("错误：找不到配置文件 'config_dist_1080p.py'。")
    print("请确保 'config_dist_1080p.py' 与本脚本在同一目录下。")
    exit()

# 确保脚本在当前文件所在目录运行，便于管理 output 文件夹
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前工作路径已设置为:", os.getcwd())


def select_video_file():
    """
    弹出一个窗口让用户选择视频文件。
    Returns:
        str: 用户选择的视频文件路径，如果用户取消则返回None。
    """
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    file_path = filedialog.askopenfilename(
        title="请选择需要进行 1080P 畸变校正的视频文件",
        filetypes=[("Video files", "*.mp4 *.avi *.mov *.mkv"), ("All files", "*.*")]
    )
    return file_path


def correct_distortion_1080p(video_path):
    """
    对输入的 1080P 视频进行畸变校正处理，并保存结果。
    Args:
        video_path (str): 输入视频的文件路径。
    """
    # 1. 初始化视频读取对象
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"错误：无法打开视频文件 {video_path}")
        return

    original_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if original_fps == 0 or original_fps is None:
        print(f"警告: 无法读取视频原始帧率，将使用配置中的 {cfg.FPS} fps。")
        output_fps = cfg.FPS
    else:
        output_fps = original_fps

    print(f"视频总帧数: {total_frames}, 原始帧率: {original_fps:.2f} fps")

    # --- 核心步骤 1: 离线预计算 (混合查找表 Blended LUT 生成) ---
    print("正在根据 1080P 相机参数预计算畸变校正查找表 (LUT)...")

    # 1.1. 计算一个统一的“新相机矩阵”和“ROI”
    # (两个模型都必须映射到这个相同的目标)
    print("正在计算最佳输出矩阵与 ROI...")
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        cfg.CAMERA_MATRIX,
        cfg.DIST_COEFFS_BASE,  # 使用基础模型来确定裁剪区域
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT),
        cfg.UNDISTORT_ALPHA,
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT)
    )

    # 1.2. 生成 LUT A (基础模型, k1, k2)
    print(f"正在生成 '基础模型' LUT (权重 {cfg.WEIGHT_BASE*100}%)")
    map_A_x, map_A_y = cv2.initUndistortRectifyMap(
        cfg.CAMERA_MATRIX,
        cfg.DIST_COEFFS_BASE,
        None,
        new_camera_matrix,
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT),
        cv2.CV_32FC1
    )

    # 1.3. 生成 LUT B (k3模型, k1, k2, k3)
    print(f"正在生成 'k3模型' LUT (权重 {cfg.WEIGHT_K3_ADDON*100}%, k3={cfg.K3_GUESS})")
    map_B_x, map_B_y = cv2.initUndistortRectifyMap(
        cfg.CAMERA_MATRIX,
        cfg.DIST_COEFFS_K3_ADDON,
        None,
        new_camera_matrix,
        (cfg.IMG_WIDTH, cfg.IMG_HEIGHT),
        cv2.CV_32FC1
    )

    # 1.4. 加权混合两个 LUT
    print("正在混合两个 LUT...")
    map_x = (cfg.WEIGHT_BASE * map_A_x) + (cfg.WEIGHT_K3_ADDON * map_B_x)
    map_y = (cfg.WEIGHT_BASE * map_A_y) + (cfg.WEIGHT_K3_ADDON * map_B_y)
    print("1920x1080 混合查找表 (Blended LUT) 生成完毕！")

    # 2. 初始化视频写入对象
    output_dir = 'output'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    base_name = os.path.basename(video_path)
    name, ext = os.path.splitext(base_name)
    output_path = os.path.join(output_dir, f"{name}_distortion_k3.mp4")

    x, y, w, h = roi
    print(f"校正后将自动裁剪画面至 ROI: x={x}, y={y}, w={w}, h={h}")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用 .mp4 编码
    out = cv2.VideoWriter(output_path, fourcc, output_fps, (w, h))

    # --- 核心步骤 2: 在线实时处理 (使用混合LUT进行Remap) ---
    print(f"开始处理视频: {base_name}")
    progress_bar = tqdm(total=total_frames, desc="畸变校正进度", unit="帧")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame.shape[1] != cfg.IMG_WIDTH or frame.shape[0] != cfg.IMG_HEIGHT:
            frame = cv2.resize(frame, (cfg.IMG_WIDTH, cfg.IMG_HEIGHT), interpolation=cv2.INTER_LINEAR)

        # 应用【最终的混合LUT】 (实时查表 + 双线性插值)
        corrected_frame = cv2.remap(
            frame,
            map_x,  # <-- 使用混合后的 map_x
            map_y,  # <-- 使用混合后的 map_y
            interpolation=cv2.INTER_LINEAR
        )

        # 根据ROI裁剪图像，去除黑边
        cropped_frame = corrected_frame[y:y+h, x:x+w]

        # 写入输出文件
        out.write(cropped_frame)

        progress_bar.update(1)

    progress_bar.close()

    # 3. 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"\n处理完成！校正后的视频已保存至: {output_path}")


if __name__ == '__main__':
    print("="*60)
    print(" 1080P 畸变校正程序 (FPGA-Blended-LUT 模拟) ".center(60, "="))
    print("="*60)
    video_file = select_video_file()
    if video_file:
        correct_distortion_1080p(video_file)
    else:
        print("未选择文件，程序退出。")