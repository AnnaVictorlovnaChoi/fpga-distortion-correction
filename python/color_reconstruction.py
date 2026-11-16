import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 核心色彩校正函数 ---

def correct_color_global(image):
    """全局灰度世界算法"""
    img_float = image.astype(np.float32)
    if np.sum(img_float) < 1e-6: return image # 避免处理全黑图像
    b_avg, g_avg, r_avg = np.mean(img_float, axis=(0, 1))
    if r_avg < 1e-6 or b_avg < 1e-6 or g_avg < 1e-6: return image
    r_gain = g_avg / r_avg
    b_gain = g_avg / b_avg
    img_float[:, :, 2] *= r_gain
    img_float[:, :, 0] *= b_gain
    corrected_image = np.clip(img_float, 0, 255)
    return corrected_image.astype(np.uint8)

def correct_color_local_blocky(image, block_size=128):
    """朴素分块灰度世界算法 (会导致块状伪影)"""
    h, w, _ = image.shape
    corrected_image = image.copy().astype(np.float32)
    for r in range(0, h, block_size):
        for c in range(0, w, block_size):
            r_end, c_end = min(r + block_size, h), min(c + block_size, w)
            block = image[r:r_end, c:c_end].astype(np.float32)
            if block.size == 0: continue
            b_avg, g_avg, r_avg = np.mean(block, axis=(0, 1))
            if r_avg < 1e-6 or b_avg < 1e-6 or g_avg < 1e-6: continue
            r_gain = g_avg / r_avg
            b_gain = g_avg / b_avg
            corrected_image[r:r_end, c:c_end, 2] *= r_gain
            corrected_image[r:r_end, c:c_end, 0] *= b_gain
    corrected_image = np.clip(corrected_image, 0, 255)
    return corrected_image.astype(np.uint8)

def correct_color_local_smooth(image, block_size=128):
    """【推荐】带双线性插值的平滑分块灰度世界算法"""
    h, w, _ = image.shape
    if h == 0 or w == 0: return image
    img_float = image.astype(np.float32)
    grid_h, grid_w = int(np.ceil(h / block_size)), int(np.ceil(w / block_size))
    gain_grid_r = np.ones((grid_h, grid_w), dtype=np.float32)
    gain_grid_b = np.ones((grid_h, grid_w), dtype=np.float32)
    for i in range(grid_h):
        for j in range(grid_w):
            r_start, c_start = i * block_size, j * block_size
            r_end, c_end = min(r_start + block_size, h), min(c_start + block_size, w)
            block = img_float[r_start:r_end, c_start:c_end]
            if block.size == 0: continue
            b_avg, g_avg, r_avg = np.mean(block, axis=(0, 1))
            if r_avg < 1e-6 or b_avg < 1e-6 or g_avg < 1e-6: continue
            gain_grid_r[i, j] = g_avg / r_avg
            gain_grid_b[i, j] = g_avg / b_avg
    r_gain_map = cv2.resize(gain_grid_r, (w, h), interpolation=cv2.INTER_LINEAR)
    b_gain_map = cv2.resize(gain_grid_b, (w, h), interpolation=cv2.INTER_LINEAR)
    corrected_image_float = img_float.copy()
    corrected_image_float[:, :, 2] *= r_gain_map
    corrected_image_float[:, :, 0] *= b_gain_map
    corrected_image = np.clip(corrected_image_float, 0, 255)
    return corrected_image.astype(np.uint8)

# --- 新增的辅助函数和处理逻辑 ---

def setup_output_directory(output_dir="output"):
    """检查并创建输出文件夹"""
    if not os.path.exists(output_dir):
        print(f"创建输出文件夹: {output_dir}")
        os.makedirs(output_dir)
    return output_dir

def select_file(title, filetypes):
    """弹出文件选择对话框"""
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    filepath = filedialog.askopenfilename(title=title, filetypes=filetypes)
    return filepath

def process_and_save_image(image, base_name, output_dir):
    """对单张图片进行三种处理并保存"""
    print(f"正在处理图片: {base_name}...")
    
    # 执行三种校正
    corrected_global = correct_color_global(image)
    corrected_blocky = correct_color_local_blocky(image)
    corrected_smooth = correct_color_local_smooth(image)
    
    # 保存结果
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_original.png"), image)
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_corrected_global.png"), corrected_global)
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_corrected_blocky.png"), corrected_blocky)
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_corrected_smooth.png"), corrected_smooth)
    
    print(f"图片处理完成，结果已保存至 '{output_dir}' 文件夹。")

def process_video(filepath, output_dir):
    """对视频文件进行三种处理并保存为新视频"""
    base_name = os.path.splitext(os.path.basename(filepath))[0]
    
    cap = cv2.VideoCapture(filepath)
    if not cap.isOpened():
        print("错误: 无法打开视频文件。")
        return

    # 获取视频属性
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 使用 mp4v 编码器，生成 .mp4 文件

    # 创建三个视频写入器
    out_global = cv2.VideoWriter(os.path.join(output_dir, f"{base_name}_corrected_global.mp4"), fourcc, fps, (width, height))
    out_blocky = cv2.VideoWriter(os.path.join(output_dir, f"{base_name}_corrected_blocky.mp4"), fourcc, fps, (width, height))
    out_smooth = cv2.VideoWriter(os.path.join(output_dir, f"{base_name}_corrected_smooth.mp4"), fourcc, fps, (width, height))

    print(f"开始处理视频: {base_name}.mp4 (共 {frame_count} 帧)")
    current_frame = 0
    start_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        current_frame += 1
        
        # 应用校正
        corrected_global = correct_color_global(frame)
        corrected_blocky = correct_color_local_blocky(frame)
        corrected_smooth = correct_color_local_smooth(frame)
        
        # 写入新视频
        out_global.write(corrected_global)
        out_blocky.write(corrected_blocky)
        out_smooth.write(corrected_smooth)
        
        # 打印进度
        progress = (current_frame / frame_count) * 100
        print(f"\r处理中... {progress:.2f}% ({current_frame}/{frame_count})", end="")

    # 释放资源
    cap.release()
    out_global.release()
    out_blocky.release()
    out_smooth.release()
    
    end_time = time.time()
    print(f"\n视频处理完成！耗时: {end_time - start_time:.2f} 秒。")
    print(f"结果已保存至 '{output_dir}' 文件夹。")

def generate_and_process_simulated_image(output_dir):
    """生成一张模拟的偏绿图像并处理"""
    print("生成模拟偏绿图像...")
    simulated_image = np.ones((1080, 1920, 3), dtype=np.float32)
    # BGR values: Blue=100, Green=150, Red=110 -> 明显的绿色调
    simulated_image[:, :] = [100, 150, 110] 
    # 添加一些其他颜色区域
    cv2.rectangle(simulated_image, (100, 100), (800, 500), (180, 200, 160), -1)
    cv2.rectangle(simulated_image, (1000, 600), (1800, 1000), (80, 90, 150), -1)
    greenish_image = np.clip(simulated_image, 0, 255).astype(np.uint8)
    
    process_and_save_image(greenish_image, "simulated_image", output_dir)


# --- 主函数 ---
def main():
    output_dir = setup_output_directory()

    print("\n--- 色彩校正工具 ---")
    print("请选择处理模式:")
    print("  1. 生成并处理一张模拟的偏绿图片")
    print("  2. 选择并处理一张本地图片")
    print("  3. 选择并处理一个本地视频")
    
    choice = input("请输入选项 (1, 2, or 3): ")

    if choice == '1':
        generate_and_process_simulated_image(output_dir)
    
    elif choice == '2':
        image_path = select_file(
            title="请选择一张图片",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.tif"), ("All files", "*.*")]
        )
        if image_path:
            image = cv2.imread(image_path)
            if image is not None:
                base_name = os.path.splitext(os.path.basename(image_path))[0]
                process_and_save_image(image, base_name, output_dir)
            else:
                print("错误: 无法读取所选图片。")
        else:
            print("未选择文件，程序退出。")

    elif choice == '3':
        video_path = select_file(
            title="请选择一个视频",
            filetypes=[("Video files", "*.mp4 *.avi *.mov *.mkv"), ("All files", "*.*")]
        )
        if video_path:
            process_video(video_path, output_dir)
        else:
            print("未选择文件，程序退出。")

    else:
        print("无效的选项，程序退出。")

if __name__ == "__main__":
    main()

