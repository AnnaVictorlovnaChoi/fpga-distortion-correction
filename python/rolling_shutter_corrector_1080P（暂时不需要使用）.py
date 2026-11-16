# rolling_shutter_corrector.py
# ------------------------------
# 主算法逻辑文件，实现基于简化光流法的卷帘效应校正。

import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前工作路径已设置为:", os.getcwd())

# 从配置文件导入所有硬件和算法参数
try:
    import camera_config as cfg
except ImportError:
    print("错误：找不到配置文件 'camera_config.py'。请确保它与主脚本在同一目录下。")
    exit()

def setup_output_directory(output_dir):
    """检查并创建输出文件夹"""
    if not os.path.exists(output_dir):
        print(f"创建输出文件夹: {output_dir}")
        os.makedirs(output_dir)
    return output_dir

def select_file(title="请选择一个视频文件"):
    """弹出文件选择对话框"""
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    filepath = filedialog.askopenfilename(title=title)
    return filepath

def calculate_global_horizontal_motion(prev_gray, curr_gray):
    """
    使用LK稀疏光流计算全局平均水平运动。
    这模拟了FPGA中对关键特征进行跟踪并汇总运动信息的逻辑。
    """
    # 1. 在前一帧中寻找好的特征点
    p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **cfg.FEATURE_PARAMS)
    
    if p0 is None:
        return 0.0 # 如果没有找到特征点，则认为没有运动

    # 2. 计算光流
    p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, p0, None, **cfg.LK_PARAMS)

    # 3. 筛选出成功跟踪的点
    if p1 is not None and st is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]
    else:
        return 0.0

    if len(good_new) < 5: # 如果跟踪到的点太少，则认为结果不可靠
        return 0.0

    # 4. 计算所有成功跟踪点的水平位移并求平均值
    horizontal_displacements = good_new[:, 0] - good_old[:, 0]
    avg_horizontal_motion = np.mean(horizontal_displacements)
    
    return avg_horizontal_motion

def correct_frame_rolling_shutter(frame, vx):
    """
    使用像素重映射(cv2.remap)来校正单帧图像的卷帘效应。
    这种方法高效且非常适合硬件实现。
    
    :param frame: 待校正的图像帧
    :param vx: 该帧的全局平均水平速度 (像素/帧)
    :return: 校正后的图像帧
    """
    height, width, _ = frame.shape

    # 创建基础的映射网格
    map_x = np.tile(np.arange(width, dtype=np.float32), (height, 1))
    map_y = np.tile(np.arange(height, dtype=np.float32).reshape(-1, 1), (1, width))

    # 计算每一行的水平偏移量
    # 偏移量与行号y成正比。第一行(y=0)偏移为0，最后一行(y=height-1)偏移最大。
    # shift(y) = vx * (y / height)
    # 我们需要引入 DELAY_FACTOR 来表示曝光延迟的强度
    row_shift = (vx * cfg.DELAY_FACTOR * np.arange(height) / height).reshape(-1, 1)
    
    # 将偏移量应用到水平映射网格上
    # 新坐标 = 旧坐标 - 偏移量
    map_x -= row_shift
    
    # 执行重映射
    # cv2.INTER_LINEAR 提供了速度和质量的良好平衡
    corrected_frame = cv2.remap(frame, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    
    return corrected_frame


def main():
    """主执行函数"""
    output_dir = setup_output_directory(cfg.OUTPUT_FOLDER)
    
    video_path = select_file()
    if not video_path:
        print("未选择文件，程序退出。")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"错误: 无法打开视频文件 '{video_path}'")
        return

    # 获取视频属性以创建输出文件
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    base_name = os.path.splitext(os.path.basename(video_path))[0]
    output_path = os.path.join(output_dir, f"{base_name}{cfg.OUTPUT_FILENAME_SUFFIX}.mp4")
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print("-" * 50)
    print(f"开始处理视频: {os.path.basename(video_path)}")
    print(f"分辨率: {width}x{height}, 帧率: {fps}, 总帧数: {frame_count}")
    print(f"输出路径: {output_path}")
    print("-" * 50)

    # 读取第一帧
    ret, prev_frame = cap.read()
    if not ret:
        print("错误：无法读取视频的第一帧。")
        cap.release()
        return
        
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    
    # 第一帧不进行校正，直接写入
    out.write(prev_frame)
    
    frame_num = 1
    start_time = time.time()

    while True:
        ret, curr_frame = cap.read()
        if not ret:
            break
        
        frame_num += 1
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        # 1. 计算当前帧相对于前一帧的全局水平运动
        vx = calculate_global_horizontal_motion(prev_gray, curr_gray)
        
        # 2. 根据运动速度校正当前帧
        # 注意：我们用 t-1 到 t 的运动来校正 t 帧，这是一个合理的近似
        corrected_frame = correct_frame_rolling_shutter(curr_frame, vx)
        
        # 3. 写入输出视频
        out.write(corrected_frame)
        
        # 4. 更新前一帧
        prev_gray = curr_gray.copy()
        
        # 打印进度
        progress = (frame_num / frame_count) * 100
        print(f"\r处理中... {progress:.2f}% (帧 {frame_num}/{frame_count})", end="")

    # 释放所有资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    end_time = time.time()
    print(f"\n\n处理完成！总耗时: {end_time - start_time:.2f} 秒。")

if __name__ == '__main__':
    main()
