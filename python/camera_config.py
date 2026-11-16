# camera_config.py
# ------------------
# 该文件用于存储相机的硬件参数和算法相关的配置信息。
# 主算法文件将从这里导入所有必要的常量。

import cv2

# --- 相机基本参数 ---
RESOLUTION = (1920, 1080)  # (宽度, 高度)
FRAME_RATE = 60  # 帧率 (fps)
MOTION_SPEED_MAX_CM_PER_SEC = 20  # 设计目标最大运动速度 (cm/s)

# --- 卷帘效应相关物理参数 ---
# 传感器单帧读出时间，即第一行和最后一行的曝光时间差
# 范围是 8ms 到 16ms
FRAME_READOUT_TIME_MS = 16.0  # 我们取最坏情况进行设计 (ms)

# 卷帘方向: 'vertical' (从上到下) 或 'horizontal' (从左到右)
# 根据描述“水平方向，逐行从上到下曝光方式”，这里指扫描线是水平的，扫描过程是垂直的。
ROLLING_SHUTTER_DIRECTION = 'vertical'

# --- 物理到像素的转换参数 ---
PIXEL_PITCH_UM = 4.0  # 像素尺寸 (μm)
FOCAL_LENGTH_MM = 6.0  # 焦距 (mm)
FOV_H = 130  # 水平视场角 (°)
FOV_V = 75   # 垂直视场角 (°)

# --- 算法核心参数 ---
# delay_factor 代表了单帧读出时间占总帧时间(1/fps)的比例。
# 这是决定卷帘效应强度的关键系数。
# delay_factor = FRAME_READOUT_TIME_MS / (1000 / FRAME_RATE)
DELAY_FACTOR = FRAME_READOUT_TIME_MS / (1000.0 / FRAME_RATE)

# --- 光流法相关配置 ---
# 用于Lucas-Kanade光流法的特征点检测参数
FEATURE_PARAMS = dict(
    maxCorners=100,         # 最多检测100个角点
    qualityLevel=0.3,       # 角点质量阈值
    minDistance=7,          # 角点之间最小距离
    blockSize=7             # 计算梯度的窗口大小
)

# Lucas-Kanade光流法参数
LK_PARAMS = dict(
    winSize=(15, 15),       # 跟踪窗口大小
    maxLevel=2,             # 图像金字塔层数
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# --- 输出配置 ---
OUTPUT_FOLDER = "output"
OUTPUT_FILENAME_SUFFIX = "_corrected_rs" # rs for rolling_shutter