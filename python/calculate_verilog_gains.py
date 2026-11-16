#
#  calculate_verilog_gains.py
#
#  功能：
#  1. 弹窗让用户选择一张“基准图像”（推荐使用灰卡或白纸）。
#  2. 使用“全局灰度世界”算法计算R和B通道的增益。
#  3. 将浮点增益转换为Q4.8定点数格式。
#  4. 在终端输出Verilog代码可以直接使用的 localparam 定义。
#  5. 用于color_correction的verilog算法编写
import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print("当前路径已设置为：", os.getcwd())

# --- 辅助函数 ---

def select_file(title, filetypes):
    """弹出文件选择对话框"""
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    filepath = filedialog.askopenfilename(title=title, filetypes=filetypes)
    return filepath

def calculate_gains_for_verilog(image, q_format_bits=8):
    """
    计算全局增益，并将其转换为Verilog定点数格式。
    
    :param image: 输入的BGR格式图像 (来自cv2.imread)
    :param q_format_bits: 用于定点数的小数位数 (例如 8 bits for Q4.8)
    :return: 成功则返回True，失败则返回False
    """
    print("\n--- 开始计算增益 ---")
    
    # 1. 转换为浮点数进行精确计算
    img_float = image.astype(np.float32)
    
    if np.sum(img_float) < 1e-6:
        print("错误: 图像全黑，无法计算增益。")
        return False

    # 2. 计算R, G, B三通道的全局平均值
    b_avg, g_avg, r_avg = np.mean(img_float, axis=(0, 1))
    
    print(f"检测到平均值 (BGR): B_avg={b_avg:.4f}, G_avg={g_avg:.4f}, R_avg={r_avg:.4f}")

    if r_avg < 1e-6 or b_avg < 1e-6 or g_avg < 1e-6:
        print("错误: 某个通道平均值过低，无法计算增益。")
        return False
        
    # 3. 计算浮点增益 (以G通道为基准)
    r_gain_float = g_avg / r_avg
    b_gain_float = g_avg / b_avg
    
    print(f"\n计算出的浮点增益:")
    print(f"  R_GAIN_FLOAT = {r_gain_float:.6f}")
    print(f"  B_GAIN_FLOAT = {b_gain_float:.6f}")
    
    # 4. 转换为Q4.8定点数 (4位整数, 8位小数, 共12位)
    # 1.0 (float) == 1 * (2^q_format_bits) == 256
    q_factor = 1 << q_format_bits
    
    r_gain_fixed = int(np.round(r_gain_float * q_factor))
    b_gain_fixed = int(np.round(b_gain_float * q_factor))
    
    # 确定需要的总位数 (例如 12 bits for Q4.8)
    total_bits = 4 + q_format_bits
    
    print(f"\n--- Verilog 定点数参数 (Q4.{q_format_bits} 格式, {total_bits}位) ---")
    print("请将以下 `localparam` 复制到您的 Verilog 顶层或校正模块中：")
    
    print("\n" + "="*50)
    print(f"// (浮点值: R_Gain={r_gain_float:.4f}, B_Gain={b_gain_float:.4f})")
    print(f"localparam GAIN_WIDTH = {total_bits};")
    print(f"localparam Q_SHIFT    = {q_format_bits};")
    print(f"localparam R_GAIN_FX  = {total_bits}'d{r_gain_fixed}; // (十六进制: {total_bits}'h{r_gain_fixed:X})")
    print(f"localparam B_GAIN_FX  = {total_bits}'d{b_gain_fixed}; // (十六进制: {total_bits}'h{b_gain_fixed:X})")
    print("="*50 + "\n")
    
    return True

# --- 主函数 ---
def main():
    print("\n--- Verilog 全局色彩校正增益计算工具 ---")
    print("请选择一张用于校准的“基准图像”。")
    print("推荐使用相机在标准光源下拍摄的【白纸】或【18%灰卡】。")
    
    image_path = select_file(
        title="请选择一张用于校准的基准图像 (灰卡或白纸)",
        filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.tif"), ("All files", "*.*")]
    )
    
    if not image_path:
        print("未选择文件，程序退出。")
        return

    print(f"正在读取图像: {image_path}")
    image = cv2.imread(image_path)
    
    if image is None:
        print("错误: 无法读取所选图片。")
        return
        
    # 执行计算
    calculate_gains_for_verilog(image, q_format_bits=8)

if __name__ == "__main__":
    main()

