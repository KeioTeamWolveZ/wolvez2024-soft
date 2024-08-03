import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
import time
import tkinter as tk
from tkinter import Scale
import os

# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("mtx.npy")
distortion_coeff = np.load("dist.npy")

MAX_CONTOUR_THRESHOLD = 1000

# ==============================カメラの設定==============================
camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(0)
elif int(camera) == 2:
    from picamera2 import Picamera2 #laptopでは使わないため
    from libcamera import controls #laptopでは使わないため
    picam2 = Picamera2()
    size = (1800, 1000)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})

    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})
    lens = 5.5

# ==================================motor setting==================================
# GPIO.setwarnings(False)
# motor1 = motor.motor(6,5,13)
# motor2 = motor.motor(20,16,12)

# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0  # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ==============================クラスのインスタンス化==============================
# ar = Artools()

# ==============================オレンジ色検出のためのHSV値の設定==============================
def load_values_from_file(filename):
    """Load HSV values from a text file."""
    if os.path.exists(filename):
        with open(filename, "r") as file:
            lines = file.readlines()
            lower_orange = np.array(eval(lines[0].split(":")[1].strip()))
            upper_orange = np.array(eval(lines[1].split(":")[1].strip()))
            return lower_orange, upper_orange
    else:
        # Default values if file doesn't exist
        return np.array([105, 56, 0]), np.array([150, 250, 250])

def save_values_to_file(filename):
    """Save HSV values to a text file."""
    with open(filename, "w") as file:
        file.write(f"lower_orange: {lower_orange.tolist()}\n")
        file.write(f"upper_orange: {upper_orange.tolist()}\n")

# Load initial HSV values from file or use default values
lower_orange, upper_orange = load_values_from_file("orange_hsv_values.txt")

# ==============================Tkinter GUIの設定==============================
def update_values():
    global lower_orange, upper_orange
    lower_orange = np.array([hue_lower.get(), sat_lower.get(), val_lower.get()])
    upper_orange = np.array([hue_upper.get(), sat_upper.get(), val_upper.get()])
    save_values_to_file("orange_hsv_values.txt")

root = tk.Tk()
root.title("HSV Range Adjuster")

hue_lower = Scale(root, from_=0, to=179, label="Hue Lower", orient=tk.HORIZONTAL)
hue_lower.set(lower_orange[0])
hue_lower.pack()

sat_lower = Scale(root, from_=0, to=255, label="Saturation Lower", orient=tk.HORIZONTAL)
sat_lower.set(lower_orange[1])
sat_lower.pack()

val_lower = Scale(root, from_=0, to=255, label="Value Lower", orient=tk.HORIZONTAL)
val_lower.set(lower_orange[2])
val_lower.pack()

hue_upper = Scale(root, from_=0, to=179, label="Hue Upper", orient=tk.HORIZONTAL)
hue_upper.set(upper_orange[0])
hue_upper.pack()

sat_upper = Scale(root, from_=0, to=255, label="Saturation Upper", orient=tk.HORIZONTAL)
sat_upper.set(upper_orange[1])
sat_upper.pack()

val_upper = Scale(root, from_=0, to=255, label="Value Upper", orient=tk.HORIZONTAL)
val_upper.set(upper_orange[2])
val_upper.pack()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
def main_loop():
    update_values()
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    
    if frame is not None:
        # オレンジ色の検出
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # 形態学的処理（膨張と収縮）を追加
        kernel = np.ones((8,8), np.uint8)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)

        orange_detected = cv2.countNonZero(mask_orange) > 0

        # 輪郭を抽出して最大の面積を算出し、線で囲む
        contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > MAX_CONTOUR_THRESHOLD:  # 面積が1000より大きい場合のみ描画
                cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 3)
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
                    # cv2.putText(frame, "centroid", (cX - 25, cY - 25),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                print(f"Centroid: ({cX}, {cY})")

        # オレンジ色が検出された場合、適切な処理を実行
        if orange_detected:
            print("Orange detected! Avoiding... :")
            time.sleep(0.01)  # 避けるために一時停止
        else:
            print("---motor go---")
            time.sleep(0.01)

        # 結果の表示
        mask_orange = cv2.resize(mask_orange, None, fx=0.5, fy=0.5)
        frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
        cv2.imshow('masked', mask_orange)
        cv2.imshow('ARmarker', frame)
    
    key = cv2.waitKey(1)  # キー入力の受付
    if key == 27:  # ESCキーで終了
        if int(camera) == 1:
            cap.release()
        cv2.destroyAllWindows()
        root.destroy()
        return
    
    root.after(10, main_loop)

root.after(10, main_loop)
root.mainloop()
