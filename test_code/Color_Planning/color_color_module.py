import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
import time
import tkinter as tk
from tkinter import Scale

# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("mtx.npy")
distortion_coeff = np.load("dist.npy")

MAX_CONTOUR_THRESHOLD = 1000

# ==============================カメラの設定==============================
camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(1)
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
lower_blue = np.array([90, 40, 26])
upper_blue = np.array([135, 250, 250])

lower_red = np.array([165, 15, 10])
upper_red = np.array([179, 250, 250])
# ==============================Tkinter GUIの設定==============================
def update_values(lower_orange, upper_orange):
    lower_orange = np.array([hue_lower.get(), sat_lower.get(), val_lower.get()])
    upper_orange = np.array([hue_upper.get(), sat_upper.get(), val_upper.get()])

root = tk.Tk()
root.title("HSV Range Adjuster")

hue_lower = Scale(root, from_=0, to=179, label="Hue Lower", orient=tk.HORIZONTAL)
hue_lower.set(lower_blue[0])
hue_lower.pack()

sat_lower = Scale(root, from_=0, to=255, label="Saturation Lower", orient=tk.HORIZONTAL)
sat_lower.set(lower_blue[1])
sat_lower.pack()

val_lower = Scale(root, from_=0, to=255, label="Value Lower", orient=tk.HORIZONTAL)
val_lower.set(lower_blue[2])
val_lower.pack()

hue_upper = Scale(root, from_=0, to=179, label="Hue Upper", orient=tk.HORIZONTAL)
hue_upper.set(upper_blue[0])
hue_upper.pack()

sat_upper = Scale(root, from_=0, to=255, label="Saturation Upper", orient=tk.HORIZONTAL)
sat_upper.set(upper_blue[1])
sat_upper.pack()

val_upper = Scale(root, from_=0, to=255, label="Value Upper", orient=tk.HORIZONTAL)
val_upper.set(upper_blue[2])
val_upper.pack()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
def main_loop():
    # update_values(lower_blue,upper_blue)
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    
    self.cameraCount += 1
    self.frame = self.picam2.capture_array()
    self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
    cv2.imwrite(self.results_img_dir+f'/{self.cameraCount}.jpg',self.frame2)
    height = self.frame2.shape[0]
    width = self.frame2.shape[1]
    
    if self.frame2 is not None:
        # オレンジ色の検出
        lower_blue = np.array([90, 40, 26])
        upper_blue = np.array([135, 250, 250])
        lower_red = np.array([165, 15, 10])
        upper_red = np.array([179, 250, 250])

        hsv = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        # 形態学的処理（膨張と収縮）を追加
        kernel = np.ones((10,10), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        # 輪郭を抽出して最大の面積を算出し、線で囲む
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            max_contour = max(contours_blue, key=cv2.contourArea)
            hull = cv2.convexHull(max_contour)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            max_contour = max(contours_red, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > MAX_CONTOUR_THRESHOLD//10:  # 面積が1000より大きい場合のみ描画
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    try:
                        if cv2.pointPolygonTest(hull, (cX, cY), False) >= 0:
                            print("\033[33m~~~ INSIDE ~~~\033[0m")
                        else:
                            print("\033[33m~~~ OUTSIDE ~~~\033[0m")
                    except :
                        pass

        # 結果の表示
        # mask_blue = cv2.resize(mask_blue, None, fx=0.5, fy=0.5)
        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
        # cv2.imshow('masked', mask_blue)
        # cv2.imshow('ARmarker', frame)
    
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
