import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
from Ar_tools import Artools
# import motor_pico as motor 
# import RPi.GPIO as GPIO
import time

# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# マーカーサイズの設定
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）

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
    # picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})
    lens = 5.5

# ==================================motor setting==================================
# GPIO.setwarnings(False)
# motor1 = motor.motor(6,5,13)
# motor2 = motor.motor(20,16,12)

# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0 # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ==============================クラスのインスタンス化==============================
ar = Artools()

# ==============================オレンジ色検出のためのHSV値の設定==============================
lower_orange = np.array([0, 135, 75])
upper_orange = np.array([13, 255, 255])

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
while True:
    # picam2.set_controls({"AfMode":0,"LensPosition":lens})
    # カメラ画像の取得
    if int(camera) == 1:
        ret, frame = cap.read()
        
        camera_matrix = np.load("mtx_laptop.npy")
        distortion_coeff = np.load("dist_laptop.npy")
    elif int(camera) == 2:
        frame = picam2.capture_array()        
        camera_matrix = np.load("mtx.npy")
        distortion_coeff = np.load("dist.npy")
    height = frame.shape[0]
    width = frame.shape[1]
    
    # オレンジ色の検出
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    orange_detected = cv2.countNonZero(mask_orange) > 0
    
    if orange_detected:
        # オレンジ色の重心を計算
        moments = cv2.moments(mask_orange)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            # 重心に赤い点を描画
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            mask_orange = cv2.cvtColor(mask_orange,cv2.COLOR_GRAY2RGB)
            cv2.circle(mask_orange, (cx, cy), 5, (0, 0, 255), -1)
        print("cx, cy:",cx, cy)
        time.sleep(0.01)  # 避けるために一時停止
        if cx > width/2:
            # motor1.go(70)
            # motor2.go(0)
            print("right")
            time.sleep(0.5)
            # motor1.stop()
            # motor2.stop()

        if cx < width/2:
            # motor1.go(0)
            # motor2.go(70)
            print("left")
            time.sleep(0.5)
            # motor1.stop()
            # motor2.stop()
            
    else:
        # motor1.go(70)
        # motor2.go(70)
        time.sleep(0.5)
        # motor1.stop()
        # motor2.stop()
        print("---motor go---")

    # ====================================結果の表示===================================
    # 画像のリサイズを行う
    mask_orange = cv2.resize(mask_orange, None, fx=0.5, fy=0.5)
    cv2.imshow('masked', mask_orange)
    cv2.imshow('ARmarker', frame)
    key = cv2.waitKey(1)  # キー入力の受付
    if key == 27:  # ESCキーで終了
        break

# ==============================終了処理==============================
if int(camera) == 1:
    cap.release()
cv2.destroyAllWindows()
# GPIO.cleanup()
