# ARマーカーを認識するプログラム
import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
from Ar_tools import Artools
import motor
import RPi.GPIO as GPIO
import time



# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# マーカーサイズの設定
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("mtx.npy")
distortion_coeff = np.load("dist.npy")

# ==============================カメラの設定==============================

camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(1)
elif int(camera) == 2:
    from picamera2 import Picamera2 #laptopでは使わないため
    from libcamera import controls #laptopでは使わないため
    picam2 = Picamera2()
    size = (1800, 1000)
    config = picam2.create_still_configuration(
                main={"format": 'XRGB8888', "size": size})

    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    # picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})
    lens = 5.5
# ==================================motor setting==================================
GPIO.setwarnings(False)
motor1 = motor.motor(6,5,13)
motor2 = motor.motor(20,16,12)
# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0 # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ====================================成功の定義====================================
closing_threshold = 0.4
closing_range = 0.02
# ==============================クラスのインスタンス化==============================
ar = Artools()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
while True:
    picam2.set_controls({"AfMode":0,"LensPosition":lens})
    # カメラ画像の取得
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    height = frame.shape[0]
    width = frame.shape[1]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary) # ARマーカーの検出    

    if ids is not None:
        # aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            if ids[i] in [0,1,2,3,4,5]:
                image_points_2d = np.array(corners[i],dtype='double')
                # print(corners[i])

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, distortion_coeff)
                tvec = np.squeeze(tvec)
                rvec = np.squeeze(rvec)
                # 回転ベクトルからrodoriguesへ変換
                rvec_matrix = cv2.Rodrigues(rvec)
                rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                transpose_tvec = tvec[np.newaxis, :].T # 並進ベクトルの転置
                proj_matrix = np.hstack((rvec_matrix, transpose_tvec)) # 合成
                euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # オイラー角への変換[deg]
                prev = list(prev)

                if ultra_count < 20:
                    prev.append(tvec)
                    print("ARマーカーの位置を算出中")
                    ultra_count += 1 #最初（位置リセット後も）は20回取得して平均取得
                else:
                    # print("prev_length: ",len(prev))
                    TorF = ar.outlier(tvec, prev, ultra_count, 0.3) # true:correct, false:outlier
                    ultra_count += 1
                    if TorF: # detected AR marker is reliable
                        reject_count = 0
                        print("x : " + str(tvec[0]))
                        print("y : " + str(tvec[1]))
                        print("z : " + str(tvec[2]))
                        # print("roll : " + str(euler_angle[0]))
                        # print("pitch: " + str(euler_angle[1]))
                        # print("yaw  : " + str(euler_angle[2]))
                        tvec[0] = tvec[0]
                        polar_exchange = ar.polar_change(tvec)
                        print(f"yunosu_function_{ids[i]}:",polar_exchange)
                        
                        distance_of_marker = polar_exchange[0]
                        angle_of_marker = polar_exchange[1]
                        if distance_of_marker >= closing_threshold:
                            if tvec[0] >= 0.05:
                                motor1.go(70)
                                motor2.go(45)
                                time.sleep(0.05)
                                print(f"---motor LEFT {angle_of_marker}---")
                                motor1.stop()
                                motor2.stop()
                            elif 0.05 > tvec[0] > -0.05:
                                go_ahead_gain = (distance_of_marker-closing_threshold) / closing_threshold
                                motor1.go(40+60*go_ahead_gain)
                                motor2.go(40+60*go_ahead_gain)
                                time.sleep(0.05)
                                print("---motor GO AHEAD---")
                                motor1.stop()
                                motor2.stop()
                            else:
                                motor1.go(45)
                                motor2.go(70)
                                time.sleep(0.05)
                                print("---motor RIGHT---")
                                motor1.stop()
                                motor2.stop()
                        elif distance_of_marker >= closing_threshold - closing_range:
                            if tvec[0] >= 0.03:
                                print("---turn RIGHT---")
                                motor1.go(45)
                                motor2.back(45)
                                time.sleep(0.03)
                                motor1.stop()
                                motor2.stop()
                            elif tvec[0] <= -0.03:
                                print("---turn LEFT---")
                                motor1.back(45)
                                motor2.go(45)
                                time.sleep(0.04)
                                motor1.stop()
                                motor2.stop()
                                
                            else:
                                print("'\033[32m'---perfect REACHED---'\033[0m'")

                        else:
                            motor1.back(60)
                            motor2.back(60)
                            time.sleep(0.03)
                            motor1.stop()
                            motor2.stop()
                            print("need to go back!!")
                        
                    else: # detected AR marker is not reliable
                        print("state of marker is rejected")
                        print(ultra_count)
                        reject_count += 1 # 拒否された回数をカウント
                        if reject_count > 10: # 拒否され続けたらリセットしてARマーカーの基準を上書き（再計算）
                            ultra_count = 0
                            reject_count = 0 #あってもなくても良い
                    

                # 発見したマーカーから1辺が30センチメートルの正方形を描画
                color = (0,255,0)
                line = np.int32(np.squeeze(corners[i]))
                cv2.polylines(frame,[line],True,color,2)
                    
                cv2.line(frame, (width//2,0), (width//2,height),(255,255,0))
                distance, angle = ar.Correct(tvec,VEC_GOAL)
                polar_exchange = ar.polar_change(tvec)
                # print("kabuto_function:",distance,angle)
                # print("yunosu_function:",polar_exchange)

                # カメラレンズの回帰式
                change_lens = -17.2*polar_exchange[0]+9.84
                if change_lens < 3:
                    lens = 3
                elif change_lens > 10:
                    lens = 10.5
                else:
                    lens = change_lens
                



    # ====================================結果の表示===================================
    # #　画像のリサイズを行う
    frame = cv2.resize(frame,None,fx=0.5,fy=0.5)
#    cv2.imshow('ARmarker', frame)
#    key = cv2.waitKey(1)# キー入力の受付
#    if key == 27:  # ESCキーで終了
#        break

# ==============================終了処理==============================
cap.release()
# cv2.destroyAllWindows()
# GPIO.cleanup()
