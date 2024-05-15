# ARマーカーを認識するプログラム
import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
from Ar_tools import Artools


# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# マーカーサイズの設定
marker_length = 0.02  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("../../mtx.npy")
distortion_coeff = np.load("../../dist.npy")

# ==============================カメラの設定==============================

camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(1)
elif int(camera) == 2:
    from picamera2 import Picamera2 #laptopでは使わないため
    from libcamera import controls #laptopでは使わないため
    picam2 = Picamera2()
    size = (1800, 1200)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})

    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    # picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})

# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0 # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ==============================クラスのインスタンス化==============================
ar = Artools()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
while True:
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
                    TorF = ar.outlier(tvec, prev, 0.2) # true:correct, false:outlier
                    if TorF:
                        reject_count = 0
                        print("x : " + str(tvec[0]))
                        print("y : " + str(tvec[1]))
                        print("z : " + str(tvec[2]))
                        # print("roll : " + str(euler_angle[0]))
                        # print("pitch: " + str(euler_angle[1]))
                        # print("yaw  : " + str(euler_angle[2]))
                        polar_exchange = ar.polar_change(tvec)
                        print(f"yunosu_function_{ids[i]}:",polar_exchange)
                    else:
                        print("state of marker is rejected")
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
                



    # ====================================結果の表示===================================
    # #　画像のリサイズを行う
    # frame = cv2.resize(frame,None,fx=0.7,fy=0.7)
    cv2.imshow('ARmarker', frame)
    key = cv2.waitKey(1)# キー入力の受付
    if key == 27:  # ESCキーで終了
        break

# ==============================終了処理==============================
cap.release()
cv2.destroyAllWindows()
