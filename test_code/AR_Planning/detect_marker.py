# ARマーカーを認識するプログラム
import cv2
import numpy as np
import cv2.aruco as aruco
from picamera2 import Picamera2
from libcamera import controls
from datetime import datetime
from correction import Correct
from polar import polar_change
from AR_outlier import outlier

# カメラのキャプチャ
# cap = cv2.VideoCapture(1)
picam2 = Picamera2()

# ARマーカーの辞書の選択
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# マーカーサイズの設定
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("../../mtx.npy")
distortion_coeff = np.load("../../dist.npy")

# カメラを開く
# cap = cv2.VideoCapture(1)
size = (1800, 1000)
config = picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": size})

picam2.align_configuration(config)
picam2.configure(config)
picam2.start()
# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
picam2.set_controls({"AfMode":0,"LensPosition":4.5})

VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
prev = [np.array([0,0,0])]
TorF = True


while True:
    # カメラ画像の取得
    # ret, frame = cap.read()
    frame = picam2.capture_array()
    height = frame.shape[0]
    width = frame.shape[1]

    # print(height,width)
    # if not ret:
    #     break

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ARマーカーの検出
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary)
    

    # 検出したマーカーのI検出したマーカーから垂直に30センチメートルの距離にある点を描画
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
                # 並進ベクトルの転置
                transpose_tvec = tvec[np.newaxis, :].T
                # 合成
                proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                # オイラー角への変換
                euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

                if ultra_count == 0:
                    prev.append(tvec)
                    ultra_count = 1
                else:
                    TorF = outlier(tvec, prev) # true:correct,false:outlier
                    if TorF:
                        print("x : " + str(tvec[0]))
                        print("y : " + str(tvec[1]))
                        print("z : " + str(tvec[2]))
                        # print("roll : " + str(euler_angle[0]))
                        # print("pitch: " + str(euler_angle[1]))
                        # print("yaw  : " + str(euler_angle[2]))


                # 発見したマーカーから1辺が30センチメートルの正方形を描画
                # aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec, tvec, 0.1)
                # aruco.drawDetectedMarkers(frame, corners, ids, (0, 0, 255))
                # aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec, tvec, 0.1)
                # aruco.drawDetectedCornersCharuco(frame, corners, ids, (0, 0, 255))
                
                    
                point_3d = np.array([[tvec[0], tvec[1], tvec[2]]], dtype=np.float64)
                imgpts, jac = cv2.projectPoints(point_3d,rvec, tvec, camera_matrix, distortion_coeff)
                cv2.line(frame, (width//2,0), (width//2,height),(255,255,0))
                # print(imgpts)
                # print("ID:",ids[i])
                distance, angle = Correct(tvec,VEC_GOAL)
                # print("kabuto_function:",distance,angle)
                polar_exchange = polar_change(tvec)
                # print("yunosu_function:",polar_exchange)


    # 結果の表示
    cv2.imshow('ARmarker', frame)
    # キー入力の受付
    key = cv2.waitKey(1)
    if key == 27:  # ESCキーで終了
        break

# 終了処理
cap.release()
cv2.destroyAllWindows()
