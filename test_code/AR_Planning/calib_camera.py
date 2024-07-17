#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import matplotlib.pyplot as plt
import numpy as np

camera  = input("Which camera do you want to use? (laptop:1 or picamera:2): ")

if int(camera) == 1:
    cap = cv2.VideoCapture(1)
elif int(camera) == 2:
    from picamera2 import Picamera2
    from libcamera import controls
    picam2 = Picamera2()
    size = (1800, 2400)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})
    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# ==============================キャリブレーションの設定==============================
square_size = 1.8      # 正方形の1辺のサイズ[cm]
pattern_size = (7, 7)  # 交差ポイントの数

reference_img = 100 # 参照画像の枚数
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 ) #チェスボード（X,Y,Z）座標の指定 (Z=0)
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size
objpoints = []
imgpoints = []

while len(objpoints) < reference_img:
# 画像の取得
    if int(camera) == 1:
        ret, img = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    img = cv2.rotate(frame,cv2.ROTATE_90_CLOCKWISE)
    height = img.shape[0]
    width = img.shape[1]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # チェスボードのコーナーを検出
    ret, corner = cv2.findChessboardCorners(gray, pattern_size)
    # コーナーがあれば
    if ret == True:
        print("detected coner!")
        print(str(len(objpoints)+1) + "/" + str(reference_img))
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(gray, corner, (5,5), (-1,-1), term)
        imgpoints.append(corner.reshape(-1, 2))   #appendメソッド：リストの最後に因数のオブジェクトを追加
        objpoints.append(pattern_points)

    cv2.imshow('image', img)
    # 毎回判定するから 200 ms 待つ．遅延するのはココ
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

print("calculating camera parameter...")
# 内部パラメータを計算
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 計算結果を保存
if int(camera) == 1:
    np.save("mtx_laptop", mtx) # カメラ行列
    np.save("dist_laptop", dist.ravel()) # 歪みパラメータ
elif int(camera) == 2:
    np.save("mtx_new", mtx)
    np.save("dist_new", dist.ravel())

# 計算結果を表示
print("RMS = ", ret)
print("mtx = \n", mtx)
print("dist = ", dist.ravel())
