#Last Update 2024/07/02
#Author : Yuma Suzuki

import const
import numpy as np
import os
import pandas as pd

data = pd.read_csv('uketome_distance.csv') # ファイルの読み込み
distance_data = data['distance [cm]'].values # 飛距離のデータを抽出

# load the latest color threshold
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
        return np.array([158, 85, 70]), np.array([179, 250, 250])
lower_red, upper_red = load_values_from_file("orange_hsv_values.txt")

# load the latest pressure threshold
def load_pressure_from_file(filename):
    if os.path.exists(filename):
        with open(filename, "r") as file:
            lines = file.readlines()
            press = float(lines[0])
            return press
    else:
        # Default values if file doesn't exist
        return 100000.00
latest_pressure = load_pressure_from_file("pressure_threshold.txt")


## Pin Number
# Motor
const.RIGHT_MOTOR_IN1_PIN = 6
const.RIGHT_MOTOR_IN2_PIN = 5
const.RIGHT_MOTOR_VREF_PIN = 13

const.LEFT_MOTOR_IN1_PIN = 20
const.LEFT_MOTOR_IN2_PIN = 16
const.LEFT_MOTOR_VREF_PIN = 12

# MARKER
const.MARKER_LENGTH = 0.0165  # マーカーの1辺の長さ（メートル）

# Servo motor
const.SERVO_PIN = 23

# LED
const.RED_LED_PIN =  10
const.BLUE_LED_PIN = 9
const.GREEN_LED_PIN = 11

# Separation Pin
const.SEPARATION_PARA = 8
const.SEPARATION_MOD1 = 24
const.SEPARATION_MOD2 = 25


# Flight Pin
const.FLIGHTPIN_PIN = 4

## Variables
# Goal information
const.GPS_GOAL_LAT = 35.55500000  # 南緯は負の値で与える
const.GPS_GOAL_LON = 139.65600000 # 西経は負の値で与える
const.GOAL_DISTANCE_THRE = 0.0005 # [km] (50 [cm])
const.GOAL_COLOR_THRE = 15000000 # 色認識によって認識して，ゴール判定を行う面積（要調整）

# # Motor VREF
const.LANDING_MOTOR_VREF = 80
const.RELEASING_MOTOR_VREF = 50
const.RUNNING_MOTOR_VREF = 100
const.STUCK_MOTOR_VREF = 100

const.SURFACE_GAIN = 1.0

# # State Threshold
const.TIME_CONSTANT_1 = 240
const.TIME_CONSTANT_2 = 420
const.TIME_CONSTANT_3 = 600

const.PREPARING_GPS_COUNT_THRE= 30
const.PREPARING_TIME_THRE = 10

const.FLYING_FLIGHTPIN_COUNT_THRE = 10

const.DROPPING_TIME_THRE = 10 #60
const.DROPPING_ACC_COUNT_THRE = 20
const.DROPPING_ACC_THRE = 0.008 #加速度の値 0.005
const.DROPPING_PRESS_THRE = 99087 # 気圧センサのカウンタ latest_pressure + 5m
const.DROPPING_PRESS_COUNT_THRE = 20 # 気圧センサのカウンタ

const.PARA_ESCAPE_TIME_THRE = 10

const.LOWER_ORANGE = np.array([0, 220, 158])
const.UPPER_ORANGE = np.array([55, 255, 255])
const.LOWER_BLUE = np.array([90, 96, 90])
const.UPPER_BLUE = np.array([137, 225, 255])

const.LOWER_RED = lower_red #np.array([158, 85, 70])
const.UPPER_RED = upper_red #np.array([179, 250, 250])

const.LOWER_GOAL = np.array([0, 220, 158]) #must change
const.UPPER_GOAL = np.array([55, 255, 255])

const.MAX_CONTOUR_THRESHOLD = 100

const.CLOSING_THRE = 0.6
const.CLOSING_RANGE_THRE = 0.05
const.CLOSING_RANGE_THRE2 = 0.3
const.CLOSING_RANGE_THRE_2 = 0.1

const.SEPARATION_TIME_THRE = 5 #焼き切り時間
const.LANDING_MOTOR_TIME_THRE = 10 #分離シートから離れるためにモータを回転させる時間
# const.RELEASING_MOTOR_TIME_THRE = 0.7 #放出と放出の間にモータを回転させる時間
# const.TURNING_MOTOR_TIME_THRE = 1.5 #turning time after the end of second-releasing
# const.MODULE_SEPARATION_TIME_THRE = 30 # モジュール同士の接続の際の焼き切り時間

# const.ARM_CALIB_POSITION = 0

const.LOST_MARKER_THRE = 30
const.AVOID_COLOR_THRE = 20 #色認識されなかった合計回数の閾値

# const.CONNECTED_HEIGHT_THRE = 700 #アームを上げた場合に接続できていることを確認する時の色の高さの閾値
const.EARTH_RADIUS = 6378.137 # [km]

# # Stack
const.STUCK_ACC_THRE = 0.5
const.STUCK_COUNT_THRE = 7
const.MIRRER_COUNT_THRE = 10
const.VANISH_BY_STUCK_THRE = 240 # ステート6で長時間何も見えなかった場合

# 運動方程式の各パラメータ（posture_judgement）
const.tolerance = 0.15  # 落下許容エリアの半径
const.m = 0.005  # 物資ジュール質量
const.g = 9.81  # 重力加速度
const.k = 0.05  # 空気抵抗係数
const.V0 = 4.684517487133138  # 物資モジュールの初速度
const.theta = np.deg2rad(45)  # 放出角度（ラジアン）
const.U = np.array([0, 0, 0])  # 風速ベクトル
const.x0, const.y0, const.z0 = 0.0, -0.02, -0.03  # 物資モジュールの初期位置（カメラに対する投射機構先端の位置）

# 投射成功確率に用いるパラメータ（posture_judgement）
const.mu = np.mean(distance_data) # 飛距離の平均
const.std = np.std(distance_data) # 飛距離の標準偏差
const.prob_threshold = 0.6 # 投射成功確率の閾値
