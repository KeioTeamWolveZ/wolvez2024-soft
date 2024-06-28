import time
import RPi.GPIO as GPIO
import sys
import numpy as np
import bno055
import Adafruit_BMP.BMP085 as BMP085

GPIO.setmode(GPIO.BCM) #GPIOの設定
bno055 = bno055.BNO055()
bno055.setupBno()
sensor = BMP085.BMP085()

if bno055.begin() is not True:
    print("Error initializing device")
    exit()
    

class Judgement():
    def __init__(self):
        self.countAccDropLoop = 0
        self.countPressDropLoop = 0
        self.TIME_THRESHOLD = 3
        self.DROPPING_ACC_THRE = 0.0005
        self.DROPPING_PRESS_THRE = 100000
        self.DROPPING_ACC_COUNT_THRE = 30
        self.DROPPING_PRESS_COUNT_THRE = 30
        self.time_tf = False
        self.acc_tf = False
        self.press_tf = False
        
    def judge_arrival(self, t, ax, ay, az, press):
        """
        引数：time:ステート以降後の経過時間、加速度の値(できればベクトル)、気圧(or高度)の値
        戻り値：着陸判定（着地：True,未着陸：False）
        """
        # 時間の判定
        if time.time() - t > self.TIME_THRESHOLD:
            self.time_tf =True
        else:
            self.time_tf = False
        # 加速度の判定
        if (ax**2 + ay**2 + az**2) < self.DROPPING_ACC_THRE**2: #加速度が閾値以下で着地判定
            self.countAccDropLoop+=1            
            if self.countAccDropLoop > self.DROPPING_ACC_COUNT_THRE: #加速度判定の複数回連続成功が必要
                self.acc_tf = True
        else:
            self.countAccDropLoop = 0 #初期化の必要あり
            self.acc_tf = False

        # 気圧の判定
        if press > self.DROPPING_PRESS_THRE: #気圧が閾値以上で着地判定
            self.countPressDropLoop+=1            
            if self.countPressDropLoop > self.DROPPING_PRESS_COUNT_THRE: #気圧判定の複数回連続成功が必要
                self.press_tf = True
        else:
            self.countPressDropLoop = 0 #初期化の必要あり
            self.press_tf = False

        if self.time_tf and self.acc_tf and self.press_tf:
            return True
        else:
            return False

judgement = Judgement()
start = time.time()
try:
    while True:
        bno055.bnoread()
        bno055.ax=round(bno055.ax,3)
        bno055.ay=round(bno055.ay,3)
        bno055.az=round(bno055.az,3)
        t = time.time()-start
        print('ax:',bno055.ax)
        print('ay:',bno055.ay)
        print('az:',bno055.az)
        print('pressure:',sensor.read_pressure())
        print('arrival:',judgement.judge_arrival(t,bno055.ax,bno055.ay,bno055.az,sensor.read_pressure()))
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
    pass
