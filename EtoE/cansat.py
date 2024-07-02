import pigpio
import RPi.GPIO as GPIO

import sys
import cv2
import time
import numpy as np
import os
import re
import math
from datetime import datetime
from glob import glob
from picamera2 import Picamera2 
from libcamera import controls 

import constant as ct
from Wolvez2024_now.led import led
from Wolvez2024_now.gps import GPS
from Wolvez2024_now.bno055 import BNO055
from Wolvez2024_now.bmp import BMP
from Wolvez2024_now.motor_pico import motor_pico as motor


"""
ステート説明
0. preparing()
1. flying()
3. landing()
4. para_escaping()
5. first_releasing()
6. moving_release_position()
7. judgement()
8. finish()

"""
class Cansat():
	def __init__(self,state):
		
		# ================================================ GPIO ================================================ 
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #GPIOの設定
		GPIO.setup(ct.const.FLIGHTPIN_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP) #フライトピン用。プルアップを有効化

		
		# ============================================== constant ============================================== 
		
	        self.TIME_THRESHOLD = 3 # ct.const.
	        self.DROPPING_ACC_THRE = 0.0005 # ct.const.
	        self.DROPPING_PRESS_THRE = 100000 # ct.const.
	        self.DROPPING_ACC_COUNT_THRE = 30 # ct.const.
	        self.DROPPING_PRESS_COUNT_THRE = 30 # ct.const.
		
		# =============================================== モータ =============================================== 
		GPIO.setwarnings(False)
	        self.MotorL = motor.motor(ct.const.RIGHT_MOTOR_IN1_PIN,ct.const.RIGHT_MOTOR_IN2_PIN,ct.const.RIGHT_MOTOR_VREF_PIN)
	        self.MotorR = motor.motor(ct.const.LEFT_MOTOR_IN1_PIN,ct.const.LEFT_MOTOR_IN2_PIN, ct.const.LEFT_MOTOR_VREF_PIN)
		
		# =============================================== カメラ =============================================== 
		picam2 = Picamera2()
	    	size = (1100, 1800)
	    	config = picam2.create_preview_configuration(
			main={"format": 'XRGB8888', "size": size})
	    	picam2.align_configuration(config)
	    	picam2.configure(config)
	    	picam2.start()
	    	# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
	    	picam2.set_controls({"AfMode":0,"LensPosition":5.5})
		
		# ================================================= LED ================================================= 
		self.RED_LED = led(ct.const.RED_LED_PIN) # 
		self.BLUE_LED = led(ct.const.BLUE_LED_PIN) # 
		self.GREEN_LED = led(ct.const.GREEN_LED_PIN) # 

		# =============================================== センサ =============================================== 
		self.gps = GPS() #
		self.bno055 = BNO055() #
		self.bmp = BMP() #
		
		# ============================================ ステート管理 ============================================ 
		self.timer = 0 # 
		self.state = state # 
		self.last_state = 0 #
		self.time = 0 #
		self.startTime_time=time.time() #
		self.startTime = str(datetime.now())[:19].replace(" ","_").replace(":","-") #

		

		
		# =============================================== 時間記録 =============================================== 
		self.preparingTime = 0 #
		self.flyingTime = 0 #
		self.landtime = 0
		
		# =============================================== カウンタ =============================================== 
		# センサ用
		self.gpscount = 0 #
		self.countFlyLoop = 0 #
		self.startgps_lon = [] #
		self.startgps_lat = [] #
		# 着陸判定用
	        self.countAccDropLoop = 0
	        self.countPressDropLoop = 0
		
		# =============================================== bool =============================================== 
	        self.time_tf = False
	        self.acc_tf = False
	        self.press_tf = False
		
		
		# ============================================= 変数の初期化 ============================================= 
		self.temp = 0 #
		self.pressure = 0 #
		self.altitude = 0 #
		self.ax= 0 #
		self.ay= 0 #
		self.az= 0 #
		self.gx= 0 #
		self.gy= 0 #
		self.gz= 0 #
		self.ex= 0 #
		self.lat = 0 #
		self.lon = 0 #
		self.mkdir()
		
	def mkdir(self):
		"""
  			フォルダの作成
  		"""
		self.results_dir = f'results/{self.startTime}'
		self.results_img_dir = self.results_dir + '/imgs'
		os.mkdir(self.results_dir)
		os.mkdir(self.results_img_dir)
		return
		
	def mkfile(self):
		pass
		
	def mvfile(self):
		pass
		
	def writeData(self):
		"""
  			データの記録
  		"""
        
		datalog = str(self.timer) + ","\
                  + "state:"+str(self.state) + ","\
                  + "Time:"+str(self.gps.Time) + ","\
                  + "Lat:"+str(self.gps.Lat).rjust(6) + ","\
                  + "Lng:"+str(self.gps.Lon).rjust(6) + ","\
                  + "ax:"+str(self.ax).rjust(6) + ","\
                  + "ay:"+str(self.ay).rjust(6) + ","\
                  + "az:"+str(self.az).rjust(6) + ","\
                  + "q:"+str(self.ex).rjust(6)
		print(datalog)

		with open(f'results/{self.startTime}/control_result.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
				test.write(datalog + '\n')
            
	def writeMissionlog(self):
		pass
		
	# =================== mission sequence ===================
	def sequence(self):
		"""
  			ミッションシーケンスを管理する
			main.pyで毎周期実行される。
  		"""
		
		
		if self.state == 0:
#			print("\033[32m","","\033[0m")
			self.preparing()			
			
		elif self.state == 1:
			self.flying()
		elif self.state == 2:
			print("\033[32m",2,"\033[0m")
		elif self.state == 3:
			print("\033[32m",3,"\033[0m")
		elif self.state == 4:
			print("\033[32m",4,"\033[0m")
		elif self.state == 5:
			print("\033[32m",5,"\033[0m")
		elif self.state == 6:
			print("\033[32m",6,"\033[0m")
		elif self.state == 7:
			print("\033[32m",7,"\033[0m")
	
	def sensor_setup(self):
		# センサのセットアップを実行
		self.gps.setupGps()
		self.bno055.setupBno()
		self.bno055.bnoInitial()
		#self.lora.sendDevice.setup_lora()
		#self.arm.setup()
		if self.bno055.begin() is not True:
			print("Error initializing device")
			exit()
		pass

	def sensor(self):
		# センサの値を取得
		self.gps.gpsread()
		self.bno055.bnoread()
		self.temp,self.pressure,self.altitude = self.bmp.readBMP()
		self.ax=round(self.bno055.ax,3)
		self.ay=round(self.bno055.ay,3)
		self.az=round(self.bno055.az,3)
		self.gx=round(self.bno055.gx,3)
		self.gy=round(self.bno055.gy,3)
		self.gz=round(self.bno055.gz,3)
		self.ex=round(self.bno055.ex,3)
		self.lat = round(float(self.gps.Lat),5)
		self.lon = round(float(self.gps.Lon),5)
		
		self.writeData()
		pass
	
	def preparing(self):	
		self.RED_LED.led_on()
		
		if self.preparingTime == 0:
            # self.pc2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
            # self.img = self.pc2.capture(0,self.results_img_dir+f'/{self.cameraCount}')
			self.preparingTime = time.time()#時刻を取得
			self.RED_LED.led_on()
			self.BLUE_LED.led_off()
			self.GREEN_LED.led_off()
		if not self.preparingTime == 0:
			if self.gpscount <= ct.const.PREPARING_GPS_COUNT_THRE:
				self.startgps_lon.append(float(self.gps.Lon))
				self.startgps_lat.append(float(self.gps.Lat))
				self.gpscount+=1
                
			else:
				print("GPS completed!!")
				
			if time.time() - self.preparingTime > ct.const.PREPARING_TIME_THRE:
				self.startlon=np.mean(self.startgps_lon)
				self.startlat=np.mean(self.startgps_lat)
				self.state = 1
				self.laststate = 1
		time.sleep(0.1)
		self.RED_LED.led_off()
		
		
	def flying(self): #フライトピンが外れる➡︎ボイド缶から放出されたことを検出するステート
		self.BLUE_LED.led_on()
		if self.flyingTime == 0:#時刻を取得してLEDをステートに合わせて光らせる
			self.flyingTime = time.time()
			self.RED_LED.led_off()
			self.BLUE_LED.led_on()
			self.GREEN_LED.led_off()

		# if GPIO.input(ct.const.FLIGHTPIN_PIN) == GPIO.HIGH: #highかどうか＝フライトピンが外れているかチェック
		# 	self.countFlyLoop+=1
		# 	if self.countFlyLoop > ct.const.FLYING_FLIGHTPIN_COUNT_THRE: #一定時間HIGHだったらステート移行
		# 		self.state = 2
		# 		self.laststate = 2       
		# else:
		# 	self.countFlyLoop = 0 #何故かLOWだったときカウントをリセット
		
		time.sleep(0.2)
		self.BLUE_LED.led_off()
		
		print("\033[32m",1,"\033[0m")
    
	def landing(self):
		# landstate = 0: 着陸判定 -> 分離シート焼き切り
		trigger = self.judge_arrival(self.landtime,t, self.ax, self.ay, self.az, self.pressure)
		pass
		
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
			print("\033[32m","--<Successful landing>--","\033[0m")
			return True
	        else:
			print(f"\033[32m","time:{self.time_tf} ; acc:{self.acc_tf} ; pressure:{self.press_tf}","\033[0m")
			return False
		
	def para_escaping(self):
		# landstate = 1: カメラ台回転, オレンジ検出 -> パラ脱出
		# 撮影
		# 左向く-> 記録
		# 正面向く-> 記録
		# 右向く-> 記録
		# 回避しながらmotor.go()
		# stuck検知
		pass
		
	def first_releasing(self):
		# 焼き切り放出
		pass
	def moving_release_position(self):
		# releasing_state = 1: 接近
		## 作戦１：放出モジュールが十分に遠いとき
		## 作戦２：放出モジュールが遠いとき
		
		# releasing_state = 2: 判定
		## 加速度センサによる姿勢推定と投射角度の確認
		
		# releasing_state = 3: 微調整
		## 回転機構による投射角変更
		pass
	def judgement(self):
		pass
	def finish(self):
		pass
	
	
	def keyboardinterrupt(self): #キーボードインタラプト入れた場合に発動する関数
		pass
	
			
	
	
		
