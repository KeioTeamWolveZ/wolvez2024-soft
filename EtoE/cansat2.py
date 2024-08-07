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
from Wolvez2024_now.motor_pico import motor as motor
from Wolvez2024_now.Color_tools import Color_tools


"""
ステート説明
0. preparing()
1. flying()
2. landing()
3. para_escaping()
4. first_releasing()
5. moving_release_position()
6. judgement()
7. finish()

"""
class Cansat():
	def __init__(self,state):
		
		# ================================================ GPIO ================================================ 
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #GPIOの設定
		GPIO.setup(ct.const.FLIGHTPIN_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP) #フライトピン用。プルアップを有効化

		
		# ============================================== constant ============================================== 
		
		self.TIME_THRESHOLD = 3 # ct.const.DROPPING_TIME_THRE
		self.DROPPING_ACC_THRE = 0.005 # ct.const.DROPPING_ACC_THRE
		self.DROPPING_PRESS_THRE = 100000 # ct.const.DROPPING_PRESS_THRE
		self.DROPPING_ACC_COUNT_THRE = 30 # ct.const.DROPPING_ACC_COUNT_THRE
		self.DROPPING_PRESS_COUNT_THRE = 30 # ct.const.DROPPING_PRESS_COUNT_THRE
		
		# =============================================== モータ =============================================== 
		# ~ GPIO.setwarnings(False)
		# ~ self.MotorL = motor(ct.const.RIGHT_MOTOR_IN1_PIN,ct.const.RIGHT_MOTOR_IN2_PIN,ct.const.RIGHT_MOTOR_VREF_PIN)
		# ~ self.MotorR = motor(ct.const.LEFT_MOTOR_IN1_PIN,ct.const.LEFT_MOTOR_IN2_PIN, ct.const.LEFT_MOTOR_VREF_PIN)
		GPIO.setwarnings(False)
		self.motor1 = motor(6,5,13)
		self.motor2 = motor(20,16,12,-1)
		# =============================================== カメラ =============================================== 
		self.picam2 = Picamera2()
		size = (1100, 1800)
		config = self.picam2.create_preview_configuration(
		main={"format": 'XRGB8888', "size": size})
		self.picam2.align_configuration(config)
		self.picam2.configure(config)
		self.picam2.start()
		# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
		self.picam2.set_controls({"AfMode":0,"LensPosition":5.5})
		
		# ================================================= LED ================================================= 
		self.RED_LED = led(ct.const.RED_LED_PIN) # 
		self.BLUE_LED = led(ct.const.BLUE_LED_PIN) # 
		self.GREEN_LED = led(ct.const.GREEN_LED_PIN) # 

		# =============================================== センサ =============================================== 
		self.gps = GPS() #
		self.bno055 = BNO055() #
		self.bmp = BMP() #
		self.color = Color_tools(ct.const.LOWER_ORANGE,ct.const.LOWER_ORANGE)
		
		# ============================================ ステート管理 ============================================ 
		self.timer = 0 # 
		self.state = state # 
		self.last_state = 0 #
		self.time = 0 #
		self.startTime_time=time.time() #
		self.startTime = str(datetime.now())[:19].replace(" ","_").replace(":","-") #
		self.stuckTime = 0
    
		

		
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
		self.cameraCount = 0 # camera
		# 着陸判定用
		self.countAccDropLoop = 0
		self.countPressDropLoop = 0
		# スタック検知
		self.countstuckLoop = 0	
		# flight
		self.countFlyLoop = 0
		# =============================================== bool =============================================== 
		self.time_tf = False
		self.acc_tf = False
		self.press_tf = False
		self.flight = True
		
		
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
                  + "q:"+str(self.ex).rjust(6) + ","\
                  + "pressure:"+str(self.pressure).rjust(6)
		print(datalog)

		with open(f'results/{self.startTime}/control_result.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
				test.write(datalog + '\n')
            
	def writeMissionlog(self):
	    mission_log = str(self.timer) + ","\
		    + "state:"+str(self.state) 
	    if self.state == 1:
		    mission_log = mission_log + "," + "Flight_PIN:" + "True" # フライトピン
	    if self.state == 2:
		    mission_log = mission_log + ","\
		    + "Casat_Landing:" + str(self.trigger) # 着地判定
	    if self.state == 3:
		    mission_log = mission_log + ","\
		    + "Para_distancing:" + str(self.distancing_finish) # パラから距離を取る
	    # if self.state == 4:
	    #     mission_log = mission_log + ","\
	    #         + "Releasing_01:"  + str(self.releasing_01) # 電池モジュール焼き切り
	    #         + ","　+ "Releasing_01:"  + str(self.releasing_01) # 電池モジュール焼き切り
	    # if self.state == 5:
	    #     mission_log = mission_log + ","\
	    #         + "Releasing_02:"  + str(self.releasing_02) # 電力消費モジュール焼き切り
	    # if self.state == 6:
	    #     mission_log = mission_log + ","\
	    #         + "ConnectingState:" + str(self.connecting_state) + ","\
	    #         + "Done-Approach:" + str(self.done_approach) + ","\
	    #         + "Done-Connect:" + str(self.connected)

	    with open(f'results/{self.startTime}/mission_log.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
		    test.write(mission_log + '\n')
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
			self.landing()
		elif self.state == 3:
			self.para_escaping()
			pass
		elif self.state == 4:
			pass
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
	
	def preparing(self): # state = 0
		self.img = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
		print("'\033[44m'","0.preparing",'\033[0m')
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
		
		
	def flying(self): # state = 1
    #フライトピンが外れる➡︎ボイド缶から放出されたことを検出するステート
		print("'\033[44m'","1.flying",'\033[0m')
		self.BLUE_LED.led_on()
		self.flyingTime = time.time()
		while self.flight:
			if GPIO.input(ct.const.FLIGHTPIN_PIN) == GPIO.HIGH: #highかどうか＝フライトピンが外れているかチェック
				self.countFlyLoop+=1
				if self.countFlyLoop > ct.const.FLYING_FLIGHTPIN_COUNT_THRE: #一定時間HIGHだったらステート移行
					self.flight = False	
			else:
				self.countFlyLoop = 0 #何故かLOWだったときカウントをリセット
				time.sleep(3)
			print("=====flying=====")
		self.state = 2
		time.sleep(0.2)
		self.BLUE_LED.led_off()
		
		print("\033[32m",1,"\033[0m")
    
	def landing(self): # state = 2
		# landstate = 0: 着陸判定 -> 分離シート焼き切り
		print("'\033[44m'","2.landing",'\033[0m')
		trigger = self.judge_arrival(self.landtime, self.ax, self.ay, self.az, self.pressure)
		if trigger:
			cX_right = []
			cX_left = []
			# 右を向くコード
			# ??????????????
			for i in range(5):
				self.cameraCount += 1
				self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
				# 指定色のマスクを作成
				mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
				# 輪郭を抽出して最大の面積を算出し、線で囲む
				mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
				cX_right.append(cX)
			# 左を向く
			# ??????????????
			for i in range(5):
				self.cameraCount += 1
				self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
				# 指定色のマスクを作成
				mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
				# 輪郭を抽出して最大の面積を算出し、線で囲む
				mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
				cX_left.append(cX)
			# カメラ回転機構の正常動作の判定
			try :         
				if abs(np.array(cX_right).mean() - np.array(cX_right).mean()) < ct.const.CAMERA_ROTATION_THRE:
					print("\033[33m","MISSION : ","\033[33m", "camera rotation success!")
					# mission log
					# ?????????????????
				else:
					print("\033[33m","MISSION : ","\033[33m", "camera rotation failure")
					# mission log
					# ?????????????????
			except:
				print("failure")
			
			self.state = 3
	def judge_arrival(self, t, ax, ay, az, press):
	        """
	        引数：time:ステート以降時間、加速度の値(できればベクトル)、気圧(or高度)の値
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
	            print("\033[32m",f"time:{self.time_tf} ; acc:{self.acc_tf} ; pressure:{self.press_tf}\n{(ax**2 + ay**2 + az**2)} < {self.DROPPING_ACC_THRE**2}","\033[0m")
	            return False
		
	def para_escaping(self): # state = 3
		print("'\033[44m'","3.para_escaping",'\033[0m')
		# landstate = 3: カメラ台回転, オレンジ検出 -> パラ脱出
		# 撮影
		self.cameraCount += 1
		self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
		# オレンジ色のマスクを作成
		mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
		# 輪郭を抽出して最大の面積を算出し、線で囲む
		mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
		print("\033[33m","COLOR : ","\033[0m","cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
	    
		if not cX : # パラシュートが見えていない時 -> 直進
			self.motor1.go(70)
			self.motor2.go(70)
			time.sleep(3)
			self.motor1.stop()
			self.motor2.stop()
			print("---motor go---")
			self.state = 5
		else: # パラシュートが見えているとき -> 回避
			if cX > width/2:
				print("---motor right---")
				self.motor1.go(0)
				self.motor2.go(100)
				time.sleep(0.7)
				self.motor1.stop()
				self.motor2.stop()
			else:
				print("---motor left---")
				self.motor1.go(100)
				self.motor2.go(0)
				time.sleep(0.7)
				self.motor1.stop()
				self.motor2.stop()
				# 回避しながらmotor.go()
				# stuck検知
		pass
		
	def first_releasing(self): # state = 4
		print("'\033[44m'","4.first_releasing",'\033[0m')
		# self.separation()
		# 焼き切り放出
		pass
	def moving_release_position(self): # state = 5
		# releasing_state = 1: 接近
		## 作戦１：放出モジュールが十分に遠いとき
		## 作戦２：放出モジュールが遠いとき
		
		# releasing_state = 2: 判定
		## 加速度センサによる姿勢推定と投射角度の確認
		
		# releasing_state = 3: 微調整
		## 回転機構による投射角変更
		pass
	def judgement(self): # state = 6
		pass
	def finish(self): # state = 7
		pass

	def stuck_detection(self):
	        if (self.ax**2+self.ay**2) <= ct.const.STUCK_ACC_THRE**2:
	            if self.stuckTime == 0:
	                self.stuckTime = time.time()
	            
	            if self.countstuckLoop > ct.const.STUCK_COUNT_THRE or self.landstate == 1 or self.state >= 6: #加速度が閾値以下になるケースがある程度続いたらスタックと判定
	                #トルネード実施
	                print("stuck")
	                self.MotorR.go(ct.const.STUCK_MOTOR_VREF)
	                self.MotorL.back(ct.const.STUCK_MOTOR_VREF)
	                time.sleep(2)
	                self.MotorR.stop()
	                self.MotorL.stop()
	                # self.rv = ct.const.STUCK_MOTOR_VREF
	                # self.lv = -ct.const.STUCK_MOTOR_VREF
	                self.countstuckLoop = 0
	                self.stuckTime = 0
	
	            self.countstuckLoop+= 1
	
	        else:
	            self.countstuckLoop = 0
	            self.stuckTime = 0
	def separation(self,pin):
		GPIO.setup(pin,GPIO.OUT) #焼き切り用のピンの設定tv 
		GPIO.output(pin,0) #焼き切りが危ないのでlowにしておく
		GPIO.output(pin,1) #電圧をHIGHにして焼き切りを行う
		time.sleep(10) #継続時間を指定
		GPIO.output(pin,0) #電圧をLOWにして焼き切りを終了する
		print("Separation done")

	
	def keyboardinterrupt(self): #キーボードインタラプト入れた場合に発動する関数
		pass
	
			
	
	
		
