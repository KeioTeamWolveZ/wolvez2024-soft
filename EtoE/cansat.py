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
import cv2.aruco as aruco
from glob import escape, glob
from picamera2 import Picamera2 
from libcamera import controls
from numpy import arccos, arctan2, sin, cos, tan, deg2rad, rad2deg

import constant as ct
from Wolvez2024_now.led import led
from Wolvez2024_now.gps import GPS
from Wolvez2024_now.bno055 import BNO055
from Wolvez2024_now.bmp import BMP
from Wolvez2024_now.motor_pico import motor as motor
from Wolvez2024_now.Color_tools import Color_tools
from Wolvez2024_now.Ar_tools import Artools



"""
ステート説明
0. preparing()
1. flying()
2. landing()
3. para_escaping()
4. first_releasing()
5. moving_release_position()
6. judgement()
7. running()
8. finish()

"""
class Cansat():
	def __init__(self,state):
		
		# ================================================ GPIO ================================================ 
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #GPIOの設定
		GPIO.setup(ct.const.FLIGHTPIN_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP) #フライトピン用。プルアップを有効化

		
		# ============================================== constant ============================================== 
		
		self.TIME_THRESHOLD = 60 # ct.const.DROPPING_TIME_THRE
		self.DROPPING_ACC_THRE = 0.005 # ct.const.DROPPING_ACC_THRE
		self.DROPPING_PRESS_THRE = 99887 # ct.const.DROPPING_PRESS_THRE
		self.DROPPING_ACC_COUNT_THRE = 20 # ct.const.DROPPING_ACC_COUNT_THRE
		self.DROPPING_PRESS_COUNT_THRE = 20 # ct.const.DROPPING_PRESS_COUNT_THRE
		
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

		# =============================================== ARマーカ ===============================================
		self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		# マーカーサイズの設定
		self.marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
		self.camera_matrix = np.load("mtx.npy")
		self.distortion_coeff = np.load("dist.npy")
		self.find_marker = False
		self.ar = Artools()
		self.VEC_GOAL = [0.0,0.1968730025228114,0.3]
		self.closing_threshold = 0.4
		self.CLOSING_RANGE_THRE = 0.02
		
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

		self.releasing_state == 2
    

		
		# =============================================== 時間記録 =============================================== 
		self.preparingTime = 0 #
		self.flyingTime = 0 #
		self.landtime = 0
		self.escapeTime = 0
		
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
		# 逆さま検知
		self.mirror_count = 0
		# flight
		self.countFlyLoop = 0
		# AR
		self.ultra_count = 0
		self.reject_count = 0 # 拒否された回数をカウントするための変数
		# =============================================== bool =============================================== 
		self.time_tf = False
		self.acc_tf = False
		self.press_tf = False
		self.flight = True
		self.mirrer = False
		self.prev = np.array([])
		self.TorF = True
		
		
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
		self.yunosu_pos = "Left"
		self.last_pos = "Plan_A"
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
			self.running()
		elif self.state == 8:
			print("\033[32m",8,"\033[0m")
			self.finish()
		else:
			self.state = self.laststate #どこにも引っかからない場合何かがおかしいのでlaststateに戻してあげる
	
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
		self.landtime = time.time()
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
				# ~ self.cameraCount += 1
				self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
				# 指定色のマスクを作成
				mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
				# 輪郭を抽出して最大の面積を算出し、線で囲む
				mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
				cX_right.append(cX)
			# 左を向く
			# ??????????????
			for i in range(5):
				# ~ self.cameraCount += 1
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
	        if time.time() - t > 60: # TIME_THRESHOLD
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
	        print(f"{press} > {self.DROPPING_PRESS_THRE}")
	        if press > self.DROPPING_PRESS_THRE: #気圧が閾値以上で着地判定
	            self.countPressDropLoop+=1      
	            if self.countPressDropLoop > self.DROPPING_PRESS_COUNT_THRE: #気圧判定の複数回連続成功が必要
	                self.press_tf = True
	        else:
	            self.countPressDropLoop = 0 #初期化の必要あり
	            self.press_tf = False
	        if self.time_tf and self.acc_tf and self.press_tf:
	            print("\033[32m","--<Successful landing>--","\033[0m")
	            time.sleep(100)
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
		self.frame2 = cv2.rotate(self.frame,cv2.ROTATE_90_CLOCKWISE)
		height = self.frame2.shape[0]
		width = self.frame2.shape[1]
		# オレンジ色のマスクを作成
		mask_orange = self.color.mask_color(self.frame2,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
		# 輪郭を抽出して最大の面積を算出し、線で囲む
		mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
		print("\033[33m","COLOR : ","\033[0m","cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
		self.upsidedown_checker()
		print("mirror:",self.mirror)
		motor_st_vref = 70
		motor_tr_vref = 100
		if self.mirror:
			# 逆さなら曲がる向きが反対になる
			# print("cX:",cX)
			motor_st_vref = - motor_st_vref
			motor_tr_vref = - motor_tr_vref

		if not cX : # パラシュートが見えていない時 -> 直進
			self.motor1.go(motor_st_vref)
			self.motor2.go(motor_st_vref)
			self.stuck_detection()
			print("---motor go---")
			# 一定時間経過した後に次のステートに移行
			print(self.mirror_count)
			if self.escapeTime == 0:
				self.escapeTime = time.time()
			elif time.time() - self.escapeTime > ct.const.PARA_ESCAPE_TIME_THRE:
				self.motor1.stop()
				self.motor2.stop()
				if self.mirror_count > 10:
					print("here")
					self.stuck_detection() # 止まっているときにやることで強制的にぐるぐるさせる
					self.mirror_count = 0
					# self.pre_motorTime = time.time() # 去年はこの変数を色んなステートで再利用していた？
					# 反転を解決するために頑張る
					self.motor1.go(ct.const.LANDING_MOTOR_VREF)
					self.motor2.go(ct.const.LANDING_MOTOR_VREF)
					time.sleep(3)
					self.motor1.stop()
					self.motor2.stop()
					# time.sleep()がいるかも？
				self.state += 1
				print("==============finish================")
		else: # パラシュートが見えているとき -> 回避
			self.escapeTime = 0
			print(cX > width/2)
			if cX > width/2:
				print("---motor right---")
				self.motor1.go(0)
				self.motor2.go(motor_tr_vref)
				# time.sleep(0.7)
				# self.motor1.stop()
				# self.motor2.stop()
				self.stuck_detection()
			else:
				print("---motor left---")
				self.motor1.go(motor_tr_vref)
				self.motor2.go(0)
				# time.sleep(0.7)
				# self.motor1.stop()
				# self.motor2.stop()
				# 回避しながらmotor.go()
				# stuck検知
				self.stuck_detection()
		
	def first_releasing(self): # state = 4
		print("'\033[44m'","4.first_releasing",'\033[0m')
		# self.separation()
		# 焼き切り放出
		pass

	def moving_release_position(self): # state = 5
		if self.releasing_state == 1 #: 接近
			## 作戦１：放出モジュールが十分に遠いとき
			## 作戦２：放出モジュールが遠いとき
			print("'\033[44m'","5-1.moving_release_position",'\033[0m')
			self.frame = self.picam2.capture_array()
			self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
			height = self.frame2.shape[0]
			width = self.frame2.shape[1]
			gray = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY) # グレースケールに変換
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary) # ARマーカーの検出   

			if ids is not None:
				# aruco.DetectedMarkers(frame, corners, ids)
				for i in range(len(ids)):
					if ids[i] in [0,1,2,3,4,5]:
						image_points_2d = np.array(corners[i],dtype='double')
						# print(corners[i])

						rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length,self.camera_matrix, self.distortion_coeff)
						tvec = np.squeeze(tvec)
						rvec = np.squeeze(rvec)
						# 回転ベクトルからrodoriguesへ変換
						rvec_matrix = cv2.Rodrigues(rvec)
						rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
						transpose_tvec = tvec[np.newaxis, :].T # 並進ベクトルの転置
						proj_matrix = np.hstack((rvec_matrix, transpose_tvec)) # 合成
						euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # オイラー角への変換[deg]
						self.prev = list(self.prev)

						if self.ultra_count < 20:
							self.prev.append(tvec)
							print("ARマーカーの位置を算出中")
							self.ultra_count += 1 #最初（位置リセット後も）は20回取得して平均取得
							self.find_marker = True
						else:
							# print("prev_length: ",len(prev))
							self.TorF = self.ar.outlier(tvec, self.prev, self.ultra_count, 0.3) # true:correct, false:outlier
							self.ultra_count += 1
							if self.TorF: # detected AR marker is reliable
								self.reject_count = 0
								print("x : " + str(tvec[0]))
								print("y : " + str(tvec[1]))
								print("z : " + str(tvec[2]))
								tvec[0] = tvec[0]
								polar_exchange = self.ar.polar_change(tvec)
								print(f"yunosu_function_{ids[i]}:",polar_exchange)
								distance_of_marker = polar_exchange[0] #r
								angle_of_marker = polar_exchange[1] #theta
								print("======",distance_of_marker)
								
								if distance_of_marker >= self.closing_threshold + self.CLOSING_RANGE_THRE:
									if tvec[0] >= 0.05:
										print("---motor LEFT---")
										self.motor_control(70,45,0.5) # m1:右、m2:左、time:時間
										yunosu_pos = "Left"
											
									elif 0.05 > tvec[0] > -0.05:
										go_ahead_gain = (distance_of_marker-self.closing_threshold) / self.closing_threshold
										print("---motor GO AHEAD---")
										self.motor_control(40+60*go_ahead_gain,40+60*go_ahead_gain,0.5) # m1:右、m2:左、time:時間
										self.motor1.stop()
										self.motor2.stop()
									
									else:
										print("---motor RIGHT---")
										self.motor_control(45,70,0.5) # m1:右、m2:左、time:時間
										yunosu_pos = "Right"

								elif distance_of_marker >= self.closing_threshold:
									if tvec[0] >= 0.03:
										print("---turn RIGHT---")
										self.motor_control(45,-45,0.3) # m1:右、m2:左、time:時間
									elif tvec[0] <= -0.03:
										print("---turn LEFT---")
										self.motor_control(-45,45,0.3) # m1:右、m2:左、time:時間
									else:
										print("'\033[32m'---perfect REACHED---'\033[0m'")

								if distance_of_marker <= self.closing_threshold - self.CLOSING_RANGE_THRE:
									if -20 <= angle_of_marker <= 0: #ARマーカがやや左から正面にある場合
										print("右回転")
										self.motor_control(70,-70,0.3) # m1:右、m2:左、time:時間
										yunosu_pos = "Left"
									
									
									elif 0 <= angle_of_marker <= 20: #ARマーカがやや右から正面にある場合
										print("左回転")
										self.motor_control(-70,70,0.5) # m1:右、m2:左、time:時間
										yunosu_pos = "Right"
									
									else: #4+k秒ただ直進(ARマーカから離れる)   
										print("直進")
										last_pos = "Plan_B"
										self.motor_control(70,70,2.5) # m1:右、m2:左、time:時間

							else: # detected AR marker is not reliable
								print("state of marker is rejected")
								self.find_marker = False
								print(self.ultra_count)
								self.reject_count += 1 # 拒否された回数をカウント
								if self.reject_count > 10: # 拒否され続けたらリセットしてARマーカーの基準を上書き（再計算）
									self.ultra_count = 0
									self.reject_count = 0 #あってもなくても良い
			
			if last_pos == "Plan_A" and not self.find_marker: #ARマーカを認識していない時，認識するまでその場回転
				if yunosu_pos == "Left":
					print("ARマーカー探してます(LEFT)")
					self.motor_control(-60,60,0.5) # m1:右、m2:左、time:時間
				
				elif yunosu_pos == "Right":
					print("ARマーカー探してます(RIGHT)")
					self.motor_control(60,-60,0.5) # m1:右、m2:左、time:時間
				
			elif last_pos == "Plan_B":
				print("Plan_B now")
				self.motor_control(70,70,0.5) # m1:右、m2:左、time:時間
				last_pos = "Plan_A"
			pass

		elif self.releasing_state == 2:
			"""
				微調整ステート
			"""
			print("'\033[44m'","5-2.moving_release_position",'\033[0m')
			pass
		elif self.releasing_state == 3:
			"""
				物資モジュール投射
			"""
			print("'\033[44m'","5-3.moving_release_position",'\033[0m')
			pass
			

	def motor_control(self,m1,m2,time):
		# m1:右モーターの速度
		# m2:左モーターの速度
		# time:モーターを動かす時間
		if m1>=0:
			self.motor1.go(m1)
		else:
			self.motor1.back(m1)
		if m2>=0:
			self.motor2.go(m2)
		else:
			self.motor2.back(m2)
		time.sleep(time)
		self.motor1.stop()
		self.motor2.stop()

	def judgement(self): # state = 6
		pass

	def stuck_detection(self):
		print(self.ax**2+self.ay**2)
		if (self.ax**2+self.ay**2) <= ct.const.STUCK_ACC_THRE**2 or (self.ax**2+self.ay**2) > 8:
			print("stack??")
			if self.stuckTime == 0:
				self.stuckTime = time.time()
			
			if self.countstuckLoop > ct.const.STUCK_COUNT_THRE or self.state == 1 or self.state >= 6 or self.mirror_count > 10: #加速度が閾値以下になるケースがある程度続いたらスタックと判定
				#トルネード実施
				print("===================stuck====================")
				self.motor1.back(ct.const.STUCK_MOTOR_VREF)
				self.motor2.back(ct.const.STUCK_MOTOR_VREF)
				time.sleep(1)
				self.motor1.stop()
				self.motor2.stop()
				self.motor1.go(ct.const.STUCK_MOTOR_VREF)
				self.motor2.back(ct.const.STUCK_MOTOR_VREF)
				time.sleep(2)
				self.motor1.stop()
				self.motor2.stop()
				# self.rv = ct.const.STUCK_MOTOR_VREF
				# self.lv = -ct.const.STUCK_MOTOR_VREF
				self.countstuckLoop = 0
				self.stuckTime = 0

			self.countstuckLoop+= 1

		else:
			# ~ self.countstuckLoop = 0 # change 0 when state chenge
			self.stuckTime = 0
				
	def upsidedown_checker(self):
		# 逆さまの検知（着地時に実施を想定）
		if self.gz < 5: # gz?が閾値以下で逆さまと判定
			self.mirror_count += 1
			self.mirror = True
		else:
			self.mirror_count = 0
			self.mirror = False

	def separation(self,pin):
		GPIO.setup(pin,GPIO.OUT) #焼き切り用のピンの設定tv 
		GPIO.output(pin,0) #焼き切りが危ないのでlowにしておく
		GPIO.output(pin,1) #電圧をHIGHにして焼き切りを行う
		time.sleep(10) #継続時間を指定
		GPIO.output(pin,0) #電圧をLOWにして焼き切りを終了する
		print("Separation done")
	
	def running(self): # state = 7
		dlon = self.goallon - self.lon
		# distance to the goal
		self.goaldis = ct.const.EARTH_RADIUS * arccos(sin(deg2rad(self.lat))*sin(deg2rad(self.goallat)) + cos(deg2rad(self.lat))*cos(deg2rad(self.goallat))*cos(deg2rad(dlon)))
		print(f"Distance to goal: {round(self.goaldis,4)} [km]")

		# angular to the goal (North: 0, South: 180)
		self.goalphi = 90 - rad2deg(arctan2(cos(deg2rad(self.lat))*tan(deg2rad(self.goallat)) - sin(deg2rad(self.lat))*cos(deg2rad(dlon)), sin(deg2rad(dlon))))
		if self.goalphi < 0:
		    self.goalphi += 360
		print(self.goalphi)
		
		self.arg_diff = self.goalphi - (self.ex-0)
		if self.arg_diff < 0:
		    self.arg_diff += 360
		
		print(f"Argument to goal: {round(self.arg_diff,2)} [deg]")
		
		if self.runningTime == 0:
		    self.runningTime = time.time()
		    
		# elif time.time() - self.runningTime < 10:
		    # print("run")
		    
		elif self.goaldis < ct.const.GOAL_DISTANCE_THRE:
		    self.motor1.stop()
		    self.motor2.stop()
		    self.goaltime = time.time()-self.runningTime
		    self.running_finish = True
		    print(f"Goal Time: {self.goaltime}")
		    print("GOAAAAAAAAAL!!!!!")
		    self.state = 8
		    self.laststate = 8
		
		else:
		    if self.arg_diff <= 180 and self.arg_diff > 20:
				self.motor1.go(ct.const.RUNNING_MOTOR_VREF-15)
				self.motor2.go(ct.const.RUNNING_MOTOR_VREF)
			
		    elif self.arg_diff > 180 and self.arg_diff < 340:
				self.motor1.go(ct.const.RUNNING_MOTOR_VREF)
				self.motor2.go(ct.const.RUNNING_MOTOR_VREF-15)
		    
		    else:
				self.motor1.go(ct.const.RUNNING_MOTOR_VREF)
				self.motor2.go(ct.const.RUNNING_MOTOR_VREF)

	def finish(self): # state = 8
		if self.finishTime == 0:
		    self.finishTime = time.time()
		    print("\n",self.startTime)
		    print("\nFinished\n")
		    self.motor1.stop()
		    self.motor2.stop()
		    GPIO.output(ct.const.SEPARATION_PARA,0) #焼き切りが危ないのでlowにしておく
		    GPIO.output(ct.const.SEPARATION_MOD1,0) #焼き切りが危ないのでlowにしておく
		    GPIO.output(ct.const.SEPARATION_MOD2,0) #焼き切りが危ないのでlowにしておく
		    self.RED_LED.led_off()
		    self.BLUE_LED.led_off()
		    self.GREEN_LED.led_off()
		    self.pc2.stop()
		    time.sleep(0.5)
		    cv2.destroyAllWindows()
		    sys.exit()

	
	def keyboardinterrupt(self): #キーボードインタラプト入れた場合に発動する関数
		self.motor1.stop()
		self.motor2.stop()
		GPIO.output(ct.const.SEPARATION_PARA,0) #焼き切りが危ないのでlowにしておく
		GPIO.output(ct.const.SEPARATION_MOD1,0) #焼き切りが危ないのでlowにしておく
		GPIO.output(ct.const.SEPARATION_MOD2,0) #焼き切りが危ないのでlowにしておく
		self.RED_LED.led_off()
		self.BLUE_LED.led_off()
		self.GREEN_LED.led_off()
		self.pc2.stop()
		time.sleep(0.5)
		cv2.destroyAllWindows()
		pass
	
			
	
	
		
