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
from Wolvez2024_now.lora import lora
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
	def __init__(self,state,sepa_mode):
		
		# ================================================ GPIO ================================================ 
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #GPIOの設定
		GPIO.setup(ct.const.FLIGHTPIN_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP) #フライトピン用。プルアップを有効化

		
		# ============================================== constant ============================================== 
		
		self.TIME_THRESHOLD = 10 # ct.const.DROPPING_TIME_THRE
		self.DROPPING_ACC_THRE = 0.005 # ct.const.DROPPING_ACC_THRE
		self.DROPPING_PRESS_THRE = 99887 # ct.const.DROPPING_PRESS_THRE
		self.DROPPING_ACC_COUNT_THRE = 20 # ct.const.DROPPING_ACC_COUNT_THRE
		self.DROPPING_PRESS_COUNT_THRE = 20 # ct.const.DROPPING_PRESS_COUNT_THRE
		
		# =============================================== モータ =============================================== 
		# ~ GPIO.setwarnings(False)
		# ~ self.MotorL = motor(ct.const.RIGHT_MOTOR_IN1_PIN,ct.const.RIGHT_MOTOR_IN2_PIN,ct.const.RIGHT_MOTOR_VREF_PIN)
		# ~ self.MotorR = motor(ct.const.LEFT_MOTOR_IN1_PIN,ct.const.LEFT_MOTOR_IN2_PIN, ct.const.LEFT_MOTOR_VREF_PIN)
		GPIO.setwarnings(False)
		self.motor1 = motor(dir = -1)
		self.motor2 = motor()
		self.servo = motor()
		self.servo.set_id(2)
		# =============================================== カメラ =============================================== 
		self.picam2 = Picamera2()
		size = (1800, 2400)
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
		self.closing_threshold = 0.5
		self.CLOSING_RANGE_THRE = 0.1
		self.closing_threshold_2 = 0.25
		self.CLOSING_RANGE_THRE_2 = 0.1
		
		# ================================================= LED ================================================= 
		self.RED_LED = led(ct.const.RED_LED_PIN) # 
		self.BLUE_LED = led(ct.const.BLUE_LED_PIN) # 
		self.GREEN_LED = led(ct.const.GREEN_LED_PIN) # 

		# =============================================== センサ =============================================== 
		self.gps = GPS() #
		self.bno055 = BNO055() #
		self.bmp = BMP() #
		self.color = Color_tools(ct.const.LOWER_ORANGE,ct.const.LOWER_ORANGE)
		self.lora = lora()
		
		# ============================================ ステート管理 ============================================ 
		self.timer = 0 # 
		self.state = state # 
		self.last_state = 0 #
		self.time = 0 #
		self.startTime_time=time.time() #
		self.startTime = str(datetime.now())[:19].replace(" ","_").replace(":","-") #
		self.stuckTime = 0
		self.releasing_state = 1
		self.closing_state = 1
		self.justAngle = False
		self.sepa_mode = sepa_mode
    
		# =============================================== 時間記録 =============================================== 
		self.preparingTime = 0 #
		self.flyingTime = 0 #
		self.landtime = 0
		self.escapeTime = 0
		self.runningTime = 0
		self.finishTime = 0
		
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
		#loop
		self.state5_loopCount_color = 1	
		self.state5_loopCount_ar = 1
		# 評価
		self.judge_cnt = 0
		# =============================================== bool =============================================== 
		self.time_tf = False
		self.acc_tf = False
		self.press_tf = False
		self.flight = True
		self.mirrer = False
		self.prev = np.array([])
		self.TorF = True
		self.rot_cam = False
		self.distancing_finish = False
		
		
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
		
		self.lost_marker_cnt = 0
		self.k = 0
		
		self.goaldis = 0 # gps running
		self.goalphi = 0 # gps running
		self.rv, self.lv = 0, 0
		
		self.goallat = ct.const.GPS_GOAL_LAT
		self.goallon = ct.const.GPS_GOAL_LON
		
		self.flag_AR = False
		self.tvec = [999,999,999]
		self.flag_COLOR = False
		self.control_log1 = "" # approach position or escape
		self.control_log2 = "" # motion tyokusinn miginaamemae sonoba-migi ushiro
		self.control_log_rv = 0 # right motor output
		self.control_log_lv = 0 # left motor output
		
		self.yunosu_pos = "Left"
		self.last_pos = "Plan_A"
		self.last_marker_num = 0

		self.mkdir()

		self.nowangle = 90  # サーボモータの角度
		
		
		
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
			releasing_state: self.releasing_state = 1
			self.closing_state = 1
			print("control_log1 : ",self.control_log1)
			print("control_log2 : ",self.control_log2)
			print("control_log_rv :",self.control_log_rv) 
			print("control_log_lv :",self.control_log_lv) 
  		"""
        
		datalog = str(self.timer) + ","\
                  + "state:"+str(self.state) + ","\
		  + "releasing_state:"+str(self.releasing_state) + ","\
		  + "closing_state:"+str(self.closing_state) + ","\
                  + "Time:"+str(self.gps.Time) + ","\
                  + "Lat:"+str(self.gps.Lat).rjust(6) + ","\
                  + "Lng:"+str(self.gps.Lon).rjust(6) + ","\
                  + "ax:"+str(self.ax).rjust(6) + ","\
                  + "ay:"+str(self.ay).rjust(6) + ","\
                  + "az:"+str(self.az).rjust(6) + ","\
                  + "q:"+str(self.ex).rjust(6) + ","\
                  + "pressure:"+str(self.pressure).rjust(6) + ","\
                  + "cameraCount:"+str(self.cameraCount).rjust(6)\
		  + "control_log1:"+str(self.control_log1).rjust(6) + ","\
		  + "control_log2:"+str(self.control_log2).rjust(6) + ","\
		  + "rv:"+str(self.control_log_rv).rjust(6) + ","\
		  + "lv:"+str(self.control_log_lv).rjust(6)
		print("-------",datalog,"\n-------")

		with open(f'results/{self.startTime}/control_result.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
				test.write(datalog + '\n')
            
	def writeMissionlog(self,sub=1):
	    mission_log = str(self.timer) + ","\
		    + "state:"+str(self.state) 
	    if self.state == 1:
		    mission_log = mission_log + "," + "Flight_PIN:" + "True" # フライトピン
	    if self.state == 2:
		    if sub == 1:
			    mission_log = mission_log + ","\
			    + "Casat_Landing:" + str(self.trigger) # 着地判定
		    if sub == 2:
			    mission_log = mission_log + ","\
			    + "Casat_rotation_camera:" + self.rot_cam # 着地判定
	    if self.state == 3:
		    mission_log = mission_log + ","\
		    + "Para_distancing:" + str(self.distancing_finish) # パラから距離を取る
	    if self.state == 4:
	        mission_log = mission_log + ","\
	            + "Releasing_01:"  + "True" # 電池モジュール焼き切り
	    if self.state == 5:
	        if sub == 1:
	            mission_log = mission_log + ","\
	            + "color_detected:" + "True" # color
	        if sub == 2:
	            mission_log = mission_log + ","\
	            + "ar_detected:" + "True" # ar
	        if sub == 3:
	            mission_log = mission_log + ","\
	            + "reaching:" + "True" # ar
	        if sub == 4:
	            mission_log = mission_log + ","\
	            + "just_angle:" + "True" # ar
	        if sub == 5:
	            mission_log = mission_log + ","\
	            + "Releasing_02:" + "True" # ar
	    if self.state == 8:
	        mission_log = mission_log + ","\
	            + "Finish:" + "True"

	    with open(f'results/{self.startTime}/mission_log.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
		    test.write(mission_log + '\n')
		    pass
		
	# =================== mission sequence ===================
	def sequence(self):
		"""
  			ミッションシーケンスを管理する
			main.pyで毎周期実行される。
  		"""
		self.flag_AR = False
		self.flag_COLOR = False
		self.control_log1 = "---" # approach position or escape
		self.control_log2 = "---" # motion tyokusinn miginaamemae sonoba-migi ushiro
		self.control_log_rv = 999 # right motor output
		self.control_log_lv = 999 # left motor output
		
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
			self.first_releasing()
			pass
		elif self.state == 5:
			self.moving_release_position()
		elif self.state == 6:
			self.judgement()
		elif self.state == 7:
			self.running()
		elif self.state == 8:
			self.finish()
		elif self.state == 99:
			self.motor_test()
		else:
			self.state = self.laststate #どこにも引っかからない場合何かがおかしいのでlaststateに戻してあげる
		
		# ==========================================================
		if self.flag_AR:
			print("\033[43m","AR:",self.flag_AR,f" r={self.distanceAR}","\033[0m")
		else:
			print("\033[43m","AR:","\033[0m",self.flag_AR)
		
		if self.flag_COLOR:
			print("\033[43m","COLOR:",self.flag_COLOR,"\033[0m")
		else:
			print("\033[43m","COLOR:","\033[0m",self.flag_COLOR)
		
		
		print("control_log1 : ",self.control_log1)
		print("control_log2 : ",self.control_log2)
		print("control_log_rv :",self.control_log_rv) 
		print("control_log_lv :",self.control_log_lv) 
		print("yunosu_pos : ",self.yunosu_pos)
		print("last_pos : ",self.last_pos)
	
		
		
	def sensor_setup(self):
		# センサのセットアップを実行
		self.gps.setupGps()
		self.bno055.setupBno()
		self.bno055.bnoInitial()
		self.lora.sendDevice.setup_lora()
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
		if not self.state == 1: #preparingのときは電波を発しない
			self.sendLoRa()
		pass

	def sendLoRa(self): #通信モジュールの送信を行う関数
		datalog = str(self.state)+ ","\
            + str(round(self.lat,5)) + ","\
            + str(round(self.lon,5))
		self.lora.sendData(datalog) #データを送信
	
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
		self.writeMissionlog()
		self.state = 2
		self.landtime = time.time()
		time.sleep(0.2)
		self.BLUE_LED.led_off()
		
		print("\033[32m",1,"\033[0m")
    
	def landing(self): # state = 2
		# landstate = 0: 着陸判定 -> 分離シート焼き切り
		print("'\033[44m'","2.landing",'\033[0m')
		self.trigger = self.judge_arrival(self.landtime, self.ax, self.ay, self.az, self.pressure)
		if self.trigger:
			
			self.writeMissionlog()
			# para separation
			time.sleep(3)
			self.separation(ct.const.SEPARATION_PARA)
			# kaiten
			cX_right = []
			cX_left = []
			self.servo.go_deg(120)
			for i in range(5):
				self.cameraCount += 1
				self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
				self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)	
				cv2.imwrite(self.results_img_dir+f'/right_{self.cameraCount}.jpg',self.frame2)
				# 指定色のマスクを作成
				mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
				# 輪郭を抽出して最大の面積を算出し、線で囲む
				mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
				cX_right.append(cX)
				print("-")
			self.servo.go_deg(70)
			for i in range(5):
				self.cameraCount += 1
				self.frame = self.picam2.capture_array()#0,self.results_img_dir+f'/{self.cameraCount}')
				self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
				cv2.imwrite(self.results_img_dir+f'/left_{self.cameraCount}.jpg',self.frame2)
				# 指定色のマスクを作成
				mask_orange = self.color.mask_color(self.frame,ct.const.LOWER_ORANGE,ct.const.UPPER_ORANGE)
				# 輪郭を抽出して最大の面積を算出し、線で囲む
				mask_orange,cX,cY,max_contour_area = self.color.detect_color(mask_orange,ct.const.MAX_CONTOUR_THRESHOLD)
				cX_left.append(cX)
				print("-")
			# カメラ回転機構の正常動作の判定
			self.servo.go_deg(90)
			try :         
				if abs(np.array(cX_right).mean() - np.array(cX_left).mean()) > ct.const.CAMERA_ROTATION_THRE:
					print("\033[33m","MISSION : ","\033[33m", "camera rotation success!")
					self.rot_cam = True
					self.writeMissionlog(2)
				else:
					print("\033[33m","MISSION : ","\033[33m", "camera rotation failure")
					self.writeMissionlog(2)
			except:
				print("failure")
			
			self.state = 3
	def judge_arrival(self, t, ax, ay, az, press):
	        """
	        引数：time:ステート以降時間、加速度の値(できればベクトル)、気圧(or高度)の値
	        戻り値：着陸判定（着地：True,未着陸：False）
	        """
	        # 時間の判定
	        if time.time() - t > self.TIME_THRESHOLD: # TIME_THRESHOLD
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
	        if press > self.DROPPING_PRESS_THRE-100: #気圧が閾値以上で着地判定
	            self.countPressDropLoop+=1 #気圧が閾値以上で着地判定
	            self.countPressDropLoop+=1     
	            if self.countPressDropLoop > self.DROPPING_PRESS_COUNT_THRE: #気圧判定の複数回連続成功が必要
	                self.press_tf = True
	        else:
	            self.countPressDropLoop = 0 #初期化の必要あり
	            self.press_tf = False
	        if self.time_tf and self.acc_tf and self.press_tf:
	            print("\033[32m","--<Successful landing>--","\033[0m")
	            time.sleep(3)
	            self.state = 3
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
		cv2.imwrite(self.results_img_dir+f'/{self.cameraCount}.jpg',self.frame2)
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
				self.distancing_finish = True
				self.writeMissionlog()
				self.state = 4
				print("==============finish================")
		else: # パラシュートが見えているとき -> 回避
			self.escapeTime = 0
			print("\033[43m", "=====orange=====","\033[0m")
			print(cX > width/2)
			if cX > width/2:
				print("---motor right---")
				self.motor1.go(0)
				self.motor2.go(motor_tr_vref)
				self.stuck_detection()
			else:
				print("---motor left---")
				self.motor1.go(motor_tr_vref)
				self.motor2.go(0)
				# stuck検知
				self.stuck_detection()
		
	def first_releasing(self): # state = 4
		print("'\033[44m'","4.first_releasing",'\033[0m')
		self.separation(ct.const.SEPARATION_MOD1)
		self.writeMissionlog()
		# 焼き切り放出
		time.sleep(5)
		self.state = 5
		pass

	def moving_release_position(self): # state = 5
		
		if self.releasing_state == 1 :# 接近
			## 作戦１：放出モジュールが十分に遠いとき
			## 作戦２：放出モジュールが遠いとき
			print("'\033[44m'","5-1.moving_release_position",'\033[0m')
			
			self.cameraCount += 1
			self.frame = self.picam2.capture_array()
			self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
			cv2.imwrite(self.results_img_dir+f'/{self.cameraCount}.jpg',self.frame2)
			height = self.frame2.shape[0]
			width = self.frame2.shape[1]
			gray = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY) # グレースケールに変換
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary) # ARマーカーの検出   

			# オレンジ色のマスクを作成
			mask_blue = self.color.mask_color(self.frame,ct.const.LOWER_BLUE,ct.const.UPPER_BLUE)
			# 輪郭を抽出して最大の面積を算出し、線で囲む
			mask_blue,cX,cY,max_contour_area = self.color.detect_color(mask_blue,ct.const.MAX_CONTOUR_THRESHOLD)
			#print("cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
			
			if cX:
				self.flag_COLOR = True
				if self.state5_loopCount_color == 1:
					self.writeMissionlog()
					self.state5_loopCount_color +=1
			
			if ids is not None:
				if self.last_marker_num in ids:
					ids = [self.last_marker_num]
				else:
					ids = [ids[0]]
					self.last_marker_num = ids[0]
				for i in range(len(ids)):
					if ids[i] in [1,2,3,4,5,6]:
						
						if self.state5_loopCount_ar == 1:
							self.writeMissionlog(2)
							self.state5_loopCount_ar +=1
						
						self.flag_AR = True
						rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.distortion_coeff)
						tvec = np.squeeze(tvec)
						rvec = np.squeeze(rvec)
						# 回転ベクトルからrodoriguesへ変換
						rvec_matrix = cv2.Rodrigues(rvec)
						rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
						transpose_tvec = tvec[np.newaxis, :].T # 並進ベクトルの転置
						proj_matrix = np.hstack((rvec_matrix, transpose_tvec)) # 合成
						euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # オイラー角への変換[deg]
						self.prev = list(self.prev)
						self.lost_marker_cnt = 0

						
						self.reject_count = 0
						# ~ print("x : " + str(tvec[0]))
						# ~ print("y : " + str(tvec[1]))
						# ~ print("z : " + str(tvec[2]))
						self.tvec = tvec
						polar_exchange = self.ar.polar_change(tvec)
						# ~ print(f"yunosu_function_{ids[i]}:",polar_exchange)
						distance_of_marker = polar_exchange[0] #r
						self.distanceAR = distance_of_marker
						angle_of_marker = polar_exchange[1] #theta
						# ~ print("======",distance_of_marker)
						
						self.control_log1 = "closing"
						if distance_of_marker >= self.closing_threshold + self.CLOSING_RANGE_THRE:
							if tvec[0] >= 0.05:
								turn_gain = 5*((self.closing_threshold + self.CLOSING_RANGE_THRE)/(distance_of_marker))**2
								# ~ print("---右に曲がります---")
								self.motor_control(50 + turn_gain,60,0.5)
							
								
							elif 0.05 > tvec[0] > -0.05:
								go_ahead_gain = (distance_of_marker-self.closing_threshold) / self.closing_threshold
								# ~ print("---motor GO AHEAD---")
								self.motor_control(40+60*go_ahead_gain,40+60*go_ahead_gain,0.5)
							
							
							else:
								turn_gain = 5*((self.closing_threshold + self.CLOSING_RANGE_THRE)/(distance_of_marker))**2
								# ~ print("---左に曲がります---")
								self.motor_control(50,50 + turn_gain,0.5)
								
								

						elif distance_of_marker >= self.closing_threshold:
							if tvec[0] >= 0.02:
								# ~ print("---時計周り---")
								self.motor_control(65,-65,0.5)
						
							elif tvec[0] <= -0.05:
								# ~ print("---反時計周り---")
								self.motor_control(-65,65,0.5)
							
							else:
								print("'\033[32m'---perfect REACHED---'\033[0m'")
								time.sleep(1)
								self.writeMissionlog(3)
								self.releasing_state = 2

						
						elif self.closing_threshold >= distance_of_marker >= self.closing_threshold - self.CLOSING_RANGE_THRE:
							if tvec[0] >= 0.02:
								# ~ print("---back 時計周り---")
								self.motor_control(-55,-75,0.5)
						
							elif tvec[0] <= -0.05:
								# ~ print("---back 反時計周り---")
								self.motor_control(-75,-65,0.5)
							
							else:
								# ~ print("---back---")
								self.motor_control(-65,-65,0.5)
						
						elif distance_of_marker <= self.closing_threshold - self.CLOSING_RANGE_THRE:
							self.control_log1 = "avoiding"
							if -50 <= angle_of_marker <= 0: #ARマーカがやや左から正面にある場合
								# ~ print("時計周り")
								self.motor_control(70,-70,0.3)
								self.yunosu_pos = "Left"
								self.last_pos = "Plan_B"
							
							
							elif 0 <= angle_of_marker <= 50: #ARマーカがやや右から正面にある場合
								# ~ print("反時計周り")
								self.motor_control(-70,70,0.3)
								self.yunosu_pos = "Right"
								self.last_pos = "Plan_B"
							
							
						self.distance, self.angle = self.ar.Correct(tvec,self.VEC_GOAL)
						polar_exchange = self.ar.polar_change(tvec)


			elif self.last_pos == "Plan_A" :#and not find_marker: #ARマーカを認識していない時，認識するまでその場回転
			#self.lost_marker_cnt+=1

			#if self.lost_marker_cnt > 10: # kore iru?
				if cX:
					# ~ print("==========================\n==========================\n")
					self.cam_pint = 10.5
					while self.cam_pint > 3.0: #pint change start
						if ids is None:
							self.cam_pint -= 0.5
							print("pint:",self.cam_pint)
							self.picam2.set_controls({"AfMode":0,"LensPosition":self.cam_pint})
							frame = self.picam2.capture_array()
							gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
							corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary)
						else:
							break
					print("===============\nyunosuke\n==============")
					if self.cam_pint <= 3.5:
						print("===============\nyunosuke\n==============")
						self.control_log1 = "closing"
						x,y = width-cY,cX
						self.cam_pint = 5.5 #default pint
						self.picam2.set_controls({"AfMode":0,"LensPosition":self.cam_pint})
						if x < width/2-100:
							# ~ print(f"color:ARマーカー探してます(LEFT) (x={x})")
							self.motor_control(-60,80,0.5)
						elif x > width/2+100:
							# ~ print(f"color:ARマーカー探してます(RIGHT) (x={x})")
							self.motor_control(80,-60,0.5)
						else:
							# ~ print(f"color:ARマーカー探してます(GO) (x={x})")
							self.motor_control(60,60,0.5)
					else:
						self.control_log1 = "detect AR"
							
				elif self.yunosu_pos == "Left":
					# ~ print("ARマーカー探してます(LEFT)")
					self.control_log1 = "explore"
					self.motor_control(-70,70,0.4)
					print("???????????????????????")
				elif self.yunosu_pos == "Right":
					# ~ print("ARマーカー探してます(RIGHT)")
					self.motor_control(70,-70,0.4)
					self.control_log1 = "explore"
					print("!!!!!!!!!!!!!!!!!!!!!!!")
				   

			elif self.last_pos == "Plan_B":
				self.lost_marker_cnt+=1
				# ~ print("lost marker cnt +1")
				self.control_log1 = "avoiding"
				if self.lost_marker_cnt > 10:
					if self.yunosu_pos == "Left":
						gain1 = 30
						gain2 = 0
					else:
						gain1 = 0
						gain2 = 30
						
					# ~ print("Plan_B now")
					self.motor_control(70+gain1,70+gain2,2.5 + self.k)
					self.last_pos = "Plan_A"
					self.k += 1
					# ~ print(self.k)
			    
    
		elif self.releasing_state == 2:
			"""
				微調整ステート
			"""
			print("'\033[44m'","5-2.moving_release_position", self.justAngle,'\033[0m')
			self.control_log1 = "adjust angle"
			
			self.cameraCount += 1
			self.frame = self.picam2.capture_array()
			self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
			cv2.imwrite(self.results_img_dir+f'/{self.cameraCount}.jpg',self.frame2)
			self.height = self.frame2.shape[0]
			self.width = self.frame2.shape[1]
			self.gray = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY) # グレースケールに変換
			self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.gray, self.dictionary)
			self.cam_pint = 10.5
			while self.cam_pint > 3.0: #pint change start
				# ~ print("pint:",self.cam_pint)
				if self.ids is None:
					self.flag_AR = False
					self.cam_pint -= 0.5
					# ~ print("pint:",self.cam_pint)
					self.picam2.set_controls({"AfMode":0,"LensPosition":self.cam_pint})
					self.frame = self.picam2.capture_array()
					self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
					self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.gray, self.dictionary)
					
				
				else:
					self.flag_AR = True
					break
					
			if self.ids is not None:
				if self.last_marker_num in self.ids:
					self.ids = [self.last_marker_num]
				else:
					self.ids = [self.ids[0]]
					self.last_marker_num = self.ids[0]
				for i in range(len(self.ids)):
						if self.ids[i] in [1,2,3,4,5,6]:
							self.flag_AR = True
							rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners[i], self.marker_length, self.camera_matrix, self.distortion_coeff)
							tvec = np.squeeze(tvec)
							self.justAngle = self.adjust_angle(tvec)
				

			if self.justAngle:
				
				print("\033[32m","just angle!!!!!!!!!!!!",self.nowangle,"\033[0m")
				time.sleep(5)
				self.writeMissionlog(4)
				self.frame = self.picam2.capture_array()
				self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
				cv2.imwrite(self.results_img_dir+f'/mission_{self.cameraCount}.jpg',self.frame2)
				self.releasing_state = 3
			pass
    
		elif self.releasing_state == 3:
			"""
				物資モジュール投射

			"""
			print("'\033[44m'","5-3.moving_release_position",'\033[0m')
			self.control_log1 = "releasing"
			self.control_log2 = f"pin{ct.const.SEPARATION_MOD2}:HIGH"
			self.separation(ct.const.SEPARATION_MOD2)
			print("ct.const.SEPARATION_MOD2 no settei ga hituyou")
			self.writeMissionlog(5)
			time.sleep(5)
			self.state = 6
			pass
		

	def motor_control(self,m1,m2,t):
		# m1:右モーターの速度
		# m2:左モーターの速度
		# time:モーターを動かす時間
		
		if m1 == m2 and m1 > 0:
			self.control_log2  = "go straight"
		elif m1 == m2 and m1 < 0:
			self.control_log2  = "go back"
		elif m1 > m2 and m2 > 0:
			self.control_log2  = "go right"
		elif m1 < m2 and m1 > 0:
			self.control_log2  = "go left" 
		elif m1 == -m2 and m1 > 0:
			self.control_log2  = "turn right"
		elif m1 == -m2 and m1 < 0:
			self.control_log2  = "turn left" 
		elif m1 > m2 and m1 < 0:
			self.control_log2  = "go back right"
		elif m1 < m2 and m2 < 0:
			self.control_log2  = "go back left" 
		else:
			self.control_log2  = "except" 
		
		self.control_log_rv = m2
		self.control_log_lv = m1
		self.rv, self.lv = m1, m2
		
		if m1>=0:
			self.motor1.go(m1)
		else:
			m1 = abs(m1)
			self.motor1.back(m1)
		if m2>=0:
			self.motor2.go(m2)
		else:
			m2 = abs(m2)
			self.motor2.back(m2)
		time.sleep(t)
		self.motor1.stop()
		self.motor2.stop()
		

	def judgement(self): # state = 6
		"""
			物資モジュール確認

		"""
		if self.closing_state == 1:
			print("'\033[44m'","6-1.Go to judgement",'\033[0m')
			self.cameraCount += 1
			self.frame = self.picam2.capture_array()
			self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
			cv2.imwrite(self.results_img_dir+f'/{self.cameraCount}.jpg',self.frame2)
			height = self.frame2.shape[0]
			width = self.frame2.shape[1]
			gray = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY) # グレースケールに変換
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary) # ARマーカーの検出   

			# オレンジ色のマスクを作成
			mask_blue = self.color.mask_color(self.frame,ct.const.LOWER_BLUE,ct.const.UPPER_BLUE)
			# 輪郭を抽出して最大の面積を算出し、線で囲む
			mask_blue,cX,cY,max_contour_area = self.color.detect_color(mask_blue,ct.const.MAX_CONTOUR_THRESHOLD)
			#print("cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
			if cX:
				self.flag_COLOR = True

			if ids is not None:
				if self.last_marker_num in ids:
					ids = [self.last_marker_num]
				else:
					ids = [ids[0]]
					self.last_marker_num = ids[0]
				for i in range(len(ids)):
					if ids[i] in [1,2,3,4,5,6]:
						rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.distortion_coeff)
						tvec = np.squeeze(tvec)
						rvec = np.squeeze(rvec)
						self.flag_AR = True
						# 回転ベクトルからrodoriguesへ変換
						rvec_matrix = cv2.Rodrigues(rvec)
						rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
						transpose_tvec = tvec[np.newaxis, :].T # 並進ベクトルの転置
						proj_matrix = np.hstack((rvec_matrix, transpose_tvec)) # 合成
						euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # オイラー角への変換[deg]
						self.prev = list(self.prev)
						self.lost_marker_cnt = 0

						self.reject_count = 0
						# ~ print("x : " + str(tvec[0]))
						# ~ print("y : " + str(tvec[1]))
						# ~ print("z : " + str(tvec[2]))
						tvec[0] = tvec[0]
						polar_exchange = self.ar.polar_change(tvec)
						# ~ print(f"yunosu_function_{ids[i]}:",polar_exchange)
						distance_of_marker = polar_exchange[0] #r
						self.distanceAR = distance_of_marker
						angle_of_marker = polar_exchange[1] #theta
						# ~ print("======",distance_of_marker)
						self.control_log1 = "closing"
						if distance_of_marker >= self.closing_threshold_2 + self.CLOSING_RANGE_THRE_2:
							if tvec[0] >= 0.05:
								# ~ print("---右に曲がります---")
								self.motor_control(80,65,0.4)
							
								
							elif 0.05 > tvec[0] > -0.05:
								go_ahead_gain = (distance_of_marker-self.closing_threshold_2) / self.closing_threshold_2
								# ~ print("---motor GO AHEAD---")
								self.motor_control(50+50*go_ahead_gain,50+0*go_ahead_gain,0.4)
							
							
							else:
								# ~ print("---左に曲がります---")
								self.motor_control(65,80,0.4)
								
								

						elif distance_of_marker >= self.closing_threshold_2:
							if tvec[0] >= 0.03:
								# ~ print("---時計周り---")
								self.motor_control(65,-65,0.4)
						
							elif tvec[0] <= -0.03:
								# ~ print("---反時計周り---")
								self.motor_control(-65,65,0.4)
							
							else:
								print("'\033[32m'---perfect REACHED 2---'\033[0m'")
								time.sleep(1)
								self.closing_state = 2
								# ~ print("closing:",self.closing_state)
								
								

						
						elif self.closing_threshold_2 >= distance_of_marker >= self.closing_threshold_2 - self.CLOSING_RANGE_THRE_2:
							if tvec[0] >= 0.03:
								# ~ print("---back 時計周り---")
								self.motor_control(-65,-75,0.4)
						
							elif tvec[0] <= -0.03:
								# ~ print("---back 反時計周り---")
								self.motor_control(-75,-65,0.4)
							
							else:
								# ~ print("---back---")
								self.motor_control(-65,-65,0.4)
						
						elif distance_of_marker <= self.closing_threshold_2 - self.CLOSING_RANGE_THRE_2:
							self.control_log1 = "avoiding"
							if -50 <= angle_of_marker <= 0: #ARマーカがやや左から正面にある場合
								# ~ print("時計周り")
								self.motor_control(70,-70,0.4)
								yunosu_pos = "Left"
								last_pos = "Plan_B"
							
							
							elif 0 <= angle_of_marker <= 50: #ARマーカがやや右から正面にある場合
								# ~ print("反時計周り")
								self.motor_control(-70,70,0.4)
								yunosu_pos = "Right"
								last_pos = "Plan_B"


						distance, angle = self.ar.Correct(tvec,self.VEC_GOAL)
						polar_exchange = self.ar.polar_change(tvec)
							


			elif self.last_pos == "Plan_A" :#and not find_marker: #ARマーカを認識していない時，認識するまでその場回転
				self.lost_marker_cnt+=1
			# ~ if self.lost_marker_cnt > 10: # kore iru?")
				if cX:
					print("==========================\n==========================\n")
					self.cam_pint = 10.5
					self.flag_COLOR = True
					while self.cam_pint > 3.0: #pint change start
						if ids is None:
							self.flag_AR = False
							self.cam_pint -= 0.5
							print("pint:",self.cam_pint)
							self.picam2.set_controls({"AfMode":0,"LensPosition":self.cam_pint})
							frame = self.picam2.capture_array()
							gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
							corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary)
						else:
							self.flag_AR = True
							break
					if self.cam_pint <= 3.5:
						self.control_log1 = "closing"
						x,y = width-cY,cX
						
						self.cam_pint = 5.5 #default pint
						self.picam2.set_controls({"AfMode":0,"LensPosition":self.cam_pint})
						if x < width/2-100:
							# ~ print(f"color:ARマーカー探してます(LEFT) (x={x})")
							self.motor_control(-60,80,0.5)
						elif x > width/2+100:
							# ~ print(f"color:ARマーカー探してます(RIGHT) (x={x})")
							self.motor_control(80,-60,0.5)
						else:
							# ~ print(f"color:ARマーカー探してます(GO) (x={x})")
							self.motor_control(70,70,0.5)
					else:
						self.control_log1 = "detect AR"
						
				elif self.yunosu_pos == "Left":
					self.control_log1 = "explore"
					# ~ print("ARマーカー探してます(LEFT)")
					self.motor_control(-70,70,0.5)
				elif self.yunosu_pos == "Right":
					self.control_log1 = "explore"
					# ~ print("ARマーカー探してます(RIGHT)")
					self.motor_control(70,-70,0.5)
				

			elif self.last_pos == "Plan_B":
				self.lost_marker_cnt+=1
				# ~ print("lost marker cnt +1")
				self.control_log1 = "avoiding"
				if self.lost_marker_cnt > 10:
					if self.yunosu_pos == "Left":
						gain1 = 30
						gain2 = 0
					else:
						gain1 = 0
						gain2 = 30
						
						print("Plan_B now")
						self.motor_control(70+gain1,70+gain2,2.5 + self.k)
						self.last_pos = "Plan_A"
						self.k += 1
						print(self.k)
		elif self.closing_state == 2:
			"""
				撮影・評価

			"""
			print("'\033[44m'","6-2.judgement",'\033[0m')
			self.cameraCount += 1
			self.frame = self.picam2.capture_array()
			self.frame2 = cv2.rotate(self.frame ,cv2.ROTATE_90_CLOCKWISE)
			height = self.frame2.shape[0]
			width = self.frame2.shape[1]
			
			if self.judge_cnt < 5:
				if self.frame2 is not None:
					# オレンジ色の検出
					lower_blue = np.array([90, 40, 26])
					upper_blue = np.array([135, 250, 250])
					lower_red = np.array([165, 15, 10])
					upper_red = np.array([179, 250, 250])

					hsv = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2HSV)
					mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
					mask_red = cv2.inRange(hsv, lower_red, upper_red)
					# 形態学的処理（膨張と収縮）を追加
					kernel = np.ones((10,10), np.uint8)
					mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
					mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
					mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
					mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

					# 輪郭を抽出して最大の面積を算出し、線で囲む
					contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
					if contours_blue:
						max_contour = max(contours_blue, key=cv2.contourArea)
						hull = cv2.convexHull(max_contour)
						cv2.drawContours(self.frame2, [hull], -1, (0, 0, 255), 3)

					contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
					if contours_red:
						max_contour = max(contours_red, key=cv2.contourArea)
						if cv2.contourArea(max_contour) > ct.const.MAX_CONTOUR_THRESHOLD//10:  # 面積が1000より大きい場合のみ描画
							cv2.drawContours(self.frame2, [max_contour], -1, (0, 255, 0), 3)
							M = cv2.moments(max_contour)
							if M["m00"] != 0:
								cX = int(M["m10"] / M["m00"])
								cY = int(M["m01"] / M["m00"])

								try:
									if cv2.pointPolygonTest(hull, (cX, cY), False) >= 0:
										print("\033[33m~~~ INSIDE ~~~\033[0m")
									else:
										print("\033[33m~~~ OUTSIDE ~~~\033[0m")
								except :
									pass
				self.motor_control(-70,-70,0.5)
				time.sleep(1)
				self.judge_cnt += 1
			else:
				time.sleep(3)
				self.state = 7
			
			cv2.imwrite(self.results_img_dir+f'/mission_{self.cameraCount}.jpg',self.frame2)
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
				self.rv = ct.const.STUCK_MOTOR_VREF
				self.lv = -ct.const.STUCK_MOTOR_VREF
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
		if self.sepa_mode:
			print("\n\n\==================n\nSeparation done\n\n==================\n\n")
			GPIO.setup(pin,GPIO.OUT) #焼き切り用のピンの設定tv 
			GPIO.output(pin,0) #焼き切りが危ないのでlowにしておく
			GPIO.output(pin,1) #電圧をHIGHにして焼き切りを行う
			time.sleep(6) #継続時間を指定
			GPIO.output(pin,0) #電圧をLOWにして焼き切りを終了する
		else:
			print("\n\n\==================n\nSeparation pass\n\n==================\n\n")
		
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
			    self.motor1.go(70)
			    self.motor2.go(100)
			
		    elif self.arg_diff > 180 and self.arg_diff < 340:
			    self.motor1.go(100)
			    self.motor2.go(70)
		    
		    else:
			    self.motor1.go(100)
			    self.motor2.go(100)

	def finish(self): # state = 8
		if self.finishTime == 0:
		    self.finishTime = time.time()
		    print("\n",self.startTime)
		    print("\nFinished\n")
		    self.writeMissionlog()
		    self.motor1.stop()
		    self.motor2.stop()
		    # ~ GPIO.output(ct.const.SEPARATION_PARA,0) #焼き切りが危ないのでlowにしておく
		    # ~ GPIO.output(ct.const.SEPARATION_MOD1,0) #焼き切りが危ないのでlowにしておく
		    # ~ GPIO.output(ct.const.SEPARATION_MOD2,0) #焼き切りが危ないのでlowにしておく
		    self.RED_LED.led_off()
		    self.BLUE_LED.led_off()
		    self.GREEN_LED.led_off()
		    self.picam2.stop()
		    time.sleep(0.5)
		    cv2.destroyAllWindows()
		    sys.exit()

	def adjust_angle(self, tvec):
		print(f"\033[33m", f"adjust angle : tvec = {tvec}", "\033[0m")	

		self.distanceAR = (tvec[0]**2+tvec[1]**2+tvec[2]**2)**(1/2)
		if tvec[0] > 0.015:
			if self.nowangle >= 110:
				print("=@=@=servo: "+str(self.nowangle),">110")
				return False
			else:
				self.nowangle += 3
				self.servo.go_deg(self.nowangle)
				print("=@=@=servo: "+str(self.nowangle),"B")
		elif tvec[0] < -0.015:
			if self.nowangle <= 60:
				print("=@=@=servo: "+str(self.nowangle),"<=60")
				return False
			else:
				self.nowangle -= 3
				self.servo.go_deg(self.nowangle)
				print("=@=@=servo: "+str(self.nowangle),"A")
		else:
			return True

	def keyboardinterrupt(self): #キーボードインタラプト入れた場合に発動する関数
		self.motor1.stop()
		self.motor2.stop()
		# ~ GPIO.output(ct.const.SEPARATION_PARA,0) #焼き切りが危ないのでlowにしておく
		# ~ GPIO.output(ct.const.SEPARATION_MOD1,0) #焼き切りが危ないのでlowにしておく
		# ~ GPIO.output(ct.const.SEPARATION_MOD2,0) #焼き切りが危ないのでlowにしておく
		self.RED_LED.led_off()
		self.BLUE_LED.led_off()
		self.GREEN_LED.led_off()
		self.pc2.stop()
		time.sleep(0.5)
		cv2.destroyAllWindows()
		GPIO.cleanup()
		pass
		
	def motor_test(self):
		try:
		    print("motor run") 
		    self.motor1.go(70)
		    self.motor2.go(70)
		#     Motor1.back(80)
		#     Motor2.back(80)
		#     time.sleep(0.5)
		 #   Motor2.back(80)
		    #Motor2.back(90)
		#     time.sleep(1.08)
		    time.sleep(1.5)

		    #Motor.back(100)
		    #time.sleep(3)
		    print("motor stop")
		    self.motor1.stop()
		    self.motor2.stop()
		    time.sleep(1)
		except KeyboardInterrupt:
		    self.motor1.stop()
		    self.motor2.stop()
		    time.sleep(1)
		    GPIO.cleanup()
    
	
			
	
	
		
