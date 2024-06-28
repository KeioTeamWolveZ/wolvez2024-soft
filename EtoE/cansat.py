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
import constant as ct

from Wolvez2024_now.led import led
from Wolvez2024_now.gps import GPS
from Wolvez2024_now.bno055 import BNO055
from Wolvez2024_now.bmp import BMP


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
		# GPIO設定
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #GPIOの設定
		GPIO.setup(ct.const.FLIGHTPIN_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP) #フライトピン用。プルアップを有効化
		
		
		self.timer = 0
		self.state = state
		self.last_state = 0
		self.time = 0
		self.startTime_time=time.time()
		self.startTime = str(datetime.now())[:19].replace(" ","_").replace(":","-")
		print(ct.const.RIGHT_MOTOR_IN1_PIN)
		self.RED_LED = led(ct.const.RED_LED_PIN)
		self.BLUE_LED = led(ct.const.BLUE_LED_PIN)
		self.GREEN_LED = led(ct.const.GREEN_LED_PIN)

		self.gps = GPS()
		self.bno055 = BNO055()
		self.bmp = BMP()
		
		
		
		
		self.preparingTime = 0
		self.flyingTime = 0
		
		self.gpscount = 0
		self.countFlyLoop = 0
		
		self.startgps_lon = []
		self.startgps_lat = []
		
		
		self.temp = 0
		self.pressure = 0
		self.altitude = 0
		self.ax= 0
		self.ay= 0
		self.az= 0
		self.gx= 0
		self.gy= 0
		self.gz= 0
		self.ex= 0
		self.lat = 0
		self.lon = 0
		self.mkdir()
		
	def mkdir(self):
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
		datalog = {
					"state":str(self.state),
					"time":str(self.gps.Time),
					"Lat":str(self.gps.Lat),
					"Lon":str(self.gps.Lon),
					"ax":str(self.ax),
					"ay":str(self.ay),
					"az":str(self.az),
					"gx":str(self.gx),
					"gy":str(self.gy),
					"gz":str(self.gz),
					"temp":str(self.temp),
					"pressure":str(self.pressure),
					"altitude":str(self.altitude)
					}
        
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
		# ミッションシーケンスを管理する
		# main.pyで毎周期実行される。
		
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

		# ~ if GPIO.input(ct.const.FLIGHTPIN_PIN) == GPIO.HIGH: #highかどうか＝フライトピンが外れているかチェック
			# ~ self.countFlyLoop+=1
			# ~ if self.countFlyLoop > ct.const.FLYING_FLIGHTPIN_COUNT_THRE: #一定時間HIGHだったらステート移行
				# ~ self.state = 2
				# ~ self.laststate = 2       
		# ~ else:
			# ~ self.countFlyLoop = 0 #何故かLOWだったときカウントをリセット
		
		time.sleep(0.2)
		self.BLUE_LED.led_off()
		
		print("\033[32m",1,"\033[0m")
    
	def landing(self):
		pass
	def para_escaping(self):
		pass
	def first_releasing(self):
		pass
	def moving_release_position(self):
		pass
	def judgement(self):
		pass
	def finish(self):
		pass
	
	
	def keyboardinterrupt(self): #キーボードインタラプト入れた場合に発動する関数
		self.RED_LED.led_clean()
		pass
	
			
	
	
		
