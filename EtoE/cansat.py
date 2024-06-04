import pigpio
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
		self.timer = 0
		self.state = state
		self.time = 0
		print(ct.const.RIGHT_MOTOR_IN1_PIN)
	
	def mkdir(self):
		pass
		
	def mkfile(self):
		pass
		
	def mvfile(self):
		pass
		
	def writeData(self):
		log_data = {
					"state":self.state,
					"time":self.time
					}
		print(log_data)
		
		# ~ with open(f'results/{self.startTime}/control_result.txt',"a")  as test: # [mode] x:ファイルの新規作成、r:ファイルの読み込み、w:ファイルへの書き込み、a:ファイルへの追記
            # ~ test.write(datalog + '\n')
            
	def writeMissionlog(self):
		pass
		
	# =================== mission sequence ===================
	def sequence(self):
		# ミッションシーケンスを管理する
		# main.pyで毎周期実行される。
		
		if self.state == 0:
			print("\033[32m",0,"\033[0m")
		elif self.state == 1:
			print("\033[32m",1,"\033[0m")
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
		pass
	
	def preparing(self):
		pass
	def flying(self):
		pass
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
	
			
	
	
		
