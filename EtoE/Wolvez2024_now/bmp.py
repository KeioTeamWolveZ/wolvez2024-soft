import Adafruit_BMP.BMP085 as BMP085
import time
import numpy as np

sensor = BMP085.BMP085()

class BMP(BMP085.BMP085):
	def __init__(self):
		self.sensor = BMP085.BMP085()
		
		
	def readBMP(self):
		self.temp = self.sensor.read_temperature()
		self.pressure = self.sensor.read_pressure()
		self.altitude = self.sensor.read_altitude()
		return self.temp, self.pressure, self.altitude
	
