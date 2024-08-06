import Adafruit_BMP.BMP085 as BMP085
import time
import numpy as np

sensor = BMP085.BMP085()
log_file_path = 'pressure_threshold.txt'

while True:
	temperature = sensor.read_temperature()
	pressure = sensor.read_pressure()
	altitude = sensor.read_altitude()
	sealevel_pressure = sensor.read_sealevel_pressure()
	with open(log_file_path, 'w') as file:
		file.write(f'{pressure:0.2f}')
