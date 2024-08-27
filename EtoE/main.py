import pigpio
from cansat2 import Cansat 
import time 
import RPi.GPIO as GPIO

start_state = 0
end_state = 8

sepa_mode  = False
error_cnt = 0

try:	
	cansat  = Cansat(start_state,sepa_mode)
	try:
		cansat.sensor_setup()
	except:
		print("setup ERROR")
	while True:
		
		cansat.sensor()
		time.sleep(0.03)
		cansat .sequence()
		if cansat.state > end_state:
			print("Finished")
		time.sleep(0.3)
		print("*****",cansat.state)
		
		
except KeyboardInterrupt:
    print("Finished")
    print(cansat.startTime)
    print(cansat.keyboardinterrupt)
    GPIO.cleanup()

