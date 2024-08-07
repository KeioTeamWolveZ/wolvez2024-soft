import pigpio
from cansat import Cansat 
import time 
import RPi.GPIO as GPIO


start_state = 0

end_state = 8

try:
	cansat  = Cansat(0)
	cansat.sensor_setup()
	while True:
		cansat.sensor()
		time.sleep(0.03)
		cansat .sequence()
		if cansat.state > end_state:
			print("Finished")
		time.sleep(0.2)
		
except KeyboardInterrupt:
    print("Finished")
    print(cansat.startTime)
    print(cansat.keyboardinterrupt)
    GPIO.cleanup()
