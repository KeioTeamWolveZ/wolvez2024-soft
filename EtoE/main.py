import pigpio
from cansat import Cansat 
import time 
try:
	cansat  = Cansat(0)
	cansat.sensor_setup()
	while True:
		cansat.sensor()
		time.sleep(0.03)
		cansat .sequence()
		time.sleep(0.1)
except KeyboardInterrupt:
    print("Finished")
    print(cansat.startTime)
