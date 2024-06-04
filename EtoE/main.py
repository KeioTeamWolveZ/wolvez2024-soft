import pigpio
from cansat import Cansat 
import time 
try:
	cst = Cansat(0)
	while True:
		print("=")
		cst.sequence()
		time.sleep(0.1)
except KeyboardInterrupt:
    print("Finished")
    print(cansat.startTime)
