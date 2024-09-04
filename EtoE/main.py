import pigpio
from cansat2 import Cansat 
import time 
import RPi.GPIO as GPIO
import os



def load_values_from_file(filename):
    """Load HSV values from a text file."""
    if os.path.exists(filename):
        with open(filename, "r") as file:
            lines = file.readlines()
            print(lines)
            last_state = float(lines[0])
            return last_state
    else:
        # Default values if file doesn't exist
        return float(999)


filename = "state_manager.txt"

last_state = load_values_from_file(filename)
start_state = 0
end_state = 8

sepa_mode  = False
error_cnt = 0


if last_state != 999:
	start_state = last_state
	

try:	
	cansat  = Cansat(start_state,sepa_mode)
	try:
		cansat.sensor_setup()
	except:
		print("setup ERROR")
	while True:
		
		cansat.sensor()
		time.sleep(0.03)
		try:
			cansat.sequence()
			if cansat.state_error > 5 and cansat.state < 7:
				cansat.state = 7
			
		except Exception as e:
			cansat.state_error+=1
			print("\033[33m")
			print(cansat.state_error)
			print("Exception")
			print(e,"\033[0m")
			pass
		if cansat.state > end_state:
			print("Finished")
		time.sleep(0.3)
		print("*****",cansat.state)
		with open(filename, "w") as file:
			file.write(f"{cansat.state}")
		
		
except KeyboardInterrupt:
    print("Finished")
    print(cansat.startTime)
    print(cansat.keyboardinterrupt)
    GPIO.cleanup()

finally:
	GPIO.cleanup()
	with open(filename, "w") as file:
		file.write("999")
	print("save state manager as 999")
	cansat.motor1.stop()
	cansat.motor2.stop()
	
