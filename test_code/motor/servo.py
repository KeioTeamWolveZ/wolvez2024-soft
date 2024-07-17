import motor_pico as motor
import time

servo = motor.motor()
servo.set_id(2)


servo.go_deg(110)
print("180")
time.sleep(2) # 5 seconds stop
servo.go_deg(50)
print("0")
time.sleep(2) # 5 seconds stop
servo.go_deg(90)
print("0")
time.sleep(2) # 5 seconds stop


