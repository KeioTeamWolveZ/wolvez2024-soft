import motor_pico as motor
import time

servo = motor.motor()
servo.set_id(2)


servo.go(180)
print("180")
time.sleep(5) # 5 seconds stop
servo.go(0)
print("0")
time.sleep(5) # 5 seconds stop
