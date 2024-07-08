# 2023年機体で使用すること

import motor_pico as motor 
import RPi.GPIO as GPIO
import time 


GPIO.setwarnings(False)
Motor1 = motor.motor(6,5,13)
Motor2 = motor.motor(20,16,12,-1)

try:
    print("motor run") 
    Motor1.go(50)
    Motor2.go(50)
#     Motor1.back(80)
#     Motor2.back(80)
#     time.sleep(0.5)
 #   Motor2.back(80)
    #Motor2.back(90)
#     time.sleep(1.08)
    time.sleep(1)

    #Motor.back(100)
    #time.sleep(3)
    print("motor stop")
    Motor1.stop()
    Motor2.stop()
    time.sleep(1)
except KeyboardInterrupt:
    Motor1.stop()
    Motor2.stop()
    time.sleep(1)
    GPIO.cleanup()
    

GPIO.cleanup()
