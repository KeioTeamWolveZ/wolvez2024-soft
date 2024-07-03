import motor_pico as motor
import RPi.GPIO as GPIO
import time 
import numpy as np
from datetime import datetime
import lora_send_onlyOnce

lora_device = "/dev/ttyAMA1"
channel = 15
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.IN)
lr_send = lora_send_onlyOnce.LoraSendClass(lora_device, channel)

Motor1 = motor.motor(6,5,13)
Motor2 = motor.motor(20,16,12,-1)

# 時間を計測
start = time.time()

for i in np.arange(0,30*60):
    # 現在の経過時間を表示
    elapsed_time = time.time() - start
    print(f"message sent as {elapsed_time}")
    lr_send.lora_send(f"elapsed:{elapsed_time:.1f}")
    time.sleep(1)


try:
    print("motor run")
    Motor1.go(70)
    Motor2.go(70)
    i=0
    while i < 60*120/5:
        i+=1
        elapsed_time = time.time() - start
        print(f"message sent as {elapsed_time}")
        lr_send.lora_send(f"elapsed:{elapsed_time:.1f}")
        time.sleep(5)


    time.sleep(60*120)

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