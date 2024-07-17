import RPi.GPIO as GPIO
import time

#pin1 = 25 right
#pin2 = 24 center
#pin3 = 8 left

pin1 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin1,GPIO.OUT) #焼き切り用のピンの設定tv 
GPIO.output(pin1,0) #焼き切りが危ないのでlowにしておく

try:
    print("Separating...")
    GPIO.output(pin1,1) #電圧をHIGHにして焼き切りを行う
    time.sleep(7) #継続時間を指定
    GPIO.output(pin1,0) #電圧をLOWにして焼き切りを終了する
    GPIO.cleanup()
    
    print("Separation done")

except KeyboardInterrupt:
    print("Separation Canceled")
    GPIO.output(pin1,0) #電圧をLOWにして焼き切りを終了
    GPIO.cleanup()
