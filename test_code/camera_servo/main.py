# -*- coding: utf-8 -*-
import sys
import time
import cv2
import RPi.GPIO as GPIO
import numpy as np

import motor_pico as motor
from libcam_module import Picam

loop_count = 0
GPIO.setwarnings(False)

if __name__ == '__main__':
    # サーボモータの設定
    servo = motor.motor()
    servo.set_id(2)
    pc2 = Picam()
    # 撮影をしながらサーボを動かす
    while True:
        try:
            # カメラ撮影
            img = pc2.capture(1)
            pc2.show(img)
            # サーボモータを動かす
            print(loop_count)
            if np.mod(loop_count,3) == 1:
                print("here")
                servo.go(180)
                time.sleep(2) # 5 seconds stop
            elif np.mod(loop_count,3) == 2:
                servo.go(0)
                time.sleep(2) # 5 seconds stop
            loop_count += 1

            # 画像を表示している場合はescキーで終了できる
            key = cv2.waitKey(1)
            # If you push "esc-key", this roop is finished.
            if key == 27:
                GPIO.cleanup()
                pc2.stop()
                sys.exit()
                # cv2.imwrite("test_cv2.jpg", im)
                break

            time.sleep(0.5)
        except KeyboardInterrupt:
            GPIO.cleanup()
            pc2.stop()
            sys.exit()
