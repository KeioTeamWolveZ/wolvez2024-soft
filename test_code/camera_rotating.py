# -*- coding: utf-8 -*-
import sys
import time
import cv2
import RPi.GPIO as GPIO

import motor_pico as motor
from libcam_module import Picam


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
            servo.go(180)
            time.sleep(5) # 5 seconds stop
            servo.go(0)
            time.sleep(5) # 5 seconds stop

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