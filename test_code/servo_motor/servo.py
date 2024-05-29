##################################################################
### このコードはCanSat機体とは異なるピン配置を使っています。            ###
### コードを回す際は注意してください。                               ###
##################################################################

#必要なモジュールをインポート
import RPi.GPIO as GPIO             #GPIO用のモジュールをインポート
import time                         #時間制御用のモジュールをインポート
import sys                          #sysモジュールをインポート

print("\033[32m"+"このコードはCanSat機体とは異なるピン配置を使っています。\
                    \nコードを回す際は注意してください。"+"\033[0m")
ok = input("OK or Stop : ")
if ok == "OK":
    #ポート番号の定義
    Servo_pin = 23                      #変数"Servo_pin"に18を格納

    #GPIOの設定
    GPIO.setmode(GPIO.BCM)              #GPIOのモードを"GPIO.BCM"に設定
    GPIO.setup(Servo_pin, GPIO.OUT)     #GPIO18を出力モードに設定

    #PWMの設定
    #サーボモータSG90の周波数は50[Hz]
    Servo = GPIO.PWM(Servo_pin, 50)     #GPIO.PWM(ポート番号, 周波数[Hz])

    Servo.start(0)                      #Servo.start(デューティ比[0-100%])

    #角度からデューティ比を求める関数
    def servo_angle(angle):
        duty = 2.5 + (12.0 - 2.5) * (angle + 90) / 180   #角度からデューティ比を求める
        Servo.ChangeDutyCycle(duty)     #デューティ比を変更
        time.sleep(0.3)                 #0.3秒間待つ

    #while文で無限ループ
    #サーボモータの角度をデューティ比で制御
    #Servo.ChangeDutyCycle(デューティ比[0-100%])
    while True:
        try:
            servo_angle(-90)               #サーボモータ -90°
            servo_angle(-60)               #サーボモータ -60°
            servo_angle(-30)               #サーボモータ -30°
            servo_angle(0)                 #サーボモータ  0°
            servo_angle(30)                #サーボモータ  30°
            servo_angle(60)                #サーボモータ  60°
            servo_angle(90)                #サーボモータ  90°
        except KeyboardInterrupt:          #Ctrl+Cキーが押された
            Servo.stop()                   #サーボモータをストップ
            GPIO.cleanup()                 #GPIOをクリーンアップ
            sys.exit()                     #プログラムを終了
else:
    print("でなおしてきなしゃーい")
