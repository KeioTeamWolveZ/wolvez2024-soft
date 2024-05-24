"""
purpose:
    ARマーカから20cm離れたところまで移動する
    この時、極力機体はARマーカに対して正面を向くようにする

Plan:
    ARマーカから十分に距離が取れている時 - ➀
    取れていない時 - ➁
    で場合分け


input:
    (r,theta)

output:
    (Rl,Rr) = (左のモータの回転数,右のモータの回転数)    
"""
import numpy as np
import math
#import motor
#import RPi.GPIO as GPIO
#import time
#from test_code.AR_Planning.Ar_tools import Artools #importわけわからん

r = 0.50 #ARマーカまでの距離[m](インポートして1回だけ使う、定数)
theta = math.radians(30) #マーカが右なら正、左なら負(インポートして1回だけ使う、定数)
r2 = 10 #ARマーカとの距離(時変)　値超適当
theta2 = 10 #ARマーカとの角度(時変)　値超適当
R = 0.1 #車輪半径[m]
d = 0.15 #左右の車輪間の距離[m]

"""
Plan➀ 十分に距離が取れている時(r = 40を閾値とした)
"""

if r >= 0.40:
    move_distance = r - 0.20 #機体移動距離
    t = move_distance*10 #移動時間は距離に比例
    Rr = ((move_distance/(2*R))*(theta + (d/R)))/t #右モータの1秒あたりの回転数
    Rl = (move_distance/d - Rr)/t #左モータの1秒あたりの回転数
    print("左モータの回転数:",Rl,"右モータの回転数:",Rr,"時間:",t)
    #motor1.go(Rr)
    #motor2.go(Rl)
    if -3 < math.radians(theta2) < 3: #ARマーカとのなす角が小さくなったらストップして直進
        [Rr,Rl] = [0,0] #疑似停止
        [Rr,Rl] = [10,10] #疑似直進
        #Motor1.stop()
        #Motor2.stop()
        #time.sleep(1)
        #Motor1.go(50)
        #Motor2.go(50) #直進
        if 0.195< r <0.205: #rが20センチ付近になったらストップ
            [Rr,Rl] = [0,0] #疑似停止
            #Motor1.stop()
            #Motor2.stop()

"""
memo:
    Rl,Rrとモータに入力する値の関係が分からない
     >実機を動かして値を入手する方向性？
    
    elseどうしよう

"""



