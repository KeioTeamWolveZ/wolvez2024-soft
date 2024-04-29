import math 
import numpy as np
from scipy.optimize import fsolve
from polar import polar_change

"""
input:
    各パラメータ
output:
    落下位置[r,θ]
"""

m = 0.01 #モジュールの質量 [kg]
g = 9.81 #重力加速度 [m/(s^2)]
k = 10 #ばね定数 [N/m]
V_0 = 1 #初速度 [m/s]
theta_initial = math.radians(30) #放出機構の角度 [rad]

#カメラ座標原点に対する放出項の位置 [m]
x_0 = 0
y_0 = -0.1 #ｙ軸下向きを正としているため、負値
z_0 = 0

#カメラの傾き
alpha = math.radians(0) #x軸周り [rad]
beta = math.radians(0) #z軸周り [rad]

h = 0.1 #カメラ座標原点の高さ [m]

#放出から落下までの時間の導出
def equation(t):
    return m/k * (V_0*math.sin(theta_initial)-m*g/k*math.cos(alpha)*math.cos(beta))\
    *(1-math.exp(-k/m*t)) + (m*g/k*math.cos(alpha)*math.cos(beta))*t + y_0 - h

t_initial = 0 #tの初期値

solution = fsolve(equation, t_initial)
t = solution[0]

print("t:",t)

x = m/k * (-m*g/k*math.cos(alpha)*math.sin(beta))*(1-math.exp(-k/m*t))\
    + (m*g/k*math.cos(alpha)*math.sin(beta))*t + x_0
y = m/k * (V_0*math.sin(theta_initial)-m*g/k*math.cos(alpha)*math.cos(beta))\
    *(1-math.exp(-k/m*t)) + (m*g/k*math.cos(alpha)*math.cos(beta))*t + y_0
z = m/k  * (V_0*math.cos(theta_initial)-m*g/k*math.sin(alpha))\
    *(1-math.exp(-k/m*t)) + (-m*g/k*math.sin(alpha))*t +z_0

Theoretical_position = [x, y, z]  #落下位置[x,y,z]

print("落下位置[x,y,z] = ",Theoretical_position)

polar_position = polar_change(Theoretical_position) #落下位置[r,θ,φ]
del polar_position[2] #φの削除

print("落下位置[r, θ] = ",polar_position)


"""
メモ：
    tが負になるのがおかしい。。
    ばね定数を小さくしないと、指数部分が大きくなりエラーが発生
    ばね定数k=10とした時、放出から落下までの時間が20秒とめちゃくちゃ長い。。
    座標の符号に関しては後からどうにかなりそう(?)
"""