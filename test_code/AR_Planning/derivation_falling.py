import math 
import numpy as np
from scipy.optimize import fsolve
from Ar_tools import Artools
import time

"""
input:
    各パラメータ
output:
    落下位置[r,θ]
"""
st = time.time()

m = 0.01 #モジュールの質量 [kg]
g = 9.81 #重力加速度 [m/(s^2)]
k = 0.1 #空気抵抗係数 [kg/s]
V_0 = 1 #初速度 [m/s]
theta_initial = math.radians(30) #放出機構の角度 [rad]

#カメラ座標原点に対する放出項の位置 [m]
x_0 = 0
y_0 = -0.1 #ｙ軸下向きを正としているため、負値
z_0 = 0

U_x = 0
U_y = 0
U_z = 0

#カメラの傾き
alpha = math.radians(0) #x軸周り [rad]
beta = math.radians(0) #z軸周り [rad]

h = 0.1 #カメラ座標原点の高さ [m]

#放出から落下までの時間の導出
ar= Artools()
polar_change = ar.polar_change
def equation(t):
    return  m/k * (V_0*math.sin(theta_initial)-U_y-m*g/k*math.cos(alpha)*math.cos(beta))\
    *(1-math.exp(-k/m*t)) + (U_y+m*g/k*math.cos(alpha)*math.cos(beta))*t + y_0 - h

equal = 0 #等式が成り立つ

solution = fsolve(equation, equal)
t = solution[0]

print("t:",t)

x = m/k * (-U_x-m*g/k*math.cos(alpha)*math.sin(beta))*(1-math.exp(-k/m*t))\
    + (U_x+m*g/k*math.cos(alpha)*math.sin(beta))*t + x_0
y = m/k * (V_0*math.sin(theta_initial)-U_y-m*g/k*math.cos(alpha)*math.cos(beta))\
    *(1-math.exp(-k/m*t)) + (U_y+m*g/k*math.cos(alpha)*math.cos(beta))*t + y_0
z = m/k  * (V_0*math.cos(theta_initial)-m*g/k*math.sin(alpha))\
    *(1-U_z+math.exp(-k/m*t)) + (U_z-m*g/k*math.sin(alpha))*t +z_0

Theoretical_position = [x, y, z]  #落下位置[x,y,z]

print("落下位置[x,y,z] = ",Theoretical_position)

polar_position = polar_change(Theoretical_position) #落下位置[r,θ,φ]
del polar_position[2] #φの削除

print("落下位置[r, θ] = ",polar_position)
print(time.time()-st)

"""
メモ：
"""