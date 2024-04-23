"""
import math

tvec = []

def polar_change(tvec):
    
    直交座標から極座標に変換
    input:
        tvec=[x,y,z]
    output:
        pvec=[r,theta,phi]
    
    r = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
    theta = 90 - math.degrees(math.atan2(tvec[1], tvec[0]))
    phi = -math.degrees(math.asin(tvec[2] / r))
    #phi = math.degrees(math.atan2(tvec[1], tvec[2]))
    pvec = [r,theta,phi]
    return pvec
    
print(polar_change(tvec = [0,-1,1]))  

"""


"""
以下、ベクトルを使
"""
import math
import numpy as np
import matplotlib.pyplot as plt

tvec = [-1,1,1]

vector_1 = np.array([0, 1]) #基準となる線(θ、phi)
vector_2 = np.array([tvec[0],tvec[2]])
vector_3 = np.array([tvec[1],math.sqrt(tvec[0]**2 + tvec[2]**2)])

def theta_angle(vector_1, vector_2):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    cos = np.dot(unit_vector_1, unit_vector_2)
    theta_angle = np.arccos(cos)
    theta = theta_angle / np.pi * 180
    return theta

def phi_angle(vector_1, vector_3):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_3 = vector_3 / np.linalg.norm(vector_3)
    cos = np.dot(unit_vector_1, unit_vector_3)
    phi_angle = np.arccos(cos)
    phi = phi_angle / np.pi * 180
    return phi

r = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
theta = theta_angle(vector_1, vector_2)
phi = phi_angle(vector_1, vector_3)

#xが負の時、θが負値をとるよう設定
if tvec[0] > 0: 
    theta = theta
else:
    theta = -theta
    
#y軸を下向きに取っている場合、
#ｙが正の時、ｐhiが負値をとるよう設定 
if tvec[1] > 0: 
    phi = -phi
else:
    theta = phi
       
pvec = [r,theta,phi]
print(pvec)





