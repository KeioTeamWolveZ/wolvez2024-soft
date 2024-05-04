import numpy as np
from collections import deque
from scipy.optimize import fsolve
import time
import math 


class Artools:
    def __init__(self):
        
        pass

    # ==================== from polar.py ====================
    ## 作成者：伊藤、木村
    def theta_angle(self,vector_1, vector_2):
        #各単位ベクトルの導出
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        cos = np.dot(unit_vector_1, unit_vector_2)
        theta_angle = np.arccos(cos)
        theta = theta_angle / np.pi * 180 #ラジアン→弧度
        return theta

    def phi_angle(self,vector_1, vector_3):
        #各単位ベクトルの導出
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_3 = vector_3 / np.linalg.norm(vector_3)
        cos = np.dot(unit_vector_1, unit_vector_3)
        phi_angle = np.arccos(cos)
        phi = phi_angle / np.pi * 180 #ラジアン→弧度
        return phi

    #tvecを任意に決定
    tvec = [-1,1,1]

    def polar_change(self,tvec):
        vector_1 = np.array([0, 1]) #基準となるベクトル
        vector_2 = np.array([tvec[0],tvec[2]])
        vector_3 = np.array([tvec[1],math.sqrt(tvec[0]**2 + tvec[2]**2)])
        r = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
        theta = self.theta_angle(vector_1, vector_2)
        phi = self.phi_angle(vector_1, vector_3)
        
        #x座標が左側(負の値)のとき、thetaを負にする
        if tvec[0] > 0: 
            theta = theta
        else:
            theta = -theta
        #y座標を下向きに取る時、y座標が正の時、phiを負にする
        if tvec[1] > 0: 
            phi = -phi
        else:
            theta = phi
        pvec = [r,theta,phi]
        return pvec
    
    # ==================== from correction.py ====================
    ## 作成者：木村
    def Correct(self,P, Pg):
        '''
        入力:ダミー？落下位置P(x, y, z)、目標位置Pg(xg, yg, zg)
        定数:
        出力:補正飛距離distance、補正角度angle (xz平面)
        '''
        vec = np.array((P[0], P[2]))
        unit_vec = vec/np.linalg.norm(vec)
        vecg = np.array((Pg[0], Pg[2]))
        unit_vecg = vecg/np.linalg.norm(vecg)

        distance = np.linalg.norm(vecg) - np.linalg.norm(vec)
        
        cos = np.dot(unit_vec, unit_vecg)
        angle = np.rad2deg(np.arccos(cos))
        cross = np.cross(vec, vecg)
        if cross < 0:
            angle *= -1

        return (distance, angle)

    # ==================== from AR_outlier.py ====================
    ## 作成者：太田こ
    def outlier(self,tvec, prev, standard =0.2):
        """
        Check if the tvec is an outlier or not

        Args:
            tvec (np.array): translation vector
            prev (list): list of previous translation vectors
            standard (float): standard deviation
        """
        # print(prev)
        # print(np.mean([i[0] for i in prev]))
        
        mean = np.array([np.mean([i[0] for i in prev]), np.mean([i[1] for i in prev]), np.mean([i[2] for i in prev])])

        if abs(tvec[0] - mean[0]) < standard and abs(tvec[1] - mean[1]) < standard and abs(tvec[2] - mean[2]) < standard:
            prev.append(tvec)
            if len(prev) > 5:
                prev.pop(0)
            return True
        else:
            return False



# ==================== test polar.py ====================
# tvec = [-1,1,1]
# ar = Artools()
# print(ar.polar_change(tvec))
# ==================== test correction.py ====================
# print(Correct((0.17389422024940773,0.1968730025228114,0.4020359587429208), (0.1, 1, 1)))
# ==================== test AR_outlier.py ====================
# tvec = np.array([1,1,1])
# prev = np.array([[1,1,1],[1,1,2]])
# ar = Artools()
# print(ar.outlier(tvec, prev, 0.2))