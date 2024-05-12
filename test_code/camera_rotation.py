import numpy as np

CAMERA_RADIUS = 0.02

def camera_rotation(angle, camera):
    """
    引数：カメラの回転角度[rad]、カメラ極座標（R,theta）#fiが影響ないと仮定
    出力：回転後のカメラの極座標
    """
    # カメラの回転半径r
    r = CAMERA_RADIUS

    # カメラの極座標
    R = camera[0]
    theta = camera[1]
    
    # 回転中心から対象までの距離算出(余弦定理)
    l = (r**2 + R**2 -2*r*R*np.cos(np.pi-theta))**0.5
    
    #回転中心から対象までの角度算出(余弦定理)
    alpha = np.arccos((r**2 + l**2 - R**2)/(2*r*l))

    #余弦定理によって新しいRとthetaを算出
    R_new = (r**2 + l**2 - 2*r*l*np.cos(angle - alpha))**0.5
    theta_new = np.arccos((R_new**2 + l**2 - r**2)/(2*R_new*l)) + angle - alpha

    return R_new, theta_new