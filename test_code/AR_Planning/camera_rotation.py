import numpy as np

def translate_coordinates(cam_vec, body_vec, θ):
    '''
    input:  カメラ座標系での任意の座標cam_vec(x, y, z), 
            カメラ座標原点に対するカメラの回転軸中心座標body_vec(x_0, y_0, z_0)
            カメラの回転角度θ(機体座標系y軸周りの回転) z軸をx軸に近づけるが正
    const: 
    output: カメラ座標系での任意の座標をカメラの回転角度θに基づいて変換した座標rot_vec(x_r, y_r, z_r)
    '''
    x, y, z = cam_vec
    x_0, y_0, z_0 = body_vec

    #カメラ座標から機体座標（カメラの回転軸中心を原点とする直交座標）への変換（平行移動）
    x2, y2, z2 = x - x_0, y - y_0, z - z_0

    #機体座標系の座標(x2, y2, z2)をy軸周りにθ回転（回転移動）
    rot_y = np.array([[np.cos(θ), 0, -np.sin(θ)],[0, 1, 0],[np.sin(θ), 0, np.cos(θ)]]) #y軸周りの回転行列を定義
    x3, y3, z3 = np.dot(rot_y, np.array([x2, y2, z2]))

    #カメラ座標から機体座標への変換（平行移動）
    x_0, y_0, z_0 = np.dot(rot_y, np.array([x_0, y_0, z_0]))
    rot_vec = x3 + x_0, y3 + y_0, z3 + z_0

    return rot_vec

print(translate_coordinates([1, 0, 1], [2, 0, 2], np.pi/4))