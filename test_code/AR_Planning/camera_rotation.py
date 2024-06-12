import numpy as np

def translate_coordinates(cam_vec, body_vec_x, body_vec_y, body_vec_z, θ, φ, ψ):
    '''
    input:  カメラ座標系での任意の座標cam_vec(x, y, z), 
            カメラ座標原点に対するカメラの回転軸中心座標body_vec(x_0, y_0, z_0)
            カメラの回転角度θ(機体座標系y軸周りの回転) z軸をx軸に近づけるが正
    const: 
    output: カメラ座標系での任意の座標をカメラの回転角度θに基づいて変換した座標rot_vec(x_r, y_r, z_r)
    '''
    x, y, z = cam_vec
    x0_x, y0_x, z0_x = body_vec_x
    x0_y, y0_y, z0_y = body_vec_y
    x0_z, y0_z, z0_z = body_vec_z

  #x軸周り
    #カメラ座標から機体座標（カメラの回転軸中心を原点とする直交座標）への変換（平行移動）
    x2, y2, z2 = x - x0_x, y - y0_x, z - z0_x

    #機体座標系の座標(x2, y2, z2)をx軸周りにθ回転（回転移動）
    rot_x = np.array([[1, 0, 0],[0, np.cos(θ), -np.sin(θ)],[0, np.sin(θ), np.cos(θ)]]) #x軸周りの回転行列を定義
    x3, y3, z3 = np.dot(rot_x, np.array([x2, y2, z2]))

    #カメラ座標から機体座標への変換（平行移動）
    x0_x, y0_x, z0_x = np.dot(rot_x, np.array([x0_x, y0_x, z0_x]))
    rot_vec_x = x3 + x0_x, y3 + y0_x, z3 + z0_x

  #y軸周り
    #カメラ座標から機体座標（カメラの回転軸中心を原点とする直交座標）への変換（平行移動）
    x, y, z = rot_vec_x
    x2, y2, z2 = x - x0_y, y - y0_y, z - z0_y

    #機体座標系の座標(x2, y2, z2)をy軸周りにθ回転（回転移動）
    rot_y = np.array([[np.cos(φ), 0, -np.sin(φ)],[0, 1, 0],[np.sin(φ), 0, np.cos(φ)]]) #y軸周りの回転行列を定義
    x3, y3, z3 = np.dot(rot_y, np.array([x2, y2, z2]))

    #カメラ座標から機体座標への変換（平行移動）
    x0_y, y0_y, z0_y = np.dot(rot_y, np.array([x0_y, y0_y, z0_y]))
    rot_vec_y = x3 + x0_y, y3 + y0_y, z3 + z0_y

  #z軸周り
    #カメラ座標から機体座標（カメラの回転軸中心を原点とする直交座標）への変換（平行移動）
    x, y, z = rot_vec_y
    x2, y2, z2 = x - x0_z, y - y0_z, z - z0_z

    #機体座標系の座標(x2, y2, z2)をy軸周りにθ回転（回転移動）
    rot_z = np.array([[np.cos(ψ), -np.sin(ψ), 0],[np.sin(ψ), np.cos(ψ), 0],[0, 0, 1]]) #z軸周りの回転行列を定義
    x3, y3, z3 = np.dot(rot_z, np.array([x2, y2, z2]))

    #カメラ座標から機体座標への変換（平行移動）
    x0_z, y0_z, z0_z = np.dot(rot_z, np.array([x0_z, y0_z, z0_z]))
    rot_vec_z = x3 + x0_z, y3 + y0_z, z3 + z0_z

    rot_vec = rot_vec_z
    return rot_vec

print(translate_coordinates([1, 0, 1], [0, 0, 0], [2, 0, 2], [0, 0, 0], 0, np.pi/4, 0))