import numpy as np

def camera_rotation(AR_vec, rotx_vec, roty_vec, rotz_vec, θ, φ, ψ):
    '''
    input:  ARマーカの座標AR_vec(x_AR, y_AR, z_AR) [m]
            x軸周りの回転の中心座標rotx_vec(x_rotx, y_rotx, z_rotx) [m]
            y軸周りの回転の中心座標roty_vec(x_roty, y_roty, z_roty) [m]
            z軸周りの回転の中心座標rotz_vec(x_rotz, y_rotz, z_rotz) [m]
            x軸周りの回転角θ [rad] (y軸をz軸に近づける向きを正, 機体前側を上に向けると正)
            y軸周りの回転角φ [rad] (z軸をx軸に近づける向きを正, 機体を時計回りに回すと正)
            z軸周りの回転角ψ [rad] (x軸をy軸に近づける向きを正, 機体左側を上に向けると正)
            
    output: 任意の座標でx軸, y軸, z軸周りに回転した後のARマーカの座標AR_rot_vec(x_AR_rot, y_AR_rot, z_AR_rot)
    '''
    # x軸周りの回転
    def rotate_x(vec, center, θ):
        θ = -θ
        trans = np.array([[np.cos(θ), -np.sin(θ), center[1] - center[1] * np.cos(θ) + center[2] * np.sin(θ)],
                          [np.sin(θ), np.cos(θ), center[2] - center[1] * np.sin(θ) - center[2] * np.cos(θ)],
                          [0, 0, 1]])
        y_AR_rotx, z_AR_rotx, k = np.dot(trans, np.array([vec[1], vec[2], 1]))
        return y_AR_rotx, z_AR_rotx

    # y軸周りの回転
    def rotate_y(vec, center, φ):
        trans = np.array([[np.cos(φ), -np.sin(φ), center[0] - center[0] * np.cos(φ) + center[2] * np.sin(φ)],
                          [np.sin(φ), np.cos(φ), center[2] - center[0] * np.sin(φ) - center[2] * np.cos(φ)],
                          [0, 0, 1]])
        x_AR_roty, z_AR_roty, k = np.dot(trans, np.array([vec[0], vec[2], 1]))
        return x_AR_roty, z_AR_roty

    # z軸周りの回転
    def rotate_z(vec, center, ψ):
        ψ = -ψ
        trans = np.array([[np.cos(ψ), -np.sin(ψ), center[0] - center[0] * np.cos(ψ) + center[1] * np.sin(ψ)],
                          [np.sin(ψ), np.cos(ψ), center[1] - center[0] * np.sin(ψ) - center[1] * np.cos(ψ)],
                          [0, 0, 1]])
        x_AR_rotz, y_AR_rotz, k = np.dot(trans, np.array([vec[0], vec[1], 1]))
        return x_AR_rotz, y_AR_rotz

    # 初期のARマーカーの座標
    x_AR, y_AR, z_AR = AR_vec

    # x軸周りの回転
    y_AR, z_AR = rotate_x([x_AR, y_AR, z_AR], rotx_vec, θ)
    AR_vec = [x_AR, y_AR, z_AR]

    # y軸周りの回転
    x_AR, z_AR = rotate_y([x_AR, y_AR, z_AR], roty_vec, φ)
    AR_vec = [x_AR, y_AR, z_AR]

    # z軸周りの回転
    x_AR, y_AR, z_AR = AR_vec
    x_AR, y_AR = rotate_z([x_AR, y_AR, z_AR], rotz_vec, ψ)
    AR_vec = [x_AR, y_AR, z_AR]

    return AR_vec

# 使用例
AR_vec = [1, 1, 1]  # ARマーカの座標
rotx_vec = [0, -1, -1] # x軸周りの回転の中心座標
roty_vec = [-1, 0, -1] # y軸周りの回転の中心座標
rotz_vec = [-1, -1, 0] # z軸周りの回転の中心座標
θ = np.pi / 4  # x軸周りの回転角
φ = np.pi / 4  # y軸周りの回転角
ψ = np.pi / 4  # z軸周りの回転角

rotated_AR_vec = camera_rotation(AR_vec, rotx_vec, roty_vec, rotz_vec, θ, φ, ψ)
print(rotated_AR_vec)



"""
サーボで何度y軸を回転させたかを出力
➡兜ファンクションの入力としてARマーカのtvecを広く捉える

"""

def adjust_angle(self, tvec):
    print(f"\033[33m", f"adjust angle : tvec = {tvec}", "\033[0m")	

    self.distanceAR = (tvec[0]**2 + tvec[1]**2 + tvec[2]**2) ** (1/2)
    angle_change = 0  # 回転した角度を保持する変数

    if tvec[0] > 0.03:
        if self.nowangle >= 100:
            print("=@=@=servo: "+str(self.nowangle),">160")
            self.unable_rotation_count += 1
            print("\033[44m ===== +1 ===== \033[0m")
            if self.unable_rotation_count > 1:
                self.motor_control(70, -70, 0.3)
                time.sleep(1)
                print("\033[44m ===== servo ===== \033[0m")
                self.unable_rotation_count = 0
            return angle_change  # 角度の変化がないので0をリターン
        else:
            self.nowangle += 3
            angle_change = 3  # 正の方向に3度回転
            self.servo.go_deg(self.nowangle)
            print(f"=@=@=servo: {self.nowangle} (rotated +3 degrees around the axis)")

    elif tvec[0] < -0.03:
        if self.nowangle <= 20:
            print("=@=@=servo: "+str(self.nowangle),"<=30")
            self.unable_rotation_count += 1
            print("\033[44m ===== +1 ===== \033[0m")
            if self.unable_rotation_count > 1:
                self.motor_control(-70, 70, 0.3)
                time.sleep(1)
                print("\033[44m ===== servo ===== \033[0m")
                self.unable_rotation_count = 0
            return angle_change  # 角度の変化がないので0をリターン
        else:
            self.nowangle -= 3
            angle_change = -3  # 負の方向に3度回転
            self.servo.go_deg(self.nowangle)
            print(f"=@=@=servo: {self.nowangle} (rotated -3 degrees around the axis)")

    else:
        print("~~~~~~~~~")
        return angle_change  # 角度の変化がないので0をリターン

    return angle_change  # 最後に回転角度をリターン
