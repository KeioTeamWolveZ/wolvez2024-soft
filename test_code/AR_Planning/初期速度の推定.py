import numpy as np
from scipy.optimize import fsolve

def calculate_v0(h, Z, x0, y0, z0, Ux, Uy, Uz, m, g, k, alpha, beta, theta):
    """
    初期速度v0を計算する関数

    Parameters:
    h (float): y(t)の値
    Z (float): z(t)の値
    Ux, Uy, Uz (float): 風速ベクトルの各成分
    m (float): 質量
    g (float): 重力加速度
    k (float): 空気抵抗係数
    alpha (float): x軸周りのカメラの傾き (rad)
    beta (float): z軸周りのカメラの傾き (rad)
    theta (float): 放出角度 (rad)

    Returns:
    float: 初期速度v0
    """
    
    # 方程式の定義
    def equations(vars):
        t, v0 = vars
        
        # x(t) の式
        x_t = (Ux + (m * g / k) * np.cos(alpha) * np.sin(beta)) * (t + (m / k) * (1 - np.exp(-k * t / m))) + x0
        
        # y(t) の式
        y_t = (m / k) * (-v0 * np.sin(theta) - Uy - (m * g / k) * np.cos(alpha) * np.cos(beta)) * (1 - np.exp(-k * t / m)) + (Uy + (m * g / k) * np.cos(alpha) * np.cos(beta)) * t + y0
        
        # z(t) の式
        z_t = (m / k) * (v0 * np.cos(theta) - Uz + (m * g / k) * np.sin(alpha)) * (1 - np.exp(-k * t / m)) + (Uz - (m * g / k) * np.sin(alpha)) * t + z0
        
        return [y_t - h, z_t - Z]
    
    # 初期推定値
    initial_guess = [1, 1]
    
    # 方程式を解く
    t, v0 = fsolve(equations, initial_guess)
    
    return t, v0

# 例のパラメータ (具体的な値は適宜設定してください)
x0 = 0
y0 = -0.02
z0 = -0.035
Ux = 0  # 風速Ux
Uy = 0  # 風速Uy
Uz = 0  # 風速Uz
m = 0.005  # 質量
g = 9.81  # 重力加速度
k = 0.05  # 空気抵抗係数
alpha = 0  # x軸周りのカメラの傾き
beta = 0  # z軸周りのカメラの傾き
theta = np.deg2rad(45)  # 放出角度 (45度)

# 関数を呼び出して初期速度v0を計算
h = 0.1  # y(t)の値
Z = 0.6  # z(t)の値
t, v0 = calculate_v0(h, Z, x0, y0, z0, Ux, Uy, Uz, m, g, k, alpha, beta, theta)
print(f"落下時間 t: {t} s")
print(f"初期速度 v0: {v0} m/s")

y_t = (m / k) * (-v0 * np.sin(theta) - Uy - (m * g / k) * np.cos(alpha) * np.cos(beta)) * (1 - np.exp(-k * t / m)) + (Uy + (m * g / k) * np.cos(alpha) * np.cos(beta)) * t + y0
print(f"y(t): {y_t}")
z_t = (m / k) * (v0 * np.cos(theta) - Uz + (m * g / k) * np.sin(alpha)) * (1 - np.exp(-k * t / m)) + (Uz - (m * g / k) * np.sin(alpha)) * t + z0
print(f"z(t): {z_t}")
