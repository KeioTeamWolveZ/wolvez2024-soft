import numpy as np
from numpy.typing import _128Bit
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# 定数
m = 0.005  # 質量
g = 9.81  # 重力加速度
k = 0.05  # 空気抵抗係数
V0 = 4.684517487133138  # 初速度
theta = np.deg2rad(45)  # 放出角度（ラジアン）
U = np.array([0, 0, 0])  # 風速ベクトル
x0, y0, z0 = 0.0, -0.02, -0.03  # 初期位置

def calculate_position_at_yg(tvec, alpha, beta, tolerance):
    xg, yg, zg = tvec

    def position(t, alpha, beta):
        x = (U[0] + (m * g / k) * np.cos(alpha) * np.sin(beta)) * (t + (m / k) * (1 - np.exp(-k * t / m))) + x0
        y = (m / k) * (-V0 * np.sin(theta) - U[1] - (m * g / k) * np.cos(alpha) * np.cos(beta)) * (1 - np.exp(-k * t / m)) + (U[1] + (m * g / k) * np.cos(alpha) * np.cos(beta)) * t + y0
        z = (m / k) * (V0 * np.cos(theta) - U[2] + (m * g / k) * np.sin(alpha)) * (1 - np.exp(-k * t / m)) + (U[2] - (m * g / k) * np.sin(alpha)) * t + z0
        return x, y, z

    def find_t(t, alpha, beta, yg):
        _, y, _ = position(t, alpha, beta)
        return y - yg

    # 初期推定値
    t_initial_guess = 1

    # y = ygとなるtを求める
    t_solution = fsolve(find_t, t_initial_guess, args=(alpha, beta, yg))[0]

    # 求めたtでのx, y, zを算出
    x, y, z = position(t_solution, alpha, beta)
    
    # x, zが範囲内にあるかを確認
    x_within_range = (xg - tolerance <= x <= xg + tolerance)
    z_within_range = (zg - tolerance <= z <= zg + tolerance)

    if x_within_range and z_within_range:
        result = True
    
    elif x_within_range:
        if z < zg - tolerance:
            result = "Go Straight"
        elif z > zg + tolerance:
            result = "Go Back"

    elif z_within_range:
        if x < xg - tolerance:
            result = "Turn Right"
        elif x > xg + tolerance:
            result = "Turn Left"
    
    else:
        if x < xg - tolerance and z < zg - tolerance:
            result = "Go Straight & Turn Right"
        elif x > xg - tolerance and z < zg - tolerance:
            result = "Go Straight & Turn Left"
        elif x < xg - tolerance and z > zg - tolerance:
            result = "Go Back & Turn Right"
        elif x > xg - tolerance and z > zg - tolerance:
            result = "Go Back & Turn Left"

    return t_solution, x, y, z, result

# 使用例
tvec = [0, 0.1, 0.5]  # 目標値
alpha = np.deg2rad(0)  # αの角度（度）
beta = np.deg2rad(0)  # βの角度（度）
tolerance = 0.2  # 許容範囲

t_sol, x_sol, y_sol, z_sol, result = calculate_position_at_yg(tvec, alpha, beta, tolerance)

print(f"At t = {t_sol}: x = {x_sol}, y = {y_sol}, z = {z_sol}")
print(result)

# # 時間ベクトルの設定
# t = np.linspace(0, 11, 10000)  # 0から10秒まで100ポイント
# t = 0.5583396188395241

# # 軌跡の計算
# x = (U[0] + (m * g * np.cos(alpha) * np.sin(beta)) / k) * \
#     (t + (m / k) * np.exp(-k * t / m) - m / k) + x0

# y = (m / k) * (-V0 * np.sin(theta) - U[1] - (m * g * np.cos(alpha) * np.cos(beta)) / k) * \
#     (1 - np.exp(-k * t / m)) + (U[1] + (m * g * np.cos(alpha) * np.cos(beta)) / k) * t + y0

# z = (m / k) * (V0 * np.cos(theta) - U[2] + (m * g * np.sin(alpha)) / k) * \
#     (1 - np.exp(-k * t / m)) + (U[2] - (m * g * np.sin(alpha)) / k) * t + z0

# print(f"t = {t}, x = {x}, y = {y}, z = {z}")

# # 3Dプロット
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x, y, z, linewidth=2)

# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# ax.set_title('放出物の移動軌跡')

# plt.grid(True)
# plt.show()

# # 2Dプロット (z軸を横軸、y軸を縦軸)
# plt.figure()
# # plt.plot(t, -y, linewidth=2)
# plt.plot(t, z, linewidth=2)

# plt.xlabel('t [s]')
# plt.ylabel('z [m]')
# plt.xlim(0, 0.5)
# plt.ylim(-0.1, 0.1)
# plt.grid(True)
# plt.show()