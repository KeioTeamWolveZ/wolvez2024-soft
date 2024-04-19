import numpy as np

def Correct(P, Pg):
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

print(Correct((0, 1, 1), (0.1, 1, 1)))
