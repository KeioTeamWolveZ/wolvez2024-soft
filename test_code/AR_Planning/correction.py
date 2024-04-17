import math

def Correct(vector1, vector2):
    '''
    入力:ダミー？落下位置(x, y, z)、目標位置(xg, yg, zg)
    定数:
    出力:補正飛距離distance、補正角度angle (xz平面)
    '''
    x,y,z = vector1
    xg,yg,zg = vector2
    r = (x**2 + z**2)**(1/2) #落下位置のカメラからの距離rの算出(xz平面)
    rg = (xg**2 + zg**2)**(1/2) #目標位置のカメラからの距離rg(xz平面)
    distance = rg - r
    angle = math.acos((r**2 + rg**2 - ((xg - x)**2 + (zg - z)**2))/(2*r*rg))
    return (distance, angle)

# print(Correct(1, 0, 0, 0, 1, 1))
# print(math.acos(0))
