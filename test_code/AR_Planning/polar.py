import math

tvec = []

def polar_change(tvec):
    """
    直交座標から極座標に変換
    input:
        tvec=[x,y,z]
    output:
        pvec=[r,theta,phi]
    """
    r = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
    theta = 90 - math.degrees(math.atan2(tvec[1], tvec[0]))
    phi = math.degrees(math.asin(tvec[2] / r))
    pvec = [r,theta,phi]
    return pvec
    
print(polar_change(tvec = [2,3,5]))    


