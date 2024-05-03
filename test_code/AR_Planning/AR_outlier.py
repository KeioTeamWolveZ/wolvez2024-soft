import numpy as np
from collections import deque

tvec = np.array([1,1,1])
prev = np.array([[1,1,1],[1,1,2]])
# print(prev)

def outlier(tvec, prev, standard =0.2):
    """Check if the tvec is an outlier or not

    Args:
        tvec (np.array): translation vector
        prev (list): list of previous translation vectors
        standard (float): standard deviation
    # """ 
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


print(outlier(tvec, prev, 0.2))