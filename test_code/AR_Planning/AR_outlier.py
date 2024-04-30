import numpy as np

tvec = np.array([1,1,1])
prev = []

def outlier(tvec, prev, standard = 0.02):
    """Check if the tvec is an outlier or not

    Args:
        tvec (np.array): translation vector
        prev (list): list of previous translation vectors
        standard (float): standard deviation
    """ 

    mean = np.array([np.mean([i[0] for i in prev]), np.mean([i[1] for i in prev]), np.mean([i[2] for i in prev])])
    
    if abs(tvec[0] - mean[0]) < standard and abs(tvec[1] - mean[1]) < standard and abs(tvec[2] - mean[2]) < standard:
        prev.append(tvec)
        if len(prev) > 5:
            prev.pop(0)
        return True, tvec
    else:
        return False