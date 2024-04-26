import numpy as np

tvec = np.array([[1, 1, 1]])
prev = np.array([[2.5, 1, 1]])

def outlier(tvec, prev, standard=0.05):
    """
    作成：太田
    引数：tvec:3次元座標のリスト、standard:基準値、prev:保存されているtvec
    出力：外れ値かどうかの真偽値、new_prev:更新されたprev
    取得した座標ベクトルが基準値を超えているかどうかを判定し、超えていた場合はFalseを返す。
    また、Falseの場合、新しい座標ベクトルをprevに保存する。
    """
    error = np.linalg.norm(tvec - prev)
    if error < standard:
        # print("ok")
        return error < standard, tvec
    else:
        # print("error")
        return error < standard, prev
    
    ### 以下、各座標に関しての判定ver
    # error = (tvec - prev)
    # if error[0] < standard and error[1] < standard and error[2] < standard:
    #     print("ok")
    #     return error[0] < standard, tvec
    # else:
    #     print("error")
    #     return error[0] < standard, prev


print(outlier(tvec, prev))