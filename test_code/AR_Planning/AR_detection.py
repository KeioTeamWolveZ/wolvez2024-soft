import cv2
import numpy as np
import cv2.aruco as aruco
from collections import deque

# カメラの読込み
# 内蔵カメラがある場合、下記引数の数字を変更する必要あり
cap = cv2.VideoCapture(1)

ret, frame = cap.read()
size = frame.shape

focal_length = size[1]
center = (size[1]/2, size[0]/2)
maxlen = 20
edges_history = deque(maxlen=maxlen)
edges_last = (0,0,0)

# fx,fy,cx,cy=focal_length,focal_length,center[0],center[1]
# cameraMatrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
# k1,k2,p1,p2=0,0,0,0
# distCoeff=np.array([[k1,k2,p1,p2]])
# distCoeff=np.zeros((4,1))

cameraMatrix = np.load("mtx.npy")
distCoeff = np.load("dist.npy")

detector = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)




# 動画終了まで、1フレームずつ読み込んで表示する。
while(cap.isOpened()):
    # 1フレーム毎　読込み
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ARマーカーの検出
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, detector)

    if len(corners)>0:
        for points,id in zip(corners,ids):

            image_points_2D = np.array(points[0],dtype="double") #画像上の座標(マーカー認識の結果)
            print(points[0])
            # figure_points_3D = np.array([ # 画像上の点の３次元空間での座標
            #                 ( 0.5, 0.5, 0),
            #                 ( 0.5,-0.5, 0),
            #                 (-0.5,-0.5, 0),
            #                 (-0.5, 0.5, 0),
            #             ])
            figure_points_3D = np.array([ # 画像上の点の３次元空間での座標
                            ( 0, -0.5, 0.5),
                            ( 0, -0.5,-0.5),
                            ( 0, 0.5,-0.5),
                            ( 0, 0.5, 0.5),
                        ])
            
            # draw_line = np.array([(3,3,0),(3,-3,0),(-3,-3,0),(-3,3,0)])
            draw_line = np.array([(-3,0,3),(-3,0,-3),(3,0,-3),(3,0,3)])
            next_start_point = image_points_2D

            objPoints=image_points_2D
             # 上記を対応させて姿勢などを算出する
            suc,rvec, tvec=cv2.solvePnP(figure_points_3D, image_points_2D,cameraMatrix,distCoeff)
            edges = [] # 辺の描画用
            # 高さにあたる辺の描画
            for point2,point3,line in zip(image_points_2D,figure_points_3D,draw_line):

                end_point3D = point3+line
                start_point2D= np.array([[point2]])
                # print("!!",start_point2D,end_point3D,np.array([point2]) )


                end_point2D, jacobian = cv2.projectPoints(end_point3D, rvec, tvec,cameraMatrix,distCoeff)

                point1 = ( int(start_point2D[0][0][0]), int(start_point2D[0][0][1]))
                point2 = ( int(end_point2D[0][0][0]), int(end_point2D[0][0][1]))
                # print(point2)
                # next_start_point = end_point2D[0]
                edges.append(point2) # 辺の描画用
                # cv2.line(frame, point1, point2, (0,255,0), 1)
                
            
            # 洞窟の再現
            # edges_history.append(edges)
            
            if len(edges_history) > 2:
                if abs(edges_last[0][0]-edges[0][0])>15 or abs(edges_last[0][1]-edges[0][1])>15 or abs(edges_last[1][0]-edges[1][0])>15 or abs(edges_last[1][1]-edges[1][1])>15 or abs(edges_last[2][0]-edges[2][0])>15 or abs(edges_last[2][1]-edges[2][1])>15 or abs(edges_last[3][0]-edges[3][0])>15 or abs(edges_last[3][1]-edges[3][1])>15:
                    print("error-------------------------------/n/n/n/n/n//n-------------------------q")
                    pass    
                else:
                    print("ok")
                    edges = np.mean([edges_history[-1],edges], axis=0).astype(int)
                    edges_history.append(edges)
            else:
                edges_history.append(edges)
            edges_last = edges 
            # if len(edges_history) > 2:
            #     edges = np.mean([edges_history[-1],edges], axis=0).astype(int)
            # edges_history.append(edges)
            # print(edges_history[-1][0][0])
                

            # print(edges_history)
            if len(edges_history) == maxlen:
                edges = np.mean(edges_history, axis=0).astype(int)
            

            for i in range(len(edges)):
                cv2.line(frame, edges[i-1], edges[i], (0,255,0), 5)
            # print("!!",edges)
            # cv2.fillPoly(frame, [edges], color="b")
            # cv2.fillPoly(frame, [np.array(edges)], (255, 150, 150))

    # GUIに表示
    cv2.imshow("Camera", frame)
    # qキーが押されたら途中終了
    if cv2.waitKey(1) == ord('q'):
        break

# 終了処理
cap.release()
cv2.destroyAllWindows() 