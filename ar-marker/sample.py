import cv2
import numpy as np
import cv2.aruco as aruco

# カメラの読込み
# 内蔵カメラがある場合、下記引数の数字を変更する必要あり
cap = cv2.VideoCapture(1)

ret, frame = cap.read()
size = frame.shape

focal_length = size[1]
center = (size[1]/2, size[0]/2)

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
            figure_points_3D = np.array([ # 画像上の点の３次元空間での座標
                            ( 0.5, 0.5, 0.0),
                            ( 0.5,-0.5, 0.0),
                            (-0.5,-0.5, 0.0),
                            (-0.5, 0.5, 0.0),
                        ])
            
            draw_line = np.array([(3,3,0),(3,-3,0),(-3,-3,0),(-3,3,0)])
            next_start_point = image_points_2D

            objPoints=image_points_2D
             # 上記を対応させて姿勢などを算出する
            suc,rvec, tvec=cv2.solvePnP(figure_points_3D, image_points_2D,cameraMatrix,distCoeff)
            # cv2.polylines(frame, np.array(points).astype(int), isClosed=True, color=(0, 255, 0), thickness=1)
            # cv2.drawMarker(frame, np.array(points[0][0]).astype(int), color=(255, 0, 255), markerType=cv2.MARKER_SQUARE, thickness=1,markerSize=10)
            # cv2.putText(frame,str(id[0]),np.array(points[0][0]).astype(int), fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(255, 0, 0),thickness=2,lineType=cv2.LINE_AA)
            edges = [] # 辺の描画用
            # 高さにあたる辺の描画
            for point2,point3,line in zip(image_points_2D,figure_points_3D,draw_line):

                end_point3D = point3+line
                start_point2D= np.array([[point2]])
                print("!!",start_point2D,end_point3D,np.array([point2]) )


                end_point2D, jacobian = cv2.projectPoints(end_point3D, rvec, tvec,cameraMatrix,distCoeff)

                point1 = ( int(start_point2D[0][0][0]), int(start_point2D[0][0][1]))
                point2 = ( int(end_point2D[0][0][0]), int(end_point2D[0][0][1]))
                print(point2)
                # next_start_point = end_point2D[0]
                edges.append(point2) # 辺の描画用
                # cv2.line(frame, point1, point2, (0,255,0), 1)
            
            # 洞窟の再現
            # for i in range(len(edges)):
                # cv2.line(frame, edges[i-1], edges[i], (0,255,0), 5)
            print("!!",edges)
            # cv2.fillPoly(frame, [edges], color="b")
            cv2.fillPoly(frame, [np.array(edges)], (255, 150, 150))

            # # 上面に対応する辺の描画
            # for i in range(4):
            #     end_point3D = figure_points_3D[i]+np.array([[0,0,1]])#[[-0.5  0.5  1. ]]
            #     end_point2D, jacobian = cv2.projectPoints(end_point3D, rvec, tvec,cameraMatrix,distCoeff)
            #     point1 = ( int(end_point2D[0][0][0]), int(end_point2D[0][0][1]))
                
            #     start_point3D= figure_points_3D[(i+1)%4]+np.array([[0,0,1]])
            #     start_point2D, jacobian = cv2.projectPoints(start_point3D, rvec, tvec,cameraMatrix,distCoeff)
            #     point2 = ( int(start_point2D[0][0][0]), int(start_point2D[0][0][1]))

            #     cv2.line(frame, point1, point2, (0,255,0), 1)

            #     if i==0:
            #         cv2.drawMarker(frame, point1, color=(255, 255), markerType=cv2.MARKER_SQUARE, thickness=1,markerSize=10)

    # GUIに表示
    cv2.imshow("Camera", frame)
    # qキーが押されたら途中終了
    if cv2.waitKey(1) == ord('q'):
        break

# 終了処理
cap.release()
cv2.destroyAllWindows() 
