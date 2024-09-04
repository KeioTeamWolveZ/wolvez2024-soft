import cv2
import cv2.aruco as aruco
import numpy as np

# 変更するパラメータ
marker_len = 0.0165  # m
goal_radius = 0.15  # m
mtx = np.load('mtx.npy')
dist = np.load('dist.npy')
image = cv2.imread('results//photo_15cm_0.jpg')

# グローバル変数の定義
ellipse_center = None
ellipse_axes = None
ellipse_angle = None
circle_drawn = False

def camera_to_pixel(points_3d, mtx, dist):
    """3Dポイントをカメラのピクセル座標に変換する関数"""
    points_3d = np.array(points_3d, dtype=np.float32).reshape(-1, 1, 3)
    rvec = np.zeros((3, 1), dtype=np.float32)
    tvec = np.zeros((3, 1), dtype=np.float32)
    points_2d, _ = cv2.projectPoints(points_3d, rvec, tvec, mtx, dist)
    return points_2d.reshape(-1, 2)

def create_circle_points(center, radius, rotation_matrix, num_points=100):
    """円の3Dポイントを生成し、回転して位置を更新する関数"""
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_points = np.array([
        [radius * np.cos(angle), 0, radius * np.sin(angle)]
        for angle in angles
    ])
    circle_points = np.dot(circle_points, rotation_matrix.T)
    circle_points += center
    return circle_points

def point_in_ellipse(point, ellipse_center, axes, angle):
    """点が楕円の中にあるかどうかを判定する関数"""
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    x = point[0] - ellipse_center[0]
    y = point[1] - ellipse_center[1]
    x_rot = x * cos_angle + y * sin_angle
    y_rot = -x * sin_angle + y * cos_angle
    return (x_rot ** 2 / axes[0] ** 2) + (y_rot ** 2 / axes[1] ** 2) <= 1

def mouse_callback(event, x, y, flags, param):
    """マウスイベントに基づくコールバック関数"""
    global ellipse_center, ellipse_axes, ellipse_angle, circle_drawn

    if event == cv2.EVENT_LBUTTONDOWN:
        if ellipse_center is not None and ellipse_axes is not None and ellipse_angle is not None:
            if point_in_ellipse((x, y), ellipse_center, ellipse_axes, ellipse_angle):
                print("IN")
            else:
                print("OUT")
            return

        if corners is not None:
            for i, marker in enumerate(corners):
                if cv2.pointPolygonTest(marker[0], (x, y), False) >= 0:
                    if not circle_drawn:
                        # 選択されたマーカーの位置と回転を推定
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers([marker], marker_len, mtx, dist)
                        rotation_matrix, _ = cv2.Rodrigues(rvec[0][0])

                        # マーカーの接地点を計算
                        marker_center = tvec[0][0] + np.dot([0, marker_len / 2, marker_len / 2], rotation_matrix.T)

                        # 円のカメラ座標上の軌跡を生成
                        circle_points_camera = create_circle_points(marker_center, goal_radius, rotation_matrix)

                        # カメラ座標の円をピクセル座標に変換
                        circle_points_pixel = camera_to_pixel(circle_points_camera, mtx, dist)

                        # 楕円の中心、軸、角度を計算して保存
                        ellipse_center = np.mean(circle_points_pixel, axis=0)
                        ellipse_axes = (
                            np.linalg.norm(circle_points_pixel[0] - ellipse_center),
                            np.linalg.norm(circle_points_pixel[len(circle_points_pixel) // 4] - ellipse_center)
                        )
                        ellipse_angle = np.arctan2(
                            circle_points_pixel[0][1] - ellipse_center[1],
                            circle_points_pixel[0][0] - ellipse_center[0]
                        )

                        # 画像上に楕円を描画
                        cv2.ellipse(
                            image,
                            (int(ellipse_center[0]), int(ellipse_center[1])), 
                            (int(ellipse_axes[0]), int(ellipse_axes[1])),
                            np.degrees(ellipse_angle),
                            0, 360,
                            (0, 255, 0),
                            2
                        )

                        # マーカーをピンク色で囲む
                        cv2.polylines(image, [np.int32(marker[0])], isClosed=True, color=(255, 0, 255), thickness=1)

                        cv2.imshow('Result', image)
                        circle_drawn = True
                    break

# 画像の読み込みとグレースケール変換
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# ARマーカーの検出
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

if len(corners) > 0:
    # マーカーを描画
    image = aruco.drawDetectedMarkers(image, corners)

    # ウィンドウにマウスコールバック関数を設定
    cv2.namedWindow('Result')
    cv2.setMouseCallback('Result', mouse_callback)

    # 結果画像を表示
    cv2.imshow('Result', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("ARマーカーが検出されませんでした。")
