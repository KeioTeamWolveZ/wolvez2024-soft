import cv2
from datetime import datetime

camera  = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(1)
elif int(camera) == 2:
    from picamera2 import Picamera2
    from libcamera import controls
    picam2 = Picamera2()
    # カメラを開く
    size = (1800, 1000)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})
    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})


while True:
    # 画像をキャプチャする
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    # img = img[:,:,3]
    # 画像を表示する
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESCキーで終了
        break

# カメラを閉じる
cap.release()
# すべてのウィンドウを閉じる
cv2.destroyAllWindows()