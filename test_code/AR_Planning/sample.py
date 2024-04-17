import cv2
from picamera2 import Picamera2
from libcamera import controls
from datetime import datetime

picam2 = Picamera2()
# カメラを開く
# cap = cv2.VideoCapture(1)
size = (1800, 1000)
config = picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": size})
picam2.align_configuration(config)
picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})


while True:
    # 画像をキャプチャする
    # ret, frame = cap.read()
    img = picam2.capture_array()
    # img = img[:,:,3]
    # 画像を表示する
    cv2.imshow("Image", img)

    # `q`キーを押すとループを終了する
    if cv2.waitKey(1) == ord('q'):
        break

# カメラを閉じる
cap.release()
# すべてのウィンドウを閉じる
cv2.destroyAllWindows()