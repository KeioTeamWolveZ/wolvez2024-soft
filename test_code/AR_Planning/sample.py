import cv2

# カメラを開く
cap = cv2.VideoCapture(1)

while True:
    # 画像をキャプチャする
    ret, frame = cap.read()

    # 画像を表示する
    cv2.imshow("Image", frame)

    # `q`キーを押すとループを終了する
    if cv2.waitKey(1) == ord('q'):
        break

# カメラを閉じる
cap.release()
# すべてのウィンドウを閉じる
cv2.destroyAllWindows()