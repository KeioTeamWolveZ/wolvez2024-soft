import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
import time

class Color_tools:
    def __init__(self,lower,upper):
        self.height = 0
        self.width = 0

        self.lower_orange = np.array([26, 15, 10])
        self.upper_orange = np.array([63, 255, 175])

        self.MAX_CONTOUR_THRESHOLD = 1000
        self.cap = cv2.VideoCapture(1)

    def mask_color(self,frame,lower_orange,upper_orange):
        """
        オレンジ色のマスクを作成
        input: frame, lower_orange, upper_orange
        output: mask_orange
        """
        ss = time.time()
        # オレンジ色の検出
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        # print("mask_color_time:",time.time()-ss)
        return mask_orange

    def detect_color(self,mask_orange,MAX_CONTOUR_THRESHOLD):
        """
        オレンジ色の物体を検出し、その中心座標を返す
        input: mask_frame, MAX_CONTOUR_THRESHOLD
        output: cx, cy, max_contour_area
        """
        # 輪郭を抽出して最大の面積を算出し、線で囲む
        ss = time.time()
        cX = None
        cY = None
        max_contour_area = None

        
        # 形態学的処理（膨張と収縮）を追加
        kernel = np.ones((8,8), np.uint8)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)

        
        contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > MAX_CONTOUR_THRESHOLD:  # 面積が1000より大きい場合のみ描画
                cv2.drawContours(mask_orange, [max_contour], -1, (0, 255, 0), 3)
                M = cv2.moments(max_contour)
                max_contour_area = cv2.contourArea(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    mask_orange = cv2.circle(mask_orange, (cX, cY), 7, (255, 0, 0), -1)
                    if cX > self.width/2:
                        # print("--- right color ---")
                        pass
                    else:
                        # print("--- left color ---")
                        pass
            else:
                # print("--- no color ---")
                pass
        else:
            # print("--- no color ---")
            pass
        # print("detect_color_time:",time.time()-ss)
        return mask_orange,cX,cY,max_contour_area

    def main_loop(self):
        """
        カメラ画像を取得し、オレンジ色の物体を検出する    
        """
        while True:
            # カメラ画像の取得
            ss = time.time()
            ret, frame = self.cap.read()
            self.height = frame.shape[0]
            self.width = frame.shape[1]
            # print("camera_time:",time.time()-ss)
            
            frame_resized = cv2.resize(frame, None, fx=0.5, fy=0.5)
            cv2.imshow('frame', frame_resized)

            # ============================== Color_toolsの使い方 =============================
            # オレンジ色のマスクを作成
            mask_orange = self.mask_color(frame,self.lower_orange,self.upper_orange)
            # 輪郭を抽出して最大の面積を算出し、線で囲む
            mask_orange,cX,cY,max_contour_area = self.detect_color(mask_orange,self.MAX_CONTOUR_THRESHOLD)
            print("cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)

            # ====================================結果の表示===================================
            # 画像のリサイズを行う
            mask_orange = cv2.resize(mask_orange, None, fx=0.5, fy=0.5)
            cv2.imshow('masked', mask_orange)

            key = cv2.waitKey(1)  # キー入力の受付
            print("\n")
            if key == 27:  # ESCキーで終了
                self.cap.release()
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    lower_orange = np.array([26, 15, 10])
    upper_orange = np.array([63, 255, 175])
    color_tools = Color_tools(lower_orange,upper_orange)
    color_tools.main_loop()