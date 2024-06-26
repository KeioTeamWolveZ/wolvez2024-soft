import cv2
from picamera2 import Picamera2
from libcamera import controls
import numpy as np
import time

# setting picam2 up
picam2 = Picamera2()
# setting pix
"""
picam2.senser_mode = 2
picam2.resolution = (4000, 3600)
"""
#capture_config = picam2.create_still_configuration(main={"size": (640, 480), "format": 'XRGB8888'}, raw={"size": picam2.sensor_resolution})
#config = picam2.create_preview_configuration(main={"format": 'XRGB8888'})
#config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (4000, 3600)})
capture_config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1200, 720)})
#picam2.align_configuration(config)
picam2.configure(capture_config)

picam2.start()


#Libcamera's setting to use AF mode
#picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
#Libcamera's setting to use AF mode (AfSpeed Fast)
#picam2.set_controls({"AFSpeed":controls.AfSpeedEnum.Fast})
# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous,"AfSpeed":controls.AfSpeedEnum.Fast})
picam2.set_controls({"AfMode":0,"LensPosition":3.5})
#10 : 10cm
#9 : 13cm
#8 : 15cm
#7 : 18cm
#6 : 23cm
#5 : 38cm
#4 : 40cm
#Display for input data in real -time
k = 1 
start = time.time()
t = time.time()
i = 0
lens = [i*0.5 for i in range(10,14)]
print(lens)
while True:
  picam2.set_controls({"AfMode":0,"LensPosition":3})
  
  im = picam2.capture_array()
  im = cv2.resize(im,None,fx=0.8,fy=0.8)
  cv2.imshow("Camera", im)
  key = cv2.waitKey(1)
  k += 1
  #If you push "esc-key", this roop is finished.
  if key == 27:
    end = time.time()
    print(f"FPS: {k/(end-start):.2f}")
    cv2.imwrite("test_cv2.jpg", im)
    print(np.shape(im))
    break
picam2.stop()
cv2.destroyAllWindows()

