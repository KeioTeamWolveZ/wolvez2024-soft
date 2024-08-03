"""
Take a movie
"""

from picamera2 import Picamera2
import cv2
picam2 = Picamera2()
size = (1800, 2400)
config = picam2.create_preview_configuration(
main={"format": 'XRGB8888', "size": size})
picam2.align_configuration(config)
picam2.configure(config)
picam2.start()
# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
picam2.set_controls({"AfMode":0,"LensPosition":5.5})
#movie (10s)
picam2.start_and_record_video("test.mp4", duration=10)
