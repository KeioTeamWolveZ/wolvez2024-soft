# -*- coding: utf-8 -*-
import sys
import time
import csv
import cv2
import numpy as np
import RPi.GPIO as GPIO

import bno055
from picamera2 import Picamera2
from cv2 import aruco

class ML_position:
    def __init__(self):
        # Initialize GPIO and BNO055 sensor
        GPIO.setwarnings(False)
        self.bno = bno055.BNO055()
        self.bno.setupBno()

        # Load camera matrix and distortion coefficients
        self.camera_matrix = np.load("mtx.npy")
        self.distortion_coeff = np.load("dist.npy")
        self.marker_length = 0.0165
        
        self.cnt = 0

        # Initialize the camera
        self.picam2 = Picamera2()
        size = (1800, 2400)
        config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": size})
        self.picam2.align_configuration(config)
        self.picam2.configure(config)
        self.picam2.start()

        # Set up ArUco marker detection
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

        # Open a CSV file for writing
        self.csv_file = open("datalog3.csv", mode="w", newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write CSV header
        self.csv_writer.writerow(["cnt","tvec_x", "tvec_y", "tvec_z", "ax", "ay", "az", "gx", "gy", "gz", "center_x", "center_y"])

        # Initialize timing variables
        self.last_adjustment_time = time.time()
        self.adjustment_interval = 1  # 10 seconds

    def capture(self):
        frame = self.picam2.capture_array()
        frame2 = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return frame2

    def logging(self, lens_position):
        # Set camera lens position
        self.picam2.set_controls({"AfMode": 0, "LensPosition": lens_position})

        # Detect ArUco marker and obtain tvec
        img = self.capture()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tvecs, rvecs, marker_centers = self.detect_marker(gray)

        for i in range(len(tvecs)):
            cv2.circle(img, (int(marker_centers[i][0]), int(marker_centers[i][1])), 30, (0, 255, 0), 5)
          
            # Read accelerometer values
            self.bno.bnoread()
            ax = round(self.bno.ax, 4)
            ay = round(self.bno.ay, 4)
            az = round(self.bno.az, 4)
            gx = round(self.bno.gx, 4)
            gy = round(self.bno.gy, 4)
            gz = round(self.bno.gz, 4)
            
            # Store data in a list, including image plane coordinates
            datalog = [self.cnt, tvecs[i][0], tvecs[i][1], tvecs[i][2], ax, ay, az, gx, gy, gz, marker_centers[i][0], marker_centers[i][1]]
            cv2.imwrite(f"/home/wolvez2024/wolvez2024-soft/test_code/learning/img/test_{self.cnt}.jpg",img)
            self.cnt+=1
            # Write datalog to CSV
            self.csv_writer.writerow(datalog)
        
        resized_frame = cv2.resize(img, None, fx=0.3, fy=0.3)
        cv2.imshow("resized_frame", resized_frame)

    def detect_marker(self, img):
        # Detect ArUco markers in the image
        corners, ids, rejected = aruco.detectMarkers(img, self.dictionary)
        tvecs, rvecs, centers = [], [], []
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] in [1, 2, 3, 4, 5, 6]:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.distortion_coeff)
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)

                    # Calculate the center of the marker in the image plane
                    corner_points = corners[i][0]
                    marker_center = np.mean(corner_points, axis=0)
                    print(f"tvec: {tvec}, rvec: {rvec}, center: {marker_center}")
                    tvecs.append(tvec)
                    rvecs.append(rvec)
                    centers.append(marker_center)

            return tvecs, rvecs, centers
        # Return default values if no markers are detected
        return [], [], []

    def run(self):
        try:
            lens_position = 10  # Start lens position
            while True:
                current_time = time.time()
                # Adjust lens position if 10 seconds have passed
                self.logging(lens_position)
                # Adjust lens position
                if current_time - self.last_adjustment_time >= self.adjustment_interval:
                    lens_position -= 0.5
                    print("===============",lens_position,"===============")
                    if lens_position < 3:
                        lens_position = 10.5  # Reset lens position after reaching the end

                    # Update the last adjustment time
                    self.last_adjustment_time = current_time

                key = cv2.waitKey(1)  # Key input reception
                if key == 27:  # Exit on ESC key
                    break
        finally:
            # Close CSV file on exit
            self.csv_file.close()

if __name__ == '__main__':
    ml = ML_position()
    ml.run()
