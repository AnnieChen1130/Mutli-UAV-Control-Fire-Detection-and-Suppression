import cv2 as cv
import numpy as np
from datetime import datetime
import time
import dronekit as dk
from pymavlink import mavutil
import os

dir_original = 'ORIGINAL'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")


fourcc = cv.VideoWriter_fourcc(*'XVID')
out_original = cv.VideoWriter(os.path.join(dir_original, time_stamp + '.avi'), fourcc, 8.0, (640, 480) )

cam = cv.VideoCapture(0)
while True:
    ret, img = cam.read()
    out_original.write(img)

    cv.imshow("OpticalFlow", img)  # displaying image with flow on it, for illustration purposes

    key = cv.waitKey(30)
    if key == ord('q'):
        out_original.release()
        break
