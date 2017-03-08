import cv2
import sys
import numpy as np

try:
    vidStream = cv2.VideoCapture(0) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)


num = 100

while num > 0:
    num = num - 1
    ret, frame = vidStream.read()
    print frame.shape
    cv2.imshow('undistort', frame)
    cv2.waitKey(3)