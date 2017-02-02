import cv2
import sys

try:
    vidStream = cv2.VideoCapture(0) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)

while True:
    ret, preview = vidStream.read()
    cv2.imshow("test window",preview )
    frame = cv2.resize(preview,(640,360))
    cv2.imshow('frame',frame)
    undistort = cv2.undistort()


    cv2.waitKey(5)