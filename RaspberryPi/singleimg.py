import cv2
import sys


try:
    cam = cv2.VideoCapture(0) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)

while True:
    ret, frame = cam.read()
    cv2.imshow("frame",frame)
    cv2.waitKey(5)
