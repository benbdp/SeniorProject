import cv2
import sys


try:
    vidStream = cv2.VideoCapture(0) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)

