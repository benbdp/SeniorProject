import cv2
import sys
import time
num = 0
maxFrames = 20 # if you want 5 frames only.

try:
    vidStream = cv2.VideoCapture(1) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)

while num < maxFrames:
    retval, frame = vidStream.read()
    cv2.imshow("img",frame)
    cv2.waitKey()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    #img = cv2.drawChessboardCorners(frame, (9,6), corners,ret)
    if ret == True:
        cv2.imwrite("/home/pi/Cal_Imgs/image%04i.jpg" % num, frame)
        num += 1
        print("image%02i.jpg" % num)