import cv2
import sys
import time
num = 0
maxFrames = 15 # if you want 5 frames only.

try:
    vidStream = cv2.VideoCapture(0) # index of your camera
except:
    print ("problem opening input stream")
    sys.exit(1)

while num < maxFrames:
    ret, preview = vidStream.read()
    cv2.imshow("test window",preview )  # show image in window
    cv2.waitKey()
    cv2.destroyAllWindows()
    ret, frame = vidStream.read() # read frame and return code.
    if not ret: # if return code is bad, abort.
        sys.exit(0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    #img = cv2.drawChessboardCorners(frame, (9,6), corners,ret)
    cv2.imshow("img",frame)
    cv2.waitKey()
    cv2.destroyAllWindows()
    if ret == True:
        cv2.imwrite("/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/image%04i.jpg" % num, frame)
        num += 1
        print("image%02i.jpg" % num)