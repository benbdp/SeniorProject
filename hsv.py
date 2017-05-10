#Import necessary packages
from imutils.video import WebcamVideoStream
import imutils
import cv2
import numpy as np

vs = WebcamVideoStream(src=0).start()
def nothing(x):
    pass



# Creating a window for later use
cv2.namedWindow('result')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar
cv2.createTrackbar('h', 'result',0,179,nothing)
cv2.createTrackbar('s', 'result',0,255,nothing)
cv2.createTrackbar('v', 'result',0,255,nothing)

while True:

    frame = vs.read()
    frame = imutils.resize(frame, width=640)
    h, w = frame.shape[:2]

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')

    # Normal masking algorithm
    lower_range = np.array([h,s,v])
    np.save('/Users/Benjamin/PycharmProjects/SeniorProject/tuned_vals.npy',lower_range)
    upper_range = np.array([180,255,255])


    # fooling with mask
    mask = cv2.inRange(hsv,lower_range, upper_range)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("mask",mask)
    cv2.imshow('result', result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        print("h: ", h, " s: ", s, " v: ", v)
        break


cv2.destroyAllWindows()
vs.stop()