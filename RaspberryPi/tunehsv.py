# import the necessary packages
from imutils.video import WebcamVideoStream
import imutils
import cv2
import numpy as np

mtx = np.load('/home/pi/SeniorProject/RaspberryPi/cameramatrix.npy')
dist = np.load('/home/pi/SeniorProject/RaspberryPi/distortioncoeff.npy')
# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter

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
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)  # undistort image
    src_pts = np.float32([[87, 176], [552, 177], [4, 301], [638, 302]])  # source points
    dst_pts = np.float32([[0, 0], [558, 0], [0, 264], [558, 264]])  # destination points
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warp = cv2.warpPerspective(undistort, M, (558, 264))  # warp the image

    #converting to HSV
    hsv = cv2.cvtColor(warp,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')

    # Normal masking algorithm
    lower = np.array([h,s,v])
    upper = np.array([180,255,255])


    # fooling with mask
    mask = cv2.inRange(hsv,lower, upper)
    # cv2.imshow("msk",mask)
    # h, w = mask.shape[:2]
    # x, y, w, h = h,w,h/2,w
    crop_img = mask[200:400, 100:300]
    dilation = cv2.dilate(crop_img, np.ones((5, 5), np.uint8), iterations=2)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=1)
    cv2.imshow("erode", erode)

    result = cv2.bitwise_and(warp,warp,mask = mask)

    cv2.imshow('result',result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        np.save('/home/pi/SeniorProject/RaspberryPi/hsv.npy',lower)
        print "h: ", h, " s: ", s, " v: ", v
        break

cv2.destroyAllWindows()
vs.stop()