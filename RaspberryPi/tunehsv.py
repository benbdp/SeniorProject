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

def line(img,contours,center_y):
    rows, cols = img.shape[:2]
    vx0, vy0, x0, y0 = cv2.fitLine(contours, cv2.DIST_L2, 0, 0.01, 0.01)
    lefty0 = int((-x0 * vy0 / vx0) + y0)
    righty0 = int(((cols - x0) * vy0 / vx0) + y0)
    x_00 = float(cols - 1)
    y_00 = float(righty0)
    x_01 = float(0)
    y_01 = float(lefty0)
    slope0 = float((y_01 - y_00) / (x_01 - x_00))
    yint0 = y_01 - (slope0 * x_01)
    x0 = (center_y - yint0) / slope0
    x0 = int(x0)
    return x0

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
    src_pts = np.float32([[59, 228], [568, 227], [3, 305], [625, 305]])  # source points
    dst_pts = np.float32([[0, 0], [558, 0], [0, 154], [558, 154]])  # destination points
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warp = cv2.warpPerspective(undistort, M, (558, 154))  # warp the image

    #converting to HSV
    hsv = cv2.cvtColor(warp,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')

    # Normal masking algorithm
    lower_blue = np.array([h,s,v])
    upper_blue = np.array([180,255,255])


    # fooling with mask
    mask = cv2.inRange(hsv,lower_blue, upper_blue)
    # cv2.imshow("msk",mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=2)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=1)
    cv2.imshow("erode", erode)
    # im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # newcontours = []
    # for cnt in contours:
    #     area = cv2.contourArea(cnt)
    #     if area > 10000:
    #         newcontours.append(cnt)
    #         area = cv2.contourArea(cnt)
    #         # print(area)
    #
    # x, y, w, h = cv2.boundingRect(newcontours[0])
    # cv2.rectangle(warp,(x,y),(x+w,y+h),(255,0,0),2)
    # cv2.imshow("circles",warp)

    result = cv2.bitwise_and(warp,warp,mask = mask)

    cv2.imshow('result',result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        print("h: ", h, " s: ", s, " v: ", v)
        break
    if k == 32:
        num=0
        im,contours, hier = cv2.findContours(erode, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        newcontours = []
        points = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # run test to ensure small contours are eliminated
                newcontours.append(cnt)
        for cnt in newcontours:
            rows, cols = warp.shape[:2]
            # then apply fitline() function
            [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
            # Now find two extreme points on the line to draw line
            lefty = int((-x * vy / vx) + y)
            righty = int(((warp.shape[1] - x) * vy / vx) + y)
            x_0 = float(cols - 1)
            y_0 = float(righty)
            x_1 = float(0)
            y_1 = float(lefty)
            mid = rows / 2
            slope = float((x_1 - x_0) /(y_1 - y_0) )
            print("slope%d: " %num,slope)
            yint0 = y_1 -(slope * x_1)
            x0 = line(warp,cnt,mid)

            points.append(x0)
            cv2.circle(warp, (x0, mid), 5, (0, 0, 255), -1)



            # Finally draw the line
            cv2.line(warp, (warp.shape[1] - 1, righty), (0, lefty), 255, 2)
            num =+ 1
        if len(newcontours)==2:
            rows, cols = warp.shape[:2]
            dist = (points[0]+points[1])/2
            print dist
            center = cols/2
            print center

        cv2.imshow("lines",warp)

cv2.destroyAllWindows()
vs.stop()