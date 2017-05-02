import cv2
import numpy as np

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
    cv2.circle(img, (x0, center_y), 5, (0, 0, 255), -1)

def undistort(img,mtx,dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    return cv2.undistort(img, mtx, dist, None, newcameramtx)

def warp(img):
    src_pts = np.float32([[72, 227], [576, 223], [1, 316], [634, 296]])  # src
    dst_pts = np.float32([[0, 0], [556, 0], [0, 184], [556, 156]])  # dst
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return cv2.warpPerspective(img, M, (556, 156))

def mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv', hsv)
    lower_blue = np.array([40, 70, 140])  # define range of color in HSV
    upper_blue = np.array([60, 255, 255])
    return cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color

def contours():
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours