import cv2
import numpy as np

cap = cv2.VideoCapture('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/Movie on 2-2-17 at 1.58 AM.mov')


def blur(img):
    return cv2.GaussianBlur(img, (5, 5), 3)

def hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
def mask(img):
    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
    upper_blue = np.array([130, 255, 255])
    return cv2.inRange(img, lower_blue, upper_blue)

def dilation(img):
    return cv2.dilate(img, (4, 4), 3)
def canny(img):
    return cv2.Canny(img, 50, 150)

def warp(img, src_pts, dst_pts):
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return cv2.warpPerspective(img, M, (516, 690))

def undistort(img,mtx,dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistort = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    return undistort[y:y + h, x:x + w]

def houghtransform(img,rho,theta,threshold,min_len,max_gap):
    return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_len, max_gap)

try:
    while True:
        ret, frame = cap.read()

except KeyboardInterrupt:
        pass