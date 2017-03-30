import Tkinter as tk
import sys



# import cv2
# import numpy as np
#
# img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/Photo on 3-2-17 at 2.28 PM.jpg')
#
# kernel_size = 3
#
# def blur(img,kernel_size):
#     return cv2.GaussianBlur(img, (kernel_size, kernel_size), 1)
#
# def hsv(img):
#     return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# def mask(img):
#     lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
#     upper_blue = np.array([130, 255, 255])
#     return cv2.inRange(img, lower_blue, upper_blue)
#
# def dilation(img):
#     return cv2.dilate(img, (4, 4), 3)
# def canny(img):
#     return cv2.Canny(img, 50, 150)
#
# def warp(img, src_pts, dst_pts):
#     M = cv2.getPerspectiveTransform(src_pts, dst_pts)
#     return cv2.warpPerspective(img, M, (516, 690))
#
# def undistort(img,mtx,dist):
#     h, w = img.shape[:2]
#     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
#     undistort = cv2.undistort(img, mtx, dist, None, newcameramtx)
#     x, y, w, h = roi
#     return undistort[y:y + h, x:x + w]
#
# def houghtransform(img,rho,theta,threshold,min_len,max_gap):
#     return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_len, max_gap)
#
# blur = blur(img,kernel_size)
# cv2.imshow('img',img)
# cv2.waitKey()

# def key_input(event):
#     print "key: ", event.char
#     key_press = event.char
#     #sleep_time =  0.05
#     if key_press == "w":
#         print "forward"
#     if key_press == "s":
#         print "reverse"
#     if key_press == "a":
#         print "left"
#     if key_press == "d":
#         print "right"
#     if key_press == "q":
#         sys.exit()
#
#
# command = tk.Tk()
# command.bind('<KeyPress>',key_input)
# command.mainloop()

import cv2
import numpy as np



img = cv2.imread('/Users/Benjamin/Downloads/hsv.png')



def laneDetection(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    lower_blue = np.array([50, 50, 130])  # define range of color in HSV
    upper_blue = np.array([90, 90, 220])
    mask = cv2.inRange(img, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
    cv2.imshow('mask', mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
    cv2.imshow('erode',erode)
    return erode

img = laneDetection(img)

size = np.size(img)
skel = np.zeros(img.shape, np.uint8)
ret, img = cv2.threshold(img, 127, 255, 0)
element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
done = False

while( not done):
    eroded = cv2.erode(img,element)
    temp = cv2.dilate(eroded,element)
    temp = cv2.subtract(img,temp)
    skel = cv2.bitwise_or(skel,temp)
    img = eroded.copy()

    zeros = size - cv2.countNonZero(img)
    if zeros==size:
        done = True

cv2.imshow("skel",skel)
cv2.waitKey(0)
cv2.destroyAllWindows()