import numpy as np
import cv2
import math

mtx = np.matrix([[  1.09737118e+03, 0.00000000e+00, 6.29382303e+02], [  0.00000000e+00, 1.10083151e+03, 3.71037449e+02], [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.matrix([[  1.37334519e-01, -1.20441566e+00, 2.19553714e-03, -4.06071434e-04, 2.18048197e+00]])



kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)

cap = cv2.VideoCapture('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/Movie on 2-2-17 at 1.58 AM.mov')

trap_bottom_width = 0.85
trap_top_width = 0.07
trap_height = 0.4

try:
    while True:
        ret, frame = cap.read()
        h, w = frame.shape[:2]
        print(h,w)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        undistort = undistort[y:y + h, x:x + w]
        cv2.imshow('undistort', undistort)
        src_pts = np.float32([[343, 386], [857, 386], [0, 558], [1256, 585]])  # src

        dst_pts = np.float32([[0, 0], [513, 0], [0, 660], [516, 690]])  # dst

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        dst_img = cv2.warpPerspective(frame, M, (516, 690))
        cv2.imshow('preview', dst_img)




        cv2.imshow('frame',frame)
        blur = cv2.GaussianBlur(frame, (5, 5), 3)
        cv2.imshow('blur', blur)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
        lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
        cv2.imshow('mask', mask)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('res', res)  # Show result
        dilation = cv2.dilate(mask, kernel1, iterations=3)
        cv2.imshow('dilate',dilation)
        erosion = cv2.erode(dilation, kernel1, iterations=1)
        cv2.imshow('erosion', erosion)
        edged = cv2.Canny(erosion, 50, 150)
        closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)

        cv2.waitKey(5)
except KeyboardInterrupt:
        pass