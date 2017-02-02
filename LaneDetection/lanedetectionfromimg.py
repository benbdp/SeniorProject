import cv2
import sys
import numpy as np
import math

mtx = np.matrix([[  1.09737118e+03, 0.00000000e+00, 6.29382303e+02], [  0.00000000e+00, 1.10083151e+03, 3.71037449e+02], [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.matrix([[  1.37334519e-01, -1.20441566e+00, 2.19553714e-03, -4.06071434e-04, 2.18048197e+00]])


kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)
trap_bottom_width = 0.85
trap_top_width = 0.07
trap_height = 0.4
src_img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/CameraCalibration/undistort.png')
while True:
    cv2.imshow('src_img', src_img)
    src_pts = np.float32([[343, 386], [857, 386], [0, 558], [1256, 585]])  # src
    dst_pts = np.float32([[0, 0], [513, 0], [0, 660], [516, 690]])  # dst
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    dst_img = cv2.warpPerspective(src_img, M, (516, 690))

    cv2.imshow('dst', dst_img)
    cv2.imshow('preview',dst_img)

    blur = cv2.GaussianBlur(dst_img, (5, 5), 3)
    cv2.imshow('blur', blur)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
    cv2.imshow('mask', mask)
    dilation = cv2.dilate(mask, kernel1, iterations=5)
    cv2.imshow('dilate', dilation)
    #erosion = cv2.erode(dilation, kernel1, iterations=1)
    #cv2.imshow('erosion', erosion)
    edged = cv2.Canny(dilation, 50, 150)
    closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
    cv2.imshow('edged',edged)
    minLineLength = 50
    lines = cv2.HoughLinesP(image=edged, rho=0.02, theta=np.pi / 500, threshold=5, lines=np.array([]),minLineLength=minLineLength, maxLineGap=100)
    a, b, c = lines.shape
    print(a)

    for i in range(a):
        cv2.line(dst_img, (lines[i][0][0], lines[i][0][1]),(lines[i][0][2], lines[i][0][3]), (0, 255, 0), 3)
    cv2.imshow('frame',dst_img)


    rightlanelines = []
    leftlanelines = []

    for i, line in enumerate(lines):
        x1, y1, x2, y2 = line[0]
        print('x1',x1)
        print('x2',x2)
        imgcenterx = dst_img.shape[1] / 2

        print('imgcenterx',imgcenterx)
        if x1 < imgcenterx and x2 < imgcenterx:
            leftlanelines.append(line)
        elif x1 > imgcenterx and x2 > imgcenterx:
            rightlanelines.append(line)

    print('rightlanelines',rightlanelines)
    print('leftlanelines',leftlanelines)

    rightlanelinesx = []
    rightlanelinesy = []

    for line in rightlanelines:
        x1, y1, x2, y2 = line[0]

        rightlanelinesx.append(x1)
        rightlanelinesx.append(x2)

        rightlanelinesy.append(y1)
        rightlanelinesy.append(y2)

        print('rightlanelinesx',rightlanelinesx)
        print('rightlanelinesy', rightlanelinesy)


    if len(rightlanelinesx) > 0:
        rightslope, rightintercept = np.polyfit(rightlanelinesx, rightlanelinesy, 1)  # y = m*x + b

        print('rightslope',rightslope)
        print('rightintercept', rightintercept)
    leftlanelinesx = []
    leftlanelinesy = []

    for line in leftlanelines:
        x1, y1, x2, y2 = line[0]

        leftlanelinesx.append(x1)
        leftlanelinesx.append(x2)

        leftlanelinesy.append(y1)
        leftlanelinesy.append(y2)


    print('leftlanelinesx',leftlanelinesx)
    print('leftlanelinesy',leftlanelinesy)

    if len(leftlanelinesx) > 0:
        leftslope, leftintercept = np.polyfit(leftlanelinesx, leftlanelinesy, 1)  # y = m*x + b

        print('leftslope',leftslope)
        print('leftintercept', leftintercept)

    height, width, channels = dst_img.shape
    y1 = height
    y2 = height * (0.1)

    rightx1 = (y1 - rightintercept) / rightslope
    rightx2 = (y2 - rightintercept) / rightslope

    leftx1 = (y1 - leftintercept) / leftslope
    leftx2 = (y2 - leftintercept) / leftslope

    y1 = int(y1)
    y2 = int(y2)
    rightx1 = int(rightx1)
    rightx2 = int(rightx2)
    leftx1 = int(leftx1)
    leftx2 = int(leftx2)

    cv2.line(dst_img, (rightx1, y1), (rightx2, y2), [0, 0, 255], 2)
    cv2.line(dst_img, (leftx1, y1), (leftx2, y2), [0, 0, 255], 2)

    centerlanex1 = (leftx1 + rightx1) / 2
    centerlanex2 = (leftx2 + rightx2) / 2

    centerlanex1 = int(centerlanex1)
    centerlanex2 = int(centerlanex2)
    imgcenterx = int(imgcenterx)
    cv2.line(dst_img, (imgcenterx, 0), (imgcenterx, 100), [0, 0, 0], 2)
    cv2.line(dst_img, (imgcenterx, y1), (centerlanex2, y2), [0, 0, 0], 2)
    opp = imgcenterx - centerlanex2
    adj = height - y2
    steeringangle = math.atan(opp / adj)
    angle = math.degrees(steeringangle)
    print('steering angle:', angle)
    cv2.imshow('lines', dst_img)


    cv2.waitKey()




