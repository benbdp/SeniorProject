import numpy as np
import cv2
import math


kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)

cap = cv2.VideoCapture('C:/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/BlueLaneVideo.mov')
trap_bottom_width = 0.85
trap_top_width = 0.07
trap_height = 0.4

try:
    while True:
        ret, frame = cap.read()
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
        minLineLength = 50
        lines = cv2.HoughLinesP(image=edged, rho=0.02, theta=np.pi / 500, threshold=5, lines=np.array([]), minLineLength=minLineLength, maxLineGap=100)
        a, b, c = lines.shape
        right_lines = []
        left_lines = []
        slopethreshold = 2
        slopes = []
        editedlines = []
        draw_right = True
        draw_left = True

        #for i in range(a):
           # cv2.line(frame, (lines[i][0][0], lines[i][0][1]),(lines[i][0][2], lines[i][0][3]), (0, 255, 0), 3)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) > slopethreshold:
                slopes.append(slope)
                editedlines.append(line)

        lines = editedlines

        rightlanelines = []
        leftlanelines = []

        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            imgcenterx = frame.shape[1] / 2
            if slopes[i] > 0 and x1 > imgcenterx and x2 > imgcenterx:
                rightlanelines.append(line)
            elif slopes[i] < 0 and x1 < imgcenterx and x2 < imgcenterx:
                leftlanelines.append(line)

        rightlanelinesx = []
        rightlanelinesy = []

        for line in rightlanelines:
            x1, y1, x2, y2 = line[0]

            rightlanelinesx.append(x1)
            rightlanelinesx.append(x2)

            rightlanelinesy.append(y1)
            rightlanelinesy.append(y2)

        if len(rightlanelinesx) > 0:
            rightslope, rightintercept = np.polyfit(rightlanelinesx, rightlanelinesy, 1)  # y = m*x + b

        leftlanelinesx = []
        leftlanelinesy = []

        for line in leftlanelines:
            x1, y1, x2, y2 = line[0]

            leftlanelinesx.append(x1)
            leftlanelinesx.append(x2)

            leftlanelinesy.append(y1)
            leftlanelinesy.append(y2)

        if len(leftlanelinesx) > 0:
            leftslope, leftintercept = np.polyfit(leftlanelinesx, leftlanelinesy, 1)  # y = m*x + b

        height, width, channels = frame.shape
        y1 = height
        y2 = height * (0.3)

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

        cv2.line(frame, (rightx1, y1), (rightx2, y2), [0, 0, 255], 2)
        cv2.line(frame, (leftx1, y1), (leftx2, y2), [0, 0, 255], 2)

        centerlanex1 = (leftx1 + rightx1) / 2
        centerlanex2 = (leftx2 + rightx2) / 2

        centerlanex1 = int(centerlanex1)
        centerlanex2 = int(centerlanex2)
        imgcenterx = int(imgcenterx)
        cv2.line(frame,(imgcenterx,0),(imgcenterx,426),[0, 0, 0], 2)
        cv2.line(frame, (imgcenterx, y1), (centerlanex2, y2), [0, 0, 0], 2)
        opp = imgcenterx-centerlanex2
        adj = height - y2
        steeringangle = math.atan(opp/adj)
        angle = math.degrees(steeringangle)
        print('steering angle:',angle)
        cv2.imshow('lines', frame)
        cv2.waitKey(5)
except KeyboardInterrupt:
        pass