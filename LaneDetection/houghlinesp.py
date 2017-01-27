import cv2
import numpy as np


import numpy as np
import cv2
import math


kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)

cap = cv2.VideoCapture('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/BlueLaneVideo.mov')
trap_bottom_width = 0.85  # width of bottom edge of trapezoid, expressed as percentage of image width
trap_top_width = 0.07  # ditto for top edge of trapezoid
trap_height = 0.4  #

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
        res = cv2.bitwise_and(frame, frame, mask=mask)  # Bitwise-AND mask and original image
        cv2.imshow('res', res)  # Show result
        dilation = cv2.dilate(mask, kernel1, iterations=3)
        cv2.imshow('dilate',dilation)
        erosion = cv2.erode(dilation, kernel1, iterations=1)
        cv2.imshow('erosion', erosion)
        edged = cv2.Canny(erosion, 50, 150)  # Detect edges with Canny
        # cv2.imshow('canny', edged)
        closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
        # cv2.imshow('edgded', closing)
        minLineLength = 50
        lines = cv2.HoughLinesP(image=edged, rho=0.02, theta=np.pi / 500, threshold=5, lines=np.array([]), minLineLength=minLineLength, maxLineGap=100)
        a, b, c = lines.shape
        #print(a)
        right_lines = []
        left_lines = []
        slope_threshold = 2
        slopes = []
        newlines = []
        draw_right = True
        draw_left = True

        #for i in range(a):
         #   cv2.line(frame, (lines[i][0][0], lines[i][0][1]),(lines[i][0][2], lines[i][0][3]), (0, 255, 0), 3)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0.:  # corner case, avoiding division by 0
                slope = 999.  # practically infinite slope
            else:
                slope = (y2 - y1) / (x2 - x1)

                # Filter lines based on slope
            if abs(slope) > slope_threshold:
                slopes.append(slope)
                newlines.append(line)

        lines = newlines

        right_lines = []
        left_lines = []
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            img_x_center = frame.shape[1] / 2  # x coordinate of center of image
            if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
                right_lines.append(line)
            elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
                left_lines.append(line)

        right_lines_x = []
        right_lines_y = []

        for line in right_lines:
            x1, y1, x2, y2 = line[0]

            right_lines_x.append(x1)
            right_lines_x.append(x2)

            right_lines_y.append(y1)
            right_lines_y.append(y2)

        if len(right_lines_x) > 0:
            right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
        else:
            right_m, right_b = 1, 1
            draw_right = False

        # Left lane lines
        left_lines_x = []
        left_lines_y = []

        for line in left_lines:
            x1, y1, x2, y2 = line[0]

            left_lines_x.append(x1)
            left_lines_x.append(x2)

            left_lines_y.append(y1)
            left_lines_y.append(y2)

        if len(left_lines_x) > 0:
            left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
        else:
            left_m, left_b = 1, 1
            draw_left = False

        # Find 2 end points for right and left lines, used for drawing the line
        # y = m*x + b --> x = (y - b)/m
        y1 = frame.shape[0]
        y2 = frame.shape[0] * (0.2)

        right_x1 = (y1 - right_b) / right_m
        right_x2 = (y2 - right_b) / right_m

        left_x1 = (y1 - left_b) / left_m
        left_x2 = (y2 - left_b) / left_m

        # Convert calculated end points from float to int
        y1 = int(y1)
        y2 = int(y2)
        right_x1 = int(right_x1)
        right_x2 = int(right_x2)
        left_x1 = int(left_x1)
        left_x2 = int(left_x2)

        #print('right_x1',right_x1)
        #print('right_x2',right_x2)
        #print('left_x1',left_x1)
        #print('left_x2',left_x2)

        # Draw the right and left lines on image
        if draw_right:
            cv2.line(frame, (right_x1, y1), (right_x2, y2), [255, 0, 0], 2)
        if draw_left:
            cv2.line(frame, (left_x1, y1), (left_x2, y2), [255, 0, 0], 2)



        centerx1 = (left_x1 + right_x1) / 2
        centerx2 = (left_x2 + right_x2) / 2

        centerx1 = int(centerx1)
        centerx2 = int(centerx2)


        height,width,channels = frame.shape
        #print(height)
        centerimgx = width/2
        centerimgx = int(centerimgx)
        #print(centerimgx)
        cv2.line(frame,(centerimgx,0),(centerimgx,426),[0, 0, 0], 2)

        cv2.line(frame, (centerimgx, y1), (centerx2, y2), [0, 0, 0], 2)

        opp = centerimgx-centerx2

        adj = height - y2

        errorangle = math.atan(opp/adj)

        angle = math.degrees(errorangle)
        print(angle)

        #print('centerx2',centerx2)
        #print('y2',y2)
        cv2.imshow('lines', frame)


        cv2.waitKey(5)
except KeyboardInterrupt:
        pass