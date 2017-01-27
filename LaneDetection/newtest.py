import numpy as np
import cv2
import math
import itertools

kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)

cap = cv2.VideoCapture('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/lanelines.mov')

num = 0
try:
    while True:
        num += 1
        #print(num)
        ret, frame = cap.read()
        cv2.imshow('frame',frame)
        #print(ret)
        blur = cv2.GaussianBlur(frame, (5, 5), 3)
        cv2.imshow('blur',blur)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
        lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
        cv2.imshow('mask', mask)
        res = cv2.bitwise_and(frame, frame, mask=mask)  # Bitwise-AND mask and original image
        cv2.imshow('res',res)   # Show result
        dilation = cv2.dilate(mask, kernel1, iterations=1)
        #cv2.imshow('dilate',dilation)
        erosion = cv2.erode(dilation, kernel1, iterations=1)
        #cv2.imshow('erosion', erosion)
        edged = cv2.Canny(erosion, 50, 150)  # Detect edges with Canny
        #cv2.imshow('canny', edged)
        closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
        #cv2.imshow('edgded', closing)
        edges = cv2.Canny(erosion, 50, 150)
        cv2.imshow('final',edges)
        lines = cv2.HoughLines(edges, 1, math.pi / 180.0, 80, np.array([]), 0, 0)
        a,b,c = lines.shape
        print('a',a)
        right_lines = []
        left_lines = []
        for i in range(a):
            rho = lines[i][0][0]
            print ('rho',rho)
            if rho > 0:
                right_lines.append()
                print('right_lines', right_lines)
            elif rho < 0:
                left_lines.append()
                print('left_lines',left_lines)



            if rho > 0:
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)

                #print('a',a)
                #print('b',b)
                x0, y0 = a * rho, b * rho
                #print ('x0',x0)
                #print ('y0',y0)
                pt1_x1,pt1_y1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))

                #print('point1',point1)
                pt2_x1, pt2_y1 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(frame, (pt1_x1,pt1_y1), (pt2_x1, pt2_y1), (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow('lines', frame)
            if rho < 0:
                theta = lines[i][0][1]
                # print('theta',theta)
                a = math.cos(theta)

                # print('a',a)
                b = math.sin(theta)
                # print('b',b)
                x0, y0 = a * rho, b * rho
                # print ('x0',x0)
                # print ('y0',y0)
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                # print('pt2',pt2)
                cv2.line(frame, pt1, pt2, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.imshow('lines', frame)
        cv2.waitKey(5)
except KeyboardInterrupt:
        pass