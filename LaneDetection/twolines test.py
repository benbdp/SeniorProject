import cv2
import math
import numpy as np


img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/Screen Shot 2017-02-02 at 2.06.46 PM.png')
#print kernal

#def blur(img,kernel_size):
 #   return cv2.GaussianBlur(img, (kernel_size, kernel_size), 1)

#def hsv(img):
  #  return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#def mask(img):
#    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
#    upper_blue = np.array([130, 255, 255])
#    return cv2.inRange(img, lower_blue, upper_blue)

#def dilation(img,kernal_size,iterations):
 #   return cv2.dilate(img, kernal_size, iterations)
#def canny(img):
 #   return cv2.Canny(img, 50, 150)


#cv2.imshow('img',img)
#cv2.waitKey()
#blurred = blur(img,kernel_size)
#cv2.imshow('blur',blurred)
#cv2.waitKey()
#convert_hsv = hsv(blurred)
#cv2.imshow('hsv',convert_hsv)
#cv2.waitKey()
#add_mask = mask(convert_hsv)
#cv2.imshow('mask',add_mask)
#cv2.waitKey()
#dilate = cv2.dilate(mask, kernel1, iterations=5)
#cv2.imshow('dilate',dilate)
#cv2.waitKey()
#blur = cv2.GaussianBlur(img, (5, 5), 3)
#cv2.imshow('blur', blur)
#cv2.waitKey()
#hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
#lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
#upper_blue = np.array([130, 255, 255])
#mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
#cv2.imshow('mask', mask)
#cv2.waitKey()
#dilation = cv2.dilate(mask, kernel1, iterations=5)
#cv2.imshow('dilate', dilation)
#cv2.waitKey()
#erode = cv2.erode(dilation, kernel1, iterations=3 )
#cv2.imshow('erode', erode)
#cv2.waitKey()
#edged = cv2.Canny(erode, 50, 150)
#closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
#cv2.waitKey()
#im2, contours, hierarchy = cv2.findContours(erode,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#cnt = contours[0]
#print contours
#cont = cv2.drawContours(img, contours, -1, (0,255,0), 3)
#cv2.imshow('cont',cont)
#cv2.waitKey()

#rows,cols = img.shape[:2]
#print img.shape[:1]
#vx, vy, x, y = cv2.fitLine(contours[0],cv2.DIST_L2,0,0.01,0.01)


#lefty = int((-x*vy/vx) + y)
#if lefty > img.shape[1]:
 #   lefty = img.shape[1]
#righty = int(((img.shape[1]-x)*vy/vx)+y)
#if righty < 0:
 #   righty = 0

#print righty
#print lefty
#cv2.line(img,(img.shape[1]-1,righty),(0,lefty),(0,0,255),2)
#print (img.shape[1]-1,righty),(0,lefty)
#cv2.imshow('img',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()


def laneDetection(img):
    height, width, channels = img.shape
    #print height
    center_y = height / 2
    #print center_y
    #print 'width',width
    center_x = width / 2
    #print center_x
    dilation = cv2.dilate(img, np.ones((5, 5), np.uint8), iterations=5)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
    cv2.imshow('erode',erode)
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    num_contours = len(contours)
    #print "contours",num_contours
    if num_contours < 2:
        rows, cols = img.shape[:2]
        vx0, vy0, x0, y0 = cv2.fitLine(contours[0], cv2.DIST_L2, 0, 0.01, 0.01)
        lefty0 = int((-x0 * vy0 / vx0) + y0)
        righty0 = int(((cols - x0) * vy0 / vx0) + y0)

        x_00 = float(cols - 1)
        y_00 = float(righty0)
        x_01 = float(0)
        y_01 = float(lefty0)

        slope0 = float((y_01 - y_00) / (x_01 - x_00))
        yint0 = y_01 - (slope0 * x_01)

        x0 = (height - yint0) / slope0
        x0 = int(x0)

        x1 = (0 - yint0) / slope0
        x1 = int(x1)

        adj = x0

        opp = x0 -x1
        angle = float(math.atan2((opp), adj))
        angle = math.degrees(angle)
        print "angle",angle
        return angle


    else:
        newcontours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200:
                newcontours.append(cnt)

        cv2.drawContours(img, newcontours, -1, (0, 255, 0), 3)
        #print newcontours


        rows, cols = img.shape[:2]
        vx0, vy0, x0, y0 = cv2.fitLine(newcontours[0], cv2.DIST_L2, 0, 0.01, 0.01)
        lefty0 = int((-x0 * vy0 / vx0) + y0)
        righty0 = int(((cols - x0) * vy0 / vx0) + y0)

        x_00 = float(cols-1)
        y_00 = float(righty0)
        x_01 = float(0)
        y_01 = float(lefty0)

        slope0 = float((y_01 - y_00)/(x_01-x_00))
        yint0 = y_01 - (slope0 *x_01)

        x0 = (center_y-yint0)/slope0
        x0 = int(x0)
        cv2.circle(img, (x0, center_y), 5, (0, 0, 255), -1)



        vx1, vy1, x1, y1 = cv2.fitLine(newcontours[1], cv2.DIST_L2, 0, 0.01, 0.01)
        lefty1 = int((-x1 * vy1 / vx1) + y1)
        righty1 = int(((cols - x1) * vy1 / vx1) + y1)

        x_10 = float(cols - 1)
        y_10 = float(righty1)
        x_11 = float(0)
        y_11 = float(lefty1)
        slope1 = float((y_11 - y_10) / (x_11 - x_10))
        yint1 = y_11 - (slope1 * x_11)
        x1 = (center_y - yint1) / slope1
        x1 = int(x1)
        cv2.circle(img, (x1, center_y), 5, (0, 0, 255), -1)
        center = (x0 + x1)/2
        cv2.circle(img, (center, center_y), 5, (0, 0, 255), -1)
        angle = float(math.atan2((center - center_x),center_y))
        angle = math.degrees(angle)
        print "turn angle: ", angle
        return angle

angle = laneDetection(img)

servo_pos = 70

if angle >0:
    servo_pos = servo_pos - (abs(angle) *0.8)
if angle <0:
    servo_pos = servo_pos + (abs(angle) * 0.8)

print "servo_pos: ",servo_pos



cv2.imshow('line',img)
cv2.waitKey()
#cv2.imwrite('/Users/Benjamin/Downloads/curvelines.jpg',img)



