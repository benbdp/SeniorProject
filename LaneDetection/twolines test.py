import cv2
import math
import numpy as np


img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Screen Shot 2017-02-02 at 2.06.46 PM.png')
kernel_size = 3
kernel1 = np.ones((4, 4), np.uint8)
kernel2 = np.ones((9, 9), np.uint8)

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
blur = cv2.GaussianBlur(img, (5, 5), 3)
cv2.imshow('blur', blur)
cv2.waitKey()
hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
upper_blue = np.array([130, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
cv2.imshow('mask', mask)
cv2.waitKey()
dilation = cv2.dilate(mask, kernel1, iterations=5)
cv2.imshow('dilate', dilation)
cv2.waitKey()
edged = cv2.Canny(dilation, 50, 150)
closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
cv2.imshow('edged',edged)
cv2.waitKey()
im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[0]
print contours
cont = cv2.drawContours(img, contours, -1, (0,255,0), 3)
cv2.imshow('cont',cont)
cv2.waitKey()
#rows,cols = img.shape[:2]
#[vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
#lefty = int((-x*vy/vx) + y)
#righty = int(((cols-x)*vy/vx)+y)
#cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
#cv2.imshow('line',img)
#cv2.waitKey()