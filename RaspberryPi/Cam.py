import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
import math
ser = serial.Serial('/dev/ttyACM1', 9600)

servo_center = 83
distance_limit = 50
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIGGERRIGHT = 24
ECHORIGHT = 23
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 15
ECHOLEFT = 14
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)

mtx = np.load('/home/pi/Cal_Imgs/cameramatrix.npy')
dist = np.load('/home/pi/Cal_Imgs/distortioncoeff.npy')

junk_frames = 50
camera = cv2.VideoCapture(0)

def ultrasonicleft():
    GPIO.output(TRIGGERLEFT, False)
    time.sleep(0.5)
    GPIO.output(TRIGGERLEFT, True)
    time.sleep(0.00001)
    GPIO.output(TRIGGERLEFT, False)

    while GPIO.input(ECHOLEFT) == 0:
        start1 = time.time()

    while GPIO.input(ECHOLEFT) == 1:
        stop1 = time.time()

    elapsed1 = stop1 - start1
    left_distance = (elapsed1 * 34300) / 2


    return left_distance

def ultrasonicright():
    GPIO.output(TRIGGERRIGHT, False)
    time.sleep(0.5)
    GPIO.output(TRIGGERRIGHT, True)
    time.sleep(0.00001)
    GPIO.output(TRIGGERRIGHT, False)

    while GPIO.input(ECHORIGHT) == 0:
        start2 = time.time()

    while GPIO.input(ECHORIGHT) == 1:
        stop2 = time.time()

    elapsed1 = stop2 - start2
    right_distance = (elapsed1 * 34300) / 2

    return right_distance

def get_image():
    retval, img = camera.read()
    return img

for i in xrange(junk_frames):
    temp = get_image()

capture = temp
#cv2.imshow('frame', frame)
h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
cv2.imshow('undistort', undistort)
src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(undistort,M,(558,154))
cv2.imshow('warp', dst_img)
hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
cv2.imshow('hsv', hsv)
lower_blue = np.array([40, 70, 140])  # define range of color in HSV
upper_blue = np.array([60,255,255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
cv2.imshow('mask', mask)
# height, width, channels = dst_img.shape
# # print height
# center_y = height / 2
# # print center_y
# # print width
# center_x = width / 2
# # print center_x
# dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)
# erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
# # cv2.imshow('erode',erode)
# im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# # print contours[0]
#
# newcontours = []
# for cnt in contours:
#     area = cv2.contourArea(cnt)
#     if area > 1:
#         newcontours.append(cnt)
# cv2.drawContours(dst_img, newcontours, -1, (0, 255, 0), 3)
# # print newcontours
#
# print "Found two lines"
#
# rows, cols = dst_img.shape[:2]
# vx0, vy0, x0, y0 = cv2.fitLine(newcontours[0], cv2.DIST_L2, 0, 0.01, 0.01)
# lefty0 = int((-x0 * vy0 / vx0) + y0)
# righty0 = int(((cols - x0) * vy0 / vx0) + y0)
#
# x_00 = float(cols - 1)
# y_00 = float(righty0)
# x_01 = float(0)
# y_01 = float(lefty0)
#
# slope0 = float((y_01 - y_00) / (x_01 - x_00))
#
# print"slope0: ", slope0
# yint0 = y_01 - (slope0 * x_01)
#
# x0 = (center_y - yint0) / slope0
# x0 = int(x0)
# cv2.circle(dst_img, (x0, center_y), 5, (0, 0, 255), -1)
#
# vx1, vy1, x1, y1 = cv2.fitLine(newcontours[1], cv2.DIST_L2, 0, 0.01, 0.01)
# lefty1 = int((-x1 * vy1 / vx1) + y1)
# righty1 = int(((cols - x1) * vy1 / vx1) + y1)
#
# x_10 = float(cols - 1)
# y_10 = float(righty1)
# x_11 = float(0)
# y_11 = float(lefty1)
# slope1 = float((y_11 - y_10) / (x_11 - x_10))
# print"slope1: ",slope1
# yint1 = y_11 - (slope1 * x_11)
# x1 = (center_y - yint1) / slope1
# x1 = int(x1)
# cv2.circle(dst_img, (x1, center_y), 5, (0, 0, 255), -1)
# center = (x0 + x1) / 2
# cv2.circle(dst_img, (center, center_y), 5, (0, 0, 255), -1)
# angle = float(math.atan2((center - center_x), center_y))
# angle = math.degrees(angle)
# # print "turn angle:", angle
# # cv2.imshow("img",img)
# print "amount to turn: ",angle
# term = 0.3
# servo_pos = 70
#
# if angle > 0:
#     servo_pos = servo_pos + (abs(angle) * term)
# if angle < 0:
#     servo_pos = servo_pos - (abs(angle) * term)
#
# print "servo_pos", servo_pos
#
# print(str(70) + str('m,') + str(servo_pos) + str('s,'))
# ser.write(str(72) + str('m,') + str(servo_pos) + str('s,'))
# time.sleep(0.5)
#
# print(str(0) + str('m,') + str(70) + str('s,'))
# ser.write(str(0) + str('m,') + str(70) + str('s,'))
# cv2.imshow("img",dst_img)
cv2.waitKey()
cv2.destroyAllWindows()