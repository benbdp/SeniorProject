import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
import math
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 86
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

def line(img,contours,center_y):
    rows, cols = img.shape[:2]
    vx0, vy0, x0, y0 = cv2.fitLine(contours, cv2.DIST_L2, 0, 0.01, 0.01)
    lefty0 = int((-x0 * vy0 / vx0) + y0)
    righty0 = int(((cols - x0) * vy0 / vx0) + y0)
    x_00 = float(cols - 1)
    y_00 = float(righty0)
    x_01 = float(0)
    y_01 = float(lefty0)
    slope0 = float((y_01 - y_00) / (x_01 - x_00))
    yint0 = y_01 - (slope0 * x_01)
    x0 = (center_y - yint0) / slope0
    x0 = int(x0)
    cv2.circle(img, (x0, center_y), 5, (0, 0, 255), -1)
    return x0

def get_image():
    retval, img = camera.read()
    return img

def forward(sec):
    ser.write(str(100) + str('m,') + str(servo_center) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))

def left(sec):
    ser.write(str(100) + str('m,') + str(servo_center-10) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))

def right(sec):
    ser.write(str(100) + str('m,') + str(servo_center+10) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))

def contours(img): # img should be wrapped image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv', hsv)
    lower_blue = np.array([50, 50, 130])  # define range of color in HSV
    upper_blue = np.array([95, 140, 220])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
    cv2.imshow('mask', mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)
    size = np.size(dilation)
    skel = np.zeros(dilation.shape, np.uint8)
    ret, img = cv2.threshold(dilation, 127, 255, 0)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    done = False

    while (not done):
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()

        zeros = size - cv2.countNonZero(img)
        if zeros == size:
            done = True

    cv2.imshow("skel",skel)
    im2, contours, hierarchy = cv2.findContours(skel, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def lane_detection(img):
    #cv2.imshow('frame', frame)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistort = cv2.undistort(img, mtx, dist, None, newcameramtx)
    cv2.imshow('undistort', undistort)
    src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

    dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    dst_img = cv2.warpPerspective(undistort,M,(558,154))
    cv2.imshow('warp', dst_img)
    hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv', hsv)
    lower_blue = np.array([50, 50, 130])  # define range of color in HSV
    upper_blue = np.array([95, 140, 220])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
    cv2.imshow('mask', mask)
    height, width, channels = dst_img.shape
    # print height
    center_y = height / 2
    # print center_y
    # print width
    center_x = width / 2
    # print center_x
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=6)
    cv2.imshow('erode',erode)
    cont = contours(dst_img)

    newcontours = []
    for cnt in cont:
        area = cv2.contourArea(cnt)
        if area > 100:
            newcontours.append(cnt)
    cv2.drawContours(dst_img, newcontours, -1, (0, 255, 0), 3)
    #print newcontours
    cv2.imshow("img",dst_img)
    num_contours = len(newcontours)
    if num_contours == 2 :
        print "Found two lines"
        forward(0.5)
        # zero = line(erode,newcontours[0],center_y)
        # one = line(erode,newcontours[1],center_y)
        #
        # centerlane = (zero + one) / 2
        # error = center_x - centerlane
        # #print error
        # return error

    elif num_contours == 1:
        print "Found one line"
        # rows, cols = img.shape[:2]
        # vx0, vy0, x0, y0 = cv2.fitLine(newcontours[0], cv2.DIST_L2, 0, 0.01, 0.01)
        # lefty0 = int((-x0 * vy0 / vx0) + y0)
        # righty0 = int(((cols - x0) * vy0 / vx0) + y0)
        # x_00 = float(cols - 1)
        # y_00 = float(righty0)
        # x_01 = float(0)
        # y_01 = float(lefty0)
        # slope0 = float((y_01 - y_00) / (x_01 - x_00))
        # yint0 = y_01 - (slope0 * x_01)
        # x0 = (center_y - yint0) / slope0
        # x1 = (rows - yint0) / slope0
        # dif = x1 - x0
        # angle = float(math.atan2((dif), center_y))
        # angle = math.degrees(angle)
        # return angle
        x = line(erode,newcontours[0],center_y)

        if x < center_x:
            right(0.5)

        if x > center_x:
            left(0.5)
    else:
        print "error"

def frame(junk_frames):
    for i in xrange(junk_frames):
        temp = get_image()
    return temp

while True:

    left_distance = ultrasonicleft()
    print left_distance
    right_distance = ultrasonicright()
    print right_distance
    if (right_distance > distance_limit) and (left_distance > distance_limit):
        lane_detection(frame(10))
        cv2.waitKey(5)
    else:
        ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))