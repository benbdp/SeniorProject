import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
import math
ser = serial.Serial('/dev/ttyACM0', 9600)

distance_limit = 25
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIGGERRIGHT = 18
ECHORIGHT = 23
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 24
ECHOLEFT = 25
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)

mtx = np.matrix([[486.7350296, 0., 319.86577798],
 [0., 485.14619589, 242.13957787],
 [0., 0., 1.]])
#print mtx
dist = np.matrix([[1.72030848e-01,  -4.89793474e-01,  -1.64310264e-03,   4.26229958e-04, 3.80932152e-01]])
#print dist

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
mtx = np.load('/home/pi/Cal_Imgs/cameramatrix.npy')
dist = np.load('/home/pi/Cal_Imgs/distortioncoeff.npy')


try:
    webcam = cv2.VideoCapture(0) # index of your camera
    time.sleep(2)
except:
    print ("problem opening input stream")
    sys.exit(1)
num =0
try:
    while True:
        num = num +1
        ret, frame = webcam.read()
        print webcam.get(cv2.CV_CAP_PROP_POS_FRAMES)
        #
        # #cv2.imshow('frame', frame)
        # h, w = frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # #cv2.imshow('undistort', undistort)
        # src_pts = np.float32([[72, 227], [576, 223], [1, 316], [634, 296]])  # src
        # dst_pts = np.float32([[0, 0], [556, 0], [0, 184], [556, 156]])  # dst
        # M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # dst_img = cv2.warpPerspective(undistort, M, (556, 156))
        # #cv2.imshow('warp', dst_img)
        # hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        # #cv2.imshow('hsv', hsv)
        # lower_blue = np.array([40, 70, 140])  # define range of color in HSV
        # upper_blue = np.array([60,255,255])
        # mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
        # cv2.imshow('mask', mask)
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
        #
        # cv2.drawContours(dst_img, newcontours, -1, (0, 255, 0), 3)
        # # print newcontours
        #
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
        #     servo_pos = servo_pos - (abs(angle) * term)
        # if angle < 0:
        #     servo_pos = servo_pos + (abs(angle) * term)
        #
        # print "servo_pos", servo_pos
        #
        # print(str(72) + str('m,') + str(servo_pos) + str('s,'))
        # ser.write(str(72) + str('m,') + str(servo_pos) + str('s,'))
        # time.sleep(1)
        #
        # print(str(0) + str('m,') + str(servo_pos) + str('s,'))
        # ser.write(str(0) + str('m,') + str(servo_pos) + str('s,'))
        # cv2.imshow("dst",dst_img)
        #
        #
        #
        # # cv2.imwrite('/home/pi/Desktop/hsv%04i.jpg' % num,hsv)
        # key = cv2.waitKey() & 0xFF
        #
        # # if the `q` key was pressed, break from the loop
        # if key == ord("q"):
        #     break
        # #left_distance = ultrasonicleft()
        # #right_distance = ultrasonicright()
        # #print(left_distance)
        # #print(right_distance)
        # #if (right_distance > distance_limit) and (left_distance > distance_limit):
        #     #motor_speed = str(60)
        #     #servo_angle = str(97)
        #     #print motor_speed + str('m,')
        #     #ser.write(motor_speed + str('m,') + servo_angle + str('s,'))
        # #else:
        #     #motor_speed = str(0)
        #     #print motor_speed + str('m,')
        #     #ser.write(motor_speed + str('m,'))
except:
    cv2.destroyAllWindows()
    print "User Stopped!"
    motor_speed = str(0)
    ser.write(str(0) + str('m,') + str(70) + str('s,'))