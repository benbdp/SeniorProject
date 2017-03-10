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


def laneDetection(img):
    height, width, channels = img.shape
    #print height
    center_y = height / 2
    print center_y
    #print width
    center_x = width / 2
    #print center_x
    blur = cv2.GaussianBlur(img,(5,5), 3)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv',hsv)
    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
    cv2.imshow('mask',mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
    cv2.imshow('erode',erode)
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #print contours[0]
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

    rows, cols = img.shape[:2]
    vx0, vy0, x0, y0 = cv2.fitLine(contours[0], cv2.DIST_L2, 0, 0.01, 0.01)
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

    cv2.circle(img,(x0,center_y),5,(0,0,255),-1)

    vx1, vy1, x1, y1 = cv2.fitLine(contours[1], cv2.DIST_L2, 0, 0.01, 0.01)
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
    print "turn angle:", angle

    return cv2.line(img,(cols-1,righty0),(0,lefty0),(0,242,255),3), \
           cv2.line(img, (cols - 1, righty1), (0, lefty1), (0, 242, 255), 3), \
           cv2.line(img, (center_x, 0), (center_x, height), (255, 0, 0), 2), \
           cv2.line(img, (0, center_y), (width, center_y), (255, 0, 0), 2)

mtx = np.matrix([[  1.09737118e+03, 0.00000000e+00, 6.29382303e+02], [  0.00000000e+00, 1.10083151e+03, 3.71037449e+02], [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.matrix([[  1.37334519e-01, -1.20441566e+00, 2.19553714e-03, -4.06071434e-04, 2.18048197e+00]])


try:
    webcam = cv2.VideoCapture(0) # index of your camera
    time.sleep(2)
except:
    print ("problem opening input stream")
    sys.exit(1)



try:
    while True:
        ret, frame = webcam.read()
        cv2.imshow('frame', frame)
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        #cv2.imshow('undistort', undistort)

        src_pts = np.float32([[142, 338], [522, 338], [20, 480], [635, 480]])  # src

        dst_pts = np.float32([[0, 0], [558, 0], [0, 430], [558, 430]])  # dst

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        dst_img = cv2.warpPerspective(frame, M, (558, 430))

        blur = cv2.GaussianBlur(dst_img, (5, 5), 3)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
        cv2.imshow('hsv', hsv)
        lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
        cv2.imshow('mask', mask)
        #cv2.imshow('dst', dst_img)
        #laneDetection(dst_img)

        #cv2.imwrite("/home/pi/Desktop/warp.jpg",frame)
        key = cv2.waitKey() & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        #left_distance = ultrasonicleft()
        #right_distance = ultrasonicright()
        #print(left_distance)
        #print(right_distance)
        #if (right_distance > distance_limit) and (left_distance > distance_limit):
            #motor_speed = str(60)
            #servo_angle = str(97)
            #print motor_speed + str('m,')
            #ser.write(motor_speed + str('m,') + servo_angle + str('s,'))
        #else:
            #motor_speed = str(0)
            #print motor_speed + str('m,')
            #ser.write(motor_speed + str('m,'))
except:
    cv2.destroyAllWindows()
    print "User Stopped!"
    motor_speed = str(0)
    ser.write(str(0) + str('m,') + str(97) + str('s,'))