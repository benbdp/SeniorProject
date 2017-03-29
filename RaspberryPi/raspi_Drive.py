import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
import math
ser = serial.Serial('/dev/ttyACM0', 9600)

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

kernel_size = 3
disance_limit = 25

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

try:
    while True:
        # ret, frame = webcam.read()
        # h, w = frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # src_pts = np.float32([[142, 338], [522, 338], [20, 480], [635, 480]])  # src
        # dst_pts = np.float32([[0, 0], [558, 0], [0, 430], [558, 430]])  # dst
        # M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # dst_img = cv2.warpPerspective(frame, M, (558, 430))
        #
        # angle = laneDetection(dst_img)
        # term = 0.6
        # servo_pos = 97
        #
        # if angle > 0:
        #     servo_pos = servo_pos - (abs(angle) * term)
        # if angle < 0:
        #     servo_pos = servo_pos + (abs(angle) * term)
        #
        # print servo_pos
        # cv2.imshow("frame",frame)
        # cv2.waitKey(5)

        # if the `q` key was pressed, break from the loop

        left_distance = ultrasonicleft()
        right_distance = ultrasonicright()
        print(left_distance)
        print(right_distance)
        # if (right_distance > distance_limit) and (left_distance > distance_limit):
        #     ser.write(str(80) + str('m,') + str(servo_pos) + str('s,'))
        #     time.sleep(1)
        #     ser.write(str(0) + str('m,') + str(servo_pos) + str('s,'))
        #
        # else:
        #     ser.write(str(0) + str('m,') + str(97) + str('s,'))
except:
    cv2.destroyAllWindows()
    print "User Stopped!"
    motor_speed = str(0)
    ser.write(str(0) + str('m,') + str(97) + str('s,'))