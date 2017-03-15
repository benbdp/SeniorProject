import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
#ser = serial.Serial('/dev/ttyACM0', 9600)

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

try:
    while True:
        ret, frame = webcam.read()
        #cv2.imshow('frame', frame)
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        #cv2.imshow('undistort', undistort)
        src_pts = np.float32([[72, 227], [576, 223], [1, 316], [634, 296]])  # src
        dst_pts = np.float32([[0, 0], [556, 0], [0, 184], [556, 156]])  # dst
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        dst_img = cv2.warpPerspective(undistort, M, (556, 156))
        #cv2.imshow('warp', dst_img)
        hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        cv2.imshow('hsv', hsv)
        lower_blue = np.array([40, 70, 140])  # define range of color in HSV
        upper_blue = np.array([60, 100, 160])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
        cv2.imshow('mask', mask)


        #cv2.imwrite('/home/pi/Desktop/hsv.jpg',hsv)
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
    #ser.write(str(0) + str('m,') + str(97) + str('s,'))