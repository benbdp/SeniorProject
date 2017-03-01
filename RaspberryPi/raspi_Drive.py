import picamera
import picamera.array
import time
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
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
        start1 = time.time()

    while GPIO.input(ECHORIGHT) == 1:
        stop1 = time.time()

    elapsed1 = stop1 - start1
    right_distance = (elapsed1 * 34300) / 2

    return right_distance

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 120
    time.sleep(2) # AGC warm-up time
    while True:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, 'bgr', use_video_port=True)
            frame = stream.array
            h, w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            undist = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            hsv = cv2.cvtColor(undist, cv2.COLOR_BGR2HSV)
            cv2.imshow('frame', hsv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        left_distance = ultrasonicleft()
        #print(left_distance)
        right_distance = ultrasonicright()
        #print(right_distance)
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            motor_speed = str(60)
            servo_angle = str(97)
            #print motor_speed + str('m,')
            ser.write(motor_speed + str('m,') + servo_angle + str('s,'))
        else:
            motor_speed = str(0)
            #print motor_speed + str('m,')
            ser.write(motor_speed + str('m,'))
    # Protocol if user stops program
    cv2.destroyAllWindows()
    print "User Stopped!"
    motor_speed = str(0)
    ser.write( str(0) + str('m,') + str(97) + str('s,'))