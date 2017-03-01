import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
import time
import cv2
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np

distance_limit = 25
#right_distance = int(input('enter right_distance: '))
#left_distance = int(input('enter left_distance: '))
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIGGERRIGHT = 18
ECHORIGHT = 23
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 24
ECHOLEFT = 25
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)


camera_matrix = np.matrix([[  494.1907, 0.0, 0.0], [0.0 , 492.6565, 0.0], [ 319.8568, 242.5021, 1.0]])

#print camera_matrix

dist_coeff = np.matrix( [0.1936,-0.5185,-0.0012,-8.6415,0.3824])
#print dist_coeff


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


try:
    while True:

        #ser.isOpen()
        #time.sleep(0) # sampling rate
        left_distance = ultrasonicleft()
        print(left_distance)
        right_distance = ultrasonicright()
        print(right_distance)
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            motor_speed = str(60)
            servo_angle = str(97)
            print motor_speed + str('m,')
            ser.write (motor_speed + str('m,') + servo_angle + str('s,'))



            #if turn_angle < 0:



            #if turn_angle > 0:


            #else:
             #   pass










        else:
            motor_speed = str(0)
            print motor_speed + str('m,')
            ser.write(motor_speed + str('m,'))

except KeyboardInterrupt:
    print "User Stopped"
    motor_speed = str(0)
    ser.write(motor_speed + str('m,') + str(97) + str('s,'))
    pass