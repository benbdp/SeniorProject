import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
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


ser.write(str(0) + str('m,') + str(97) + str('s,'))
time.sleep(1)
ser.write(str(60) + str('m,') + str(97) + str('s,'))
time.sleep(2)
ser.write(str(0) + str('m,') + str(97) + str('s,'))

#
# try:
#     webcam = cv2.VideoCapture(0) # index of your camera
#     time.sleep(2)
# except:
#     print ("problem opening input stream")
#     sys.exit(1)
#
#
# try:
#     while True:
#         ret, frame = webcam.read()
#         cv2.imshow('frame', frame)
#         key = cv2.waitKey(1) & 0xFF
#
#         # if the `q` key was pressed, break from the loop
#         if key == ord("q"):
#             break
#         left_distance = ultrasonicleft()
#         right_distance = ultrasonicright()
#         print(left_distance)
#         print(right_distance)
#         if (right_distance > distance_limit) and (left_distance > distance_limit):
#             motor_speed = str(60)
#             servo_angle = str(97)
#             print motor_speed + str('m,')
#             ser.write(motor_speed + str('m,') + servo_angle + str('s,'))
#         else:
#             motor_speed = str(0)
#             print motor_speed + str('m,')
#             ser.write(motor_speed + str('m,'))
# except:
#     cv2.destroyAllWindows()
#     print "User Stopped!"
#     motor_speed = str(0)
#     ser.write(str(0) + str('m,') + str(97) + str('s,'))