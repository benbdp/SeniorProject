import cv2
import picamera
import picamera.array

import time
import RPi.GPIO as GPIO
import numpy as np
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

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
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (640, 480)

        while True:
            camera.capture(stream, 'bgr', use_video_port=True)
            # stream.array now contains the image data in BGR order
            cv2.imshow('frame', stream.array)
            blur = cv2.GaussianBlur(stream.array, (5, 5), 3)
            cv2.imshow('blur', blur)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            stream.seek(0)
            stream.truncate()
            left_distance = ultrasonicleft()
            print(left_distance)
            right_distance = ultrasonicright()
            print(right_distance)
            if (right_distance > distance_limit) and (left_distance > distance_limit):
                motor_speed = str(60)
                servo_angle = str(97)
                print motor_speed + str('m,')
                ser.write(motor_speed + str('m,') + servo_angle + str('s,'))


            else:
                motor_speed = str(0)
                print motor_speed + str('m,')
                ser.write(motor_speed + str('m,'))

        cv2.destroyAllWindows()
        print "User Stopped"
        motor_speed = str(0)
        ser.write(motor_speed + str('m,') + str(97) + str('s,'))
