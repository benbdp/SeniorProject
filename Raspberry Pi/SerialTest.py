import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
import time
import cv2
import RPi.GPIO as GPIO


distance = 5
distance_limit = 10
motor_speed = 0
#right_distance = int(input('enter right_distance: '))
#left_distance = int(input('enter left_distance: '))
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIGGERRIGHT = 21
ECHORIGHT = 20
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 14
ECHOLEFT = 15
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)


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


while True:
    time.sleep(0.1) # sampling rate


    left_distance = ultrasonicleft()
    right_distance = ultrasonicright()

    #
    #
    #
    #
    #


    #Outputs:
    #right_distance
    #left_distance
    #right_distance = int(input('enter right_distance: '))
    #left_distance = int(input('enter left_distance: '))


    if (right_distance > distance_limit) and (left_distance > distance_limit):
        motor_speed = str(100)
        ser.write(motor_speed+"m,")
        #print(motor_speed+"m,")
    else:
        motor_speed=str(0)
        ser.write(motor_speed+"m,")
        #print(motor_speed+"m,")