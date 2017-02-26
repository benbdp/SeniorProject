import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
import time
import cv2
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread


distance = 5
distance_limit = 10
motor_speed = 0
#right_distance = int(input('enter right_distance: '))
#left_distance = int(input('enter left_distance: '))
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIGGERRIGHT = 25
ECHORIGHT = 24
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 18
ECHOLEFT = 23
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))

class PiVideoStream:
    def __init__(self,resolution=(640,480),framerate=32):
        self.camera=PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera,size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,format="bgr",use_video_port=True)
        self.frame = None
        self.stopped = False
    def start(self):
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        for f in self.stream:
            self.frame = f.array
            self.rawCapture.truncate(0)

            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.stream.close()
                return

    def read(self):
        return self.frame

    def stop(self):
        self.stopped=True


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
        ser.isOpen()
        time.sleep(0.01) # sampling rate
        left_distance = ultrasonicleft()
        print(left_distance)
        right_distance = ultrasonicright()
        print(right_distance)
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            motor_speed = str(100)
            print motor_speed + str('m,')
            ser.write (motor_speed + str('m,'))

            # Camera stuff

            stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)

            cv2.imshow('frame',stream)




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
    ser.close()
    pass