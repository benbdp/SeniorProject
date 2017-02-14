# import serial
# ser = serial.Serial('/dev/ttyACM0', 9600)
import time
import cv2
#import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera




camera = PiCamera()
camera.resolution = (640,480)

camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(640,480))

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture,format='bgr',use_video_port=True):
    image = frame.array

    cv2.imshow('frame',image)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)

    if key == ord('q'):
        break

distance = 5
distance_limit = 10
motor_speed = 0
#right_distance = int(input('enter right_distance: '))
#left_distance = int(input('enter left_distance: '))
#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BCM)
#TRIGGERRIGHT = 21
#ECHORIGHT = 20
#GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
#GPIO.setup(ECHORIGHT, GPIO.IN)
#TRIGGERLEFT = 14
#ECHOLEFT = 15
#GPIO.setup(TRIGGERLEFT, GPIO.OUT)
#GPIO.setup(ECHOLEFT, GPIO.IN)
#GPIO.setup(16, GPIO.OUT)
#GPIO.setup(18, GPIO.OUT)


#def ultrasonicleft():
#    GPIO.output(TRIGGERLEFT, False)
#    time.sleep(0.5)
#    GPIO.output(TRIGGERLEFT, True)
#    time.sleep(0.00001)
#    GPIO.output(TRIGGERLEFT, False)

#    while GPIO.input(ECHOLEFT) == 0:
#        start1 = time.time()

#    while GPIO.input(ECHOLEFT) == 1:
#        stop1 = time.time()

#    elapsed1 = stop1 - start1
#    left_distance = (elapsed1 * 34300) / 2


#    return left_distance

#def ultrasonicright():
#    GPIO.output(TRIGGERRIGHT, False)
#    time.sleep(0.5)
#    GPIO.output(TRIGGERRIGHT, True)
#    time.sleep(0.00001)
#    GPIO.output(TRIGGERRIGHT, False)

#    while GPIO.input(ECHORIGHT) == 0:
 #       start1 = time.time()

#    while GPIO.input(ECHORIGHT) == 1:
#        stop1 = time.time()

#    elapsed1 = stop1 - start1
#    right_distance = (elapsed1 * 34300) / 2

 #   return right_distance


while True:
    time.sleep(0.1) # sampling rate


    #distance1 = ultrasonicleft()
    #distance2 = ultrasonicright()
    #if distance1 > 0 and distance1 < 10:
     #   print("Blocked")
     #   GPIO.output(18, GPIO.LOW)
    #else:
      #  print("Distance-Left : %.1f cm" % distance1)
      #  GPIO.output(18, GPIO.HIGH)

   # if distance2 > 0 and distance2 < 10:
      #  print("Blocked")
      #  GPIO.output(16, GPIO.LOW)
    #else:
      #  print("Distance-Right : %.1f cm" % distance2)
     #   GPIO.output(16, GPIO.HIGH)

    #
    #
    #
    #
    #


    #Outputs:
    #right_distance
    #left_distance
    right_distance = int(input('enter right_distance: '))
    left_distance = int(input('enter left_distance: '))


    if (right_distance > distance_limit) and (left_distance > distance_limit):
        motor_speed = 100
        # ser.write('m%d'%motor_speed)
        print('m%d' % motor_speed)
    else:
        motor_speed=0
        print('m%d' % motor_speed)
        num = 0