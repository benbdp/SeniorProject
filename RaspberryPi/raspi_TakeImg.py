from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
num = 0
maxFrames = 15

# allow the camera to warmup
time.sleep(0.1)

# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
# display the image on screen and wait for a keypress


while True:
        try:
            number1 = float(input("Enter the first number: "))
            number2 = float(input("Enter the second number: "))
        except:
            print "That is not a number!"
            continue