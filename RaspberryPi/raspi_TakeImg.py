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

while num < maxFrames:
    cv2.imshow("Image", image)
    cv2.waitKey()
    entry = str(input('enter s or d: '))
    if entry == 's':
        cv2.imwrite("path/image%04i.jpg" % num, image)
        num += 1
        print("image%02i.jpg" % num)

    else:
        pass