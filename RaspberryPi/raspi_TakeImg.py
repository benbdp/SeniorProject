# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
num = 1

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    # show the frame
    cv2.imshow("Frame", image)
    cv2.waitKey()

    entry = int(input("Enter 1 to save: "))

    if (entry == int(1)):
        cv2.imwrite("/home/pi/Cal_Imgs/image%04i.jpg" % num, image)
        num = num + 1

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
