import picamera
import picamera.array
import time
import cv2
import numpy as np

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 60
    time.sleep(2) # AGC warm-up time
    while True:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, 'bgr', use_video_port=True)
            hsv = cv2.cvtColor(stream.array, cv2.COLOR_BGR2HSV)
            cv2.imshow('frame', hsv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()
    print "User Stopped"