import picamera
import picamera.array
import time
import cv2
import numpy as np

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 24
    time.sleep(2) # AGC warm-up time
    start = time.time()
    for i in range(24):
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, 'bgr', use_video_port=True)
            hsv = cv2.cvtColor(stream.array, cv2.COLOR_BGR2HSV)
            cv2.imshow('frame', hsv)
            cv2.waitKey(1)
            print 'Average H: %.2f, S: %.2f, V: %.2f' % (
                np.average(hsv[..., 0]),
                np.average(hsv[..., 1]),
                np.average(hsv[..., 2]),
                )
    finish = time.time()
    print 'Processed 24 frames in %d seconds at %.2ffps' % (
        finish - start, 24.0 / (finish - start))