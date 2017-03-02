
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


mtx = np.matrix([[486.7350296, 0., 319.86577798],
 [0., 485.14619589, 242.13957787],
 [0., 0., 1.]])
#print mtx
dist = np.matrix([[1.72030848e-01,  -4.89793474e-01,  -1.64310264e-03,   4.26229958e-04, 3.80932152e-01]])
#print dist

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
 time.sleep(0.01)
 # grab the raw NumPy array representing the image, then initialize the timestamp
 # and occupied/unoccupied text
 image = frame.array
 h, w = image.shape[:2]
 print h, w
 newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
 undist = cv2.undistort(image, mtx, dist, None, newcameramtx)

 # show the frame
 cv2.imshow("Frame", undist)
 key = cv2.waitKey(1) & 0xFF

 # clear the stream in preparation for the next frame
 rawCapture.truncate(0)
 # if the `q` key was pressed, break from the loop
 if key == ord("q"):
  break