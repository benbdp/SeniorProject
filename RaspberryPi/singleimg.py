import cv2
import numpy as np

mtx = np.load('/home/pi/Cal_Imgs/cameramatrix.npy')
dist = np.load('/home/pi/Cal_Imgs/distortioncoeff.npy')

junk_frames = 30
camera = cv2.VideoCapture(0)

def get_image():
    retval, img = camera.read()
    return img

for i in xrange(junk_frames):
    temp = get_image()

capture = get_image()

#cv2.imshow('frame', frame)
h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
#cv2.imshow('undistort', undistort)
src_pts = np.float32([[72, 227], [576, 223], [1, 316], [634, 296]])  # src
dst_pts = np.float32([[0, 0], [556, 0], [0, 184], [556, 156]])  # dst
M = cv2.getPerspectiveTransform(src_pts, dst_pts)
dst_img = cv2.warpPerspective(undistort, M, (556, 156))
#cv2.imshow('warp', dst_img)
hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
cv2.imshow('hsv', hsv)
lower_blue = np.array([40, 70, 140])  # define range of color in HSV
upper_blue = np.array([60,255,255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
cv2.imshow('mask', mask)
del (camera)