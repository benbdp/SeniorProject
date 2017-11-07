import cv2
import numpy as np

mtx = np.load('/home/pi/SeniorProject/RaspberryPi/cameramatrix.npy')
dist = np.load('/home/pi/SeniorProject/RaspberryPi/distortioncoeff.npy')

junk_frames = 30
camera = cv2.VideoCapture(0)

def get_image():
    retval, img = camera.read()
    return img


for i in xrange(junk_frames):
    temp = get_image()

capture = get_image()

h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
src_img = undistort
cv2.imshow('src_img',src_img)

src_pts = np.float32([[87,176],[552,177],[4,301],[638,302]])#src

dst_pts = np.float32([[0,0],[558,0],[0,264],[558,264]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(src_img,M,(558,264))

cv2.imshow('dst',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()