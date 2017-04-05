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

h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
src_img = undistort
cv2.imshow('src_img',src_img)

src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(src_img,M,(558,154))

cv2.imshow('dst',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()