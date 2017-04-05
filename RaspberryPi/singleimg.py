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
cv2.imwrite('/home/pi/Desktop/imgs/frame.png',capture)

#cv2.imshow('frame', frame)
h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
#cv2.imshow('undistort', undistort)
cv2.imwrite('/home/pi/Desktop/imgs/undistort.png',undistort)
src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(undistort,M,(558,154))
cv2.imwrite('/home/pi/Desktop/imgs/warped.png',dst_img)
#cv2.imshow("warp",dst_img)
hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
cv2.imwrite('/home/pi/Desktop/imgs/hsv.png', hsv)
#cv2.imshow('hsv', hsv)
lower_blue = np.array([50, 50, 130])  # define range of color in HSV
upper_blue = np.array([95, 140, 220])
mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
cv2.imwrite('/home/pi/Desktop/imgs/mask.png', mask)
#cv2.imshow('mask', mask)
dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)
erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=6)
cv2.imwrite('/home/pi/Desktop/erode.png', mask)
im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(capture, contours, -1, (0, 0, 255), 3)
cv2.imwrite('/home/pi/Desktop/contours.png', capture)
M = cv2.moments(contours[0])
cx0 = int(M['m10']/M['m00'])
cy0 = int(M['m01']/M['m00'])
cv2.circle(capture,(cx0,cy0),10,(0, 0, 255),-1)
M = cv2.moments(contours[1])
cx1 = int(M['m10']/M['m00'])
cy1 = int(M['m01']/M['m00'])
cv2.circle(capture,(cx1,cy1),10,(0, 0, 255),-1)
cv2.imwrite('/home/pi/Desktop/center.png', capture)

cv2.waitKey()
cv2.destroyAllWindows()
del (camera)