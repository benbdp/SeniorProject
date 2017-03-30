import cv2
import numpy as np

mtx = np.load('/home/pi/Cal_Imgs/cameramatrix.npy')
dist = np.load('/home/pi/Cal_Imgs/distortioncoeff.npy')

junk_frames = 30
camera = cv2.VideoCapture(0)

def get_image():
    retval, img = camera.read()
    return img

def contours(img): # img should be wrapped image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv', hsv)
    lower_blue = np.array([50, 50, 130])  # define range of color in HSV
    upper_blue = np.array([95, 140, 220])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
    cv2.imshow('mask', mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)
    size = np.size(dilation)
    skel = np.zeros(dilation.shape, np.uint8)
    ret, img = cv2.threshold(dilation, 127, 255, 0)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    done = False

    while (not done):
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()

        zeros = size - cv2.countNonZero(img)
        if zeros == size:
            done = True

    cv2.imshow("skel",skel)
    im2, contours, hierarchy = cv2.findContours(skel, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

for i in xrange(junk_frames):
    temp = get_image()

capture = get_image()

#cv2.imshow('frame', frame)
h, w = capture.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistort = cv2.undistort(capture, mtx, dist, None, newcameramtx)
cv2.imshow('undistort', undistort)
#cv2.imwrite('/home/pi/Desktop/warp.png',undistort)
src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(undistort,M,(558,154))
cv2.imshow("warp",dst_img)
cont = contours(dst_img)
cv2.drawContours(dst_img, cont, -1, (0, 255, 0), 3)
cv2.imshow("warp",dst_img)

cv2.waitKey()
cv2.destroyAllWindows()
del (camera)