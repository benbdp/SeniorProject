import cv2
import numpy as np
import glob
import os

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 6x9 chess board, prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
object_point = np.zeros((6*9, 3), np.float32)
object_point[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

# 3d point in real world space
object_points = []
# 2d points in image plane
image_points = []
h, w = 0, 0

path = "/Users/Benjamin/Downloads/Cal_Imgs/"
images = glob.glob(os.path.join(path, '*.jpg'))
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    print(gray.shape[::-1])

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(1000)

retval, cameramatrix, distortioncoeff, rotationvector, translationvector = cv2.calibrateCamera(objpoints, imgpoints, (640,480), None, None)

img = cv2.imread('/Users/Benjamin/Downloads/Cal_Imgs/image0001.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(w,h),1,(w,h))
# undistort
dst = cv2.undistort(img, cameraMatrix, distCoeffs, None, newcameramtx)


cv2.imshow('calibresult.png',dst)
cv2.waitKey()
