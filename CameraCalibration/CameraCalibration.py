import numpy as np
import cv2
import glob,os

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
path = "/Users/Benjamin/PycharmProjects/SeniorProject/CameraCalibration/CameraCal"
images = glob.glob(os.path.join(path, '*.png'))

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    print(gray.shape[::-1])

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(100)

retval, cameramatrix, distortioncoeff, rotationvector, translationvector = cv2.calibrateCamera(objpoints, imgpoints, (1280,720), None, None)

print('matrix',cameramatrix)
print('dist',distortioncoeff)

img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/bluelane.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameramatrix,distortioncoeff,(w,h),1,(w,h))

print('newmatrix',newcameramtx)

dst = cv2.undistort(img, cameramatrix, distortioncoeff, None, newcameramtx)
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]

cv2.imshow('dst',dst)
cv2.waitKey()