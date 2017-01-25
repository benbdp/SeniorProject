import cv2
import numpy as np
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*3,3), np.float32)
objp[:,:2] = np.mgrid[0:3,0:4].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/CameraCalibration/homography/YBr5y.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (3,4),None)
cv2.imshow('gray',gray)
cv2.waitKey()



# If found, add object points, image points (after refining them)
if ret == True:
    objpoints.append(objp)

    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (3,4), corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey()
    cv2.destroyAllWindows()


retval, cameraMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# camera intrinsic parameters
ay = cameraMatrix[1, 1]
u0 = cameraMatrix[0, 2]
v0 = cameraMatrix[1, 2]
print ("Ay:", ay)
print ("u0:", u0)
print ("v0:", v0)
