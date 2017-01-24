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
path = "/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images"
images = glob.glob(os.path.join(path, '*.jpg'))

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

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
        cv2.waitKey(500)
        cv2.destroyAllWindows()


retval, CameraMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print('ret',retval)
print ('mtx',CameraMatrix)s
print('dist',distCoeff)
print('rvecs',rvecs)
print('tvecs',tvecs)

# calibrate webcam and save output
np.savez(('/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/webcam_calibration_ouput'), ret=retval, mtx=CameraMatrix, dist=distCoeff, rvecs=rvecs, tvecs=tvecs)
np.savez(('/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/image_object_points'),objpoints=objpoints,imgpoints=imgpoints)

