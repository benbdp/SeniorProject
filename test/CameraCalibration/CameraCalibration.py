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
path = "/Users/Benjamin/Downloads/Cal_Imgs"
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

print 'matrix',cameramatrix
print'dist',distortioncoeff

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rotationvector[i], translationvector[i], cameramatrix, distortioncoeff)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print ("mean error: ", mean_error/len(objpoints))


img = cv2.imread('/Users/Benjamin/Downloads/Cal_Imgs/image0001.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameramatrix,distortioncoeff,(w,h),1,(w,h))
# undistort
dst = cv2.undistort(img, cameramatrix, distortioncoeff, None, newcameramtx)


cv2.imshow('calibresult.png',dst)
cv2.waitKey()