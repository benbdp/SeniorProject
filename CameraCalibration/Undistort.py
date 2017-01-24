import cv2
import numpy as np
import glob

a = np.load('/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/image_object_points.npz')
b = np.load('/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/webcam_calibration_ouput.npz')


images = glob.glob('/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Camera_Calibration_Images/*.jpg')
num = 0
tot_frames = 13
while num < tot_frames:
    for fname in images:
        # read image
        img = cv2.imread(fname)
        h, w = img.shape[:2]
        newcamera,roi=cv2.getOptimalNewCameraMatrix(b['mtx'],b['dist'],(w,h),1,(w,h))
        newimg = cv2.undistort(img, b['mtx'], b['dist'], None, newcamera)
        cv2.imwrite("/Users/Benjamin/PycharmProjects/SeniorProjectCar/Camera_Calibration/Undistort_Images/image%04i.jpg" % num, newimg)
        num += 1
        cv2.imshow('img',newimg)
        cv2.waitKey()
        cv2.destroyAllWindows()


