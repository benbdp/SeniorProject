import cv2
import numpy as np
import matplotlib as plt

src_img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/CameraCalibration/bluelane.png')
cv2.imshow('src_img',src_img)

src_pts = np.float32([[343,386],[857,386],[0,558],[1256,585]]) #src

dst_pts = np.float32([[0,0],[342,0],[0,440],[344,460]]) #dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(src_img,M,(460,344))

cv2.imshow('dst',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()