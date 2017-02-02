import cv2
import numpy as np
import matplotlib as plt

src_img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/bluelane.jpg')
cv2.imshow('src_img',src_img)

src_pts = np.float32([[190,75],[470,70],[-10,380],[560,400]]) #src
dst_pts = np.float32([[0,0],[640,0],[0,426],[640,426]]) #dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(src_img,M,(640,426))

#cv2.imshow('dst',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()