import cv2
import numpy as np

src_img = cv2.imread('/home/pi/Desktop/warp.jpg')
cv2.imshow('src_img',src_img)
#
# print(src_img.shape)
#
# src_pts = np.float32([[72,227],[576,223],[1,316],[634,296]])#src
#
# dst_pts = np.float32([[0,0],[556,0],[0,184],[556,156]])#dst
#
# M = cv2.getPerspectiveTransform(src_pts,dst_pts)
# dst_img = cv2.warpPerspective(src_img,M,(556,156))
#
# cv2.imshow('dst',dst_img)
# cv2.waitKey()
# cv2.destroyAllWindows()