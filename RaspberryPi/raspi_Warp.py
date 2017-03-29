import cv2
import numpy as np

src_img = cv2.imread('/home/pi/Desktop/warp.png')
cv2.imshow('src_img',src_img)

print(src_img.shape)

src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

M = cv2.getPerspectiveTransform(src_pts,dst_pts)
dst_img = cv2.warpPerspective(src_img,M,(558,154))

cv2.imshow('dst',dst_img)
cv2.waitKey()
cv2.destroyAllWindows()