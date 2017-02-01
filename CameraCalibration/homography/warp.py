import cv2
import numpy as np
import matplotlib as plt

img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Samples/bluelane.jpg')


x1 =190
y1 =75
cv2.circle(img, (x1, y1), 3, (0, 255, 0), -1)

x1 =470
y1 =70
cv2.circle(img, (x1, y1), 3, (0, 255, 0), -1)

x1 = -10
y1 =380
cv2.circle(img, (x1, y1), 3, (0, 255, 0), -1)

x1 =550
y1 =400
cv2.circle(img, (x1, y1), 3, (0, 255, 0), -1)

#cv2.imshow('points',img)
cv2.imshow('img',img)
height,width,ch = img.shape
print(width,height)



pts1 = np.float32([[190,75],[470,70],[-10,380],[560,400]])
pts2 = np.float32([[0,0],[640,0],[0,426],[640,426]])




M = cv2.getPerspectiveTransform(pts1,pts2)
dst = cv2.warpPerspective(img,M,(640,426))
#cv2.imshow('img',img)
cv2.imshow('dst',dst)
cv2.waitKey()
cv2.destroyAllWindows()