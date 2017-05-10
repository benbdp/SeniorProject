# import packages
import cv2
import numpy as np

tuned_vals = np.load("/Users/Benjamin/PycharmProjects/SeniorProject/tuned_vals.npy")

# create the camera
camera = cv2.VideoCapture(0)


#show the frames
while True:
    ret, frame = camera.read()
    height,width,channels = frame.shape
    frame = cv2.resize(frame,(width/2,height/2))
    cv2.imshow("frame",frame)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",hsv)
    # lower_range = tuned_vals
    # upper_range =np.array([180,255,255])
    # mask = cv2.inRange(hsv,lowerb=lower_range,upperb=upper_range)
    # cv2.imshow("mask",mask)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break