import cv2
import datetime
import time

cap = cv2.VideoCapture(0)
time.sleep(1)
def camera():
    #timestamp = datetime.datetime.now()
    ret, frame = cap.read()
    #ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
    #cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    cap.release()
    return frame

while True:
    frame = camera()
    cv2.imshow("frame",frame)
    cv2.waitKey(3)