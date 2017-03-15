import cv2
import datetime
import time
cap = cv2.VideoCapture(0)
timestamp = datetime.datetime.now()

while(1):
    timestamp = datetime.datetime.now()
    ret, frame = cap.read()
    ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")

    cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    cv2.imshow("Frame",frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
