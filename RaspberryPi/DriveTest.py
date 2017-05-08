import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
from imutils.video import WebcamVideoStream
import imutils
ser = serial.Serial('/dev/ttyACM0', 9600)



servo_center = 82  # servo position to drive straight
distance_limit = 40  # how close an object can get to the car
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIGGERRIGHT = 24
ECHORIGHT = 23
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 15
ECHOLEFT = 14
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)

# load camera matrix and distortion coefficients
mtx = np.load('/home/pi/SeniorProject/RaspberryPi/cameramatrix.npy')
dist = np.load('/home/pi/SeniorProject/RaspberryPi/distortioncoeff.npy')
lower = np.load('/home/pi/SeniorProject/RaspberryPi/hsv.npy')

vs = WebcamVideoStream(src=0).start()

# function to return left ultrasonic distance
def ultrasonicleft():
    GPIO.output(TRIGGERLEFT, False)
    time.sleep(0.5)
    GPIO.output(TRIGGERLEFT, True)
    time.sleep(0.00001)
    GPIO.output(TRIGGERLEFT, False)

    while GPIO.input(ECHOLEFT) == 0:
        start1 = time.time()

    while GPIO.input(ECHOLEFT) == 1:
        stop1 = time.time()

    elapsed1 = stop1 - start1
    left_distance = (elapsed1 * 34300) / 2


    return left_distance

# function to return left ultrasonic distance
def ultrasonicright():
    GPIO.output(TRIGGERRIGHT, False)
    time.sleep(0.5)
    GPIO.output(TRIGGERRIGHT, True)
    time.sleep(0.00001)
    GPIO.output(TRIGGERRIGHT, False)

    while GPIO.input(ECHORIGHT) == 0:
        start2 = time.time()

    while GPIO.input(ECHORIGHT) == 1:
        stop2 = time.time()

    elapsed1 = stop2 - start2
    right_distance = (elapsed1 * 34300) / 2

    return right_distance


# function to drive forward
def forward():
    ser.write(str(63) + str('m,') + str(servo_center) + str('s,'))
# function that will stop the car
def stop():
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))
# function to turn left
def left():
    ser.write(str(63) + str('m,') + str(servo_center-7) + str('s,'))
# function to turn right
def right():
    ser.write(str(63) + str('m,') + str(servo_center+7) + str('s,'))


def new_right():
    pass

def new_left():
    pass


def contours(img,lower): # img should be wrapped image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    upper = np.array([180,255,255])# define range of color in HSV
    mask = cv2.inRange(hsv, lower, upper)  # Threshold the HSV image to get only desired color
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)  # dilate pixels to fill in gaps
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=6)  # cut away border pixels to reduce size
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # contour detection
    return contours

def center(contour):
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00'])  # return x coordinate of center contour
    return cx

def line(img,contours,center_y):
    rows, cols = img.shape[:2]
    vx0, vy0, x0, y0 = cv2.fitLine(contours, cv2.DIST_L2, 0, 0.01, 0.01)
    lefty0 = int((-x0 * vy0 / vx0) + y0)
    righty0 = int(((cols - x0) * vy0 / vx0) + y0)
    x_00 = float(cols - 1)
    y_00 = float(righty0)
    x_01 = float(0)
    y_01 = float(lefty0)
    slope0 = float((y_01 - y_00) / (x_01 - x_00))
    yint0 = y_01 - (slope0 * x_01)
    x0 = (center_y - yint0) / slope0
    x0 = int(x0)
    return x0

def lane_detection(img):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistort = cv2.undistort(img, mtx, dist, None, newcameramtx)  # undistort image
    src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])  # source points
    dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])  # destination points
    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    warp = cv2.warpPerspective(undistort,M,(558,154))  # warp the image
    cont = contours(warp,lower)
    height, width, channels = warp.shape
    center_x = width / 2
    newcontours = []
    for cnt in cont:
        area = cv2.contourArea(cnt)
        if area > 100:  # run test to ensure small contours are eliminated
            newcontours.append(cnt)
    num_contours = len(newcontours)
    if num_contours == 2:  # result if two lines
        print "Found two lines"
        forward()

    elif num_contours == 1:  # result if one lane lines
        print "Found one line"
        x= center(newcontours[0])

        if x < center_x:  # result if line is left of image
            right()
        if x > center_x:  # result if line is right of image
            left()

    else:  # result if no lines or too many lines
        print "error"
        stop()

def main():
    # Main loop
    try:
        while True:
            start = time.time()
            left_distance = ultrasonicleft()
            print left_distance
            right_distance = ultrasonicright()
            print right_distance
            if (right_distance > distance_limit) and (
                left_distance > distance_limit):  # if car is safe distance from object drive!
                getframe = vs.read()
                newframe = imutils.resize(getframe, width=320)
                lane_detection(newframe)
            else:
                stop()  # if the car is not a safe distance do not move
            end = time.time()
            elapsed = end - start
            print elapsed
    except:
        stop()  # stop car when program is stopped
        cv2.destroyAllWindows()
        vs.stop()

if __name__ == "__main__":

    # Main loop
    main()