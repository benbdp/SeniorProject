import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import sys
import math
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 86
distance_limit = 50
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

mtx = np.load('/home/pi/Cal_Imgs/cameramatrix.npy')
dist = np.load('/home/pi/Cal_Imgs/distortioncoeff.npy')

junk_frames = 50
camera = cv2.VideoCapture(0)
class PID:
    """
	Discrete PID control
	"""

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
		Calculate PID output value for given reference input and feedback
		"""

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self, set_point):
        """
		Initilize the setpoint of PID
		"""
        self.set_point = set_point
        self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

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
    cv2.circle(img, (x0, center_y), 5, (0, 0, 255), -1)
    return x0

def get_image():
    retval, img = camera.read()
    return img

def lane_detection(img):
    #cv2.imshow('frame', frame)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistort = cv2.undistort(img, mtx, dist, None, newcameramtx)
    cv2.imshow('undistort', undistort)
    src_pts = np.float32([[59,228],[568,227],[3,305],[625,305]])#src

    dst_pts = np.float32([[0,0],[558,0],[0,154],[558,154]])#dst

    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    dst_img = cv2.warpPerspective(undistort,M,(558,154))
    cv2.imshow('warp', dst_img)
    hsv = cv2.cvtColor(dst_img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    cv2.imshow('hsv', hsv)
    lower_blue = np.array([40, 70, 140])  # define range of color in HSV
    upper_blue = np.array([60,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only desired color
    cv2.imshow('mask', mask)
    height, width, channels = dst_img.shape
    # print height
    center_y = height / 2
    # print center_y
    # print width
    center_x = width / 2
    # print center_x
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=6)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=6)
    cv2.imshow('erode',erode)
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print contours[0]

    newcontours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            newcontours.append(cnt)
    cv2.drawContours(dst_img, newcontours, -1, (0, 255, 0), 3)
    #print newcontours

    num_contours = len(newcontours)
    if num_contours == 2 :
        print "Found two lines"
        zero = line(erode,newcontours[0],center_y)
        one = line(erode,newcontours[1],center_y)

        centerlane = (zero + one) / 2
        error = center_x - centerlane
        #print error
        return error

    elif num_contours == 1:
        print "Found one line"
    else:
        print "error"

def frame(junk_frames):
    for i in xrange(junk_frames):
        temp = get_image()
    return temp

directions = []
p = PID(1, 0, 0)
p.setPoint(0)
num = 0
while True:
    print "num: ",num
    num = num +1
    lane_detection(frame(50))
    cv2.waitKey()
    pid = p.update(lane_detection(frame(50)))
    cv2.waitKey()
    directions.append(pid)
    print directions