import cv2
import math
import numpy as np


img = cv2.imread('/Users/Benjamin/PycharmProjects/SeniorProject/LaneDetection/Screen Shot 2017-02-02 at 2.06.46 PM.png')
#print kernal
#def blur(img,kernel_size):
 #   return cv2.GaussianBlur(img, (kernel_size, kernel_size), 1)

#def hsv(img):
  #  return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#def mask(img):
#    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
#    upper_blue = np.array([130, 255, 255])
#    return cv2.inRange(img, lower_blue, upper_blue)

#def dilation(img,kernal_size,iterations):
 #   return cv2.dilate(img, kernal_size, iterations)
#def canny(img):
 #   return cv2.Canny(img, 50, 150)


#cv2.imshow('img',img)
#cv2.waitKey()
#blurred = blur(img,kernel_size)
#cv2.imshow('blur',blurred)
#cv2.waitKey()
#convert_hsv = hsv(blurred)
#cv2.imshow('hsv',convert_hsv)
#cv2.waitKey()
#add_mask = mask(convert_hsv)
#cv2.imshow('mask',add_mask)
#cv2.waitKey()
#dilate = cv2.dilate(mask, kernel1, iterations=5)
#cv2.imshow('dilate',dilate)
#cv2.waitKey()
#blur = cv2.GaussianBlur(img, (5, 5), 3)
#cv2.imshow('blur', blur)
#cv2.waitKey()
#hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert to HSV
#lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
#upper_blue = np.array([130, 255, 255])
#mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
#cv2.imshow('mask', mask)
#cv2.waitKey()
#dilation = cv2.dilate(mask, kernel1, iterations=5)
#cv2.imshow('dilate', dilation)
#cv2.waitKey()
#erode = cv2.erode(dilation, kernel1, iterations=3 )
#cv2.imshow('erode', erode)
#cv2.waitKey()
#edged = cv2.Canny(erode, 50, 150)
#closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel2)
#cv2.waitKey()
#im2, contours, hierarchy = cv2.findContours(erode,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#cnt = contours[0]
#print contours
#cont = cv2.drawContours(img, contours, -1, (0,255,0), 3)
#cv2.imshow('cont',cont)
#cv2.waitKey()

#rows,cols = img.shape[:2]
#print img.shape[:1]
#vx, vy, x, y = cv2.fitLine(contours[0],cv2.DIST_L2,0,0.01,0.01)


#lefty = int((-x*vy/vx) + y)
#if lefty > img.shape[1]:
 #   lefty = img.shape[1]
#righty = int(((img.shape[1]-x)*vy/vx)+y)
#if righty < 0:
 #   righty = 0

#print righty
#print lefty
#cv2.line(img,(img.shape[1]-1,righty),(0,lefty),(0,0,255),2)
#print (img.shape[1]-1,righty),(0,lefty)
#cv2.imshow('img',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
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

def neighbours_vec(image):
    return image[2:,1:-1], image[2:,2:], image[1:-1,2:], image[:-2,2:], image[:-2,1:-1],     image[:-2,:-2], image[1:-1,:-2], image[2:,:-2]

def transitions_vec(P2, P3, P4, P5, P6, P7, P8, P9):
    return ((P3-P2) > 0).astype(int) + ((P4-P3) > 0).astype(int) + \
    ((P5-P4) > 0).astype(int) + ((P6-P5) > 0).astype(int) + \
    ((P7-P6) > 0).astype(int) + ((P8-P7) > 0).astype(int) + \
    ((P9-P8) > 0).astype(int) + ((P2-P9) > 0).astype(int)

def zhangSuen_vec(image, iterations):
    for iter in range (1, iterations):
        print iter
        # step 1
        P2,P3,P4,P5,P6,P7,P8,P9 = neighbours_vec(image)
        condition0 = image[1:-1,1:-1]
        condition4 = P4*P6*P8
        condition3 = P2*P4*P6
        condition2 = transitions_vec(P2, P3, P4, P5, P6, P7, P8, P9) == 1
        condition1 = (2 <= P2+P3+P4+P5+P6+P7+P8+P9) * (P2+P3+P4+P5+P6+P7+P8+P9 <= 6)
        cond = (condition0 == 1) * (condition4 == 0) * (condition3 == 0) * (condition2 == 1) * (condition1 == 1)
        changing1 = np.where(cond == 1)
        image[changing1[0]+1,changing1[1]+1] = 0
        # step 2
        P2,P3,P4,P5,P6,P7,P8,P9 = neighbours_vec(image)
        condition0 = image[1:-1,1:-1]
        condition4 = P2*P6*P8
        condition3 = P2*P4*P8
        condition2 = transitions_vec(P2, P3, P4, P5, P6, P7, P8, P9) == 1
        condition1 = (2 <= P2+P3+P4+P5+P6+P7+P8+P9) * (P2+P3+P4+P5+P6+P7+P8+P9 <= 6)
        cond = (condition0 == 1) * (condition4 == 0) * (condition3 == 0) * (condition2 == 1) * (condition1 == 1)
        changing2 = np.where(cond == 1)
        image[changing2[0]+1,changing2[1]+1] = 0
    return image






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

def laneDetection(img):
    height, width, channels = img.shape
    #print height
    center_y = height / 2
    #print center_y
    #print 'width',width
    center_x = width / 2
    #print center_x
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
    lower_blue = np.array([110, 50, 50])  # define range of blue color in HSV
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Threshold the HSV image to get only blue colors
    cv2.imshow('mask', mask)
    dilation = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=5)
    erode = cv2.erode(dilation, np.ones((5, 5), np.uint8), iterations=3)
    cv2.imshow('erode',erode)
    im2, contours, hierarchy = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



















    num_contours = len(contours)















    #print "contours",num_contours
    if num_contours < 2:
        zero = line(erode,contours[0],center_y)
        print zero


        return


    else:
        newcontours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200:
                newcontours.append(cnt)

        cv2.drawContours(img, newcontours, -1, (0, 255, 0), 3)
        #print newcontours

        one = line(erode,newcontours[0],center_y)
        #print one
        two = line(erode,newcontours[1],center_y)
        #print two
        centerlane = (one + two)/2
        #print center_x
        error = center_x - centerlane
        return error





laneDetection(img)
# print laneDetection(img)
#
#
#
# p = PID(1, 0, 0)
# p.setPoint(0)
# pid = p.update(laneDetection(img))
# print pid

# servo_pos = 70

# if angle >0:
#     servo_pos = servo_pos - (abs(angle) *0.8)
# if angle <0:
#     servo_pos = servo_pos + (abs(angle) * 0.8)
#
# print "servo_pos: ",servo_pos



cv2.imshow('line',img)
cv2.waitKey()
#cv2.imwrite('/Users/Benjamin/Downloads/curvelines.jpg',img)