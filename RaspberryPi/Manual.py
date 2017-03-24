import RPi.GPIO as GPIO
import serial
import time
import sys
import Tkinter as tk
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 85
distance_limit = 50
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIGGERRIGHT = 18
ECHORIGHT = 23
GPIO.setup(TRIGGERRIGHT, GPIO.OUT)
GPIO.setup(ECHORIGHT, GPIO.IN)
TRIGGERLEFT = 24
ECHOLEFT = 25
GPIO.setup(TRIGGERLEFT, GPIO.OUT)
GPIO.setup(ECHOLEFT, GPIO.IN)

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

def forward(sec):
    ser.write(str(200) + str('m,') + str(servo_center) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))

def reverse(sec):
    ser.write(str(-50) + str('m,') + str(servo_center) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))

def left(sec):
    ser.write(str(200) + str('m,') + str(servo_center-15) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center-15) + str('s,'))

def right(sec):
    ser.write(str(200) + str('m,') + str(servo_center+15) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center+15) + str('s,'))

def key_input(event):
    print "key: ", event.char
    key_press = event.char
    if key_press == "w":
        print "forward"
        left_distance = ultrasonicleft()
        print left_distance
        right_distance = ultrasonicright()
        print right_distance
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            forward(0.5)
        else:
            pass
    if key_press == "s":
        print "reverse"
        reverse(0.5)
    if key_press == "a":
        print "left"
        left_distance = ultrasonicleft()
        print left_distance
        right_distance = ultrasonicright()
        print right_distance
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            left(0.5)
        else:
            pass
    if key_press == "d":
        print "right"
        left_distance = ultrasonicleft()
        print left_distance
        right_distance = ultrasonicright()
        print right_distance
        if (right_distance > distance_limit) and (left_distance > distance_limit):
            right(0.5)
        else:
            pass
    if key_press == "q":
        sys.exit()
    else:
        pass

def main():
    command = tk.Tk()
    command.bind('<KeyPress>',key_input)
    command.mainloop()

main()