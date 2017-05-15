import time
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 82  # servo position to drive straight

def forward(sec):
    ser.write(str(200) + str('m,') + str(servo_center) + str('s,'))
    time.sleep(sec)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))
# function that will stop the car
def stop():
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))
# function to turn left
def left(angle):
    ser.write(str(100) + str('m,') + str(servo_center-angle) + str('s,'))
# function to turn right
def right(angle):
    ser.write(str(100) + str('m,') + str(servo_center+angle) + str('s,'))

def p():
    forward(3)
    right(10)
    time.sleep(3)
    # forward()
    # time.sleep(0.5)


def r():
    forward()
    time.sleep(1)
    right(10)

def o():
    forward()
    time.sleep(1)
    right(12)
    time.sleep(3)

def m():
    forward()
    right()
    left()
    right()
    forward()

def main():
    p()
    r()
    o()
    m()
    stop()


if __name__ == "__main__":
    p()
    stop()