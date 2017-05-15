import time
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 95  # servo position to drive straight

def forward():
    done = False
    while not done:
        ser.write(str(200) + str('m,') + str(servo_center) + str('s,'))
        time.sleep(2)
        done = True

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
    done = False
    while not done:
        ser.write(str(200) + str('m,') + str(servo_center) + str('s,'))
        time.sleep(2)
        done = True

# def r():
#     forward()
#     time.sleep(1)
#     right(10)
#
# def o():
#     forward()
#     time.sleep(1)
#     right(12)
#     time.sleep(3)
#
# def m():
#     forward()
#     right()
#     left()
#     right()
#     forward()

def main():
    p()
    # r()
    # o()
    # m()
    # stop()


if __name__ == "__main__":
    p()
    stop()