import time
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 95  # servo position to drive straight

ser.write(str(100) + str('m,'))