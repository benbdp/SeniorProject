import time
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_center = 95  # servo position to drive straight

ser.write(str(100) + str('m,') + str(servo_center) + str('s,'))
time.sleep(5)
ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))