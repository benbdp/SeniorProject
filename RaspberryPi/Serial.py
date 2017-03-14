import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)

servo_pos=70
print(str(72) + str('m,') + str(servo_pos) + str('s,'))
ser.write(str(80) + str('m,') + str(servo_pos) + str('s,'))
time.sleep(3)
print(str(0) + str('m,') + str(servo_pos) + str('s,'))
ser.write(str(0) + str('m,') + str(servo_pos) + str('s,'))
