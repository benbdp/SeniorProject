import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
servo_pos=97

print(str(80) + str('m,') + str(servo_pos) + str('s,'))
time.sleep(2)
# ser.write(str(0) + str('m,') + str(servo_pos) + str('s,'))
print(str(0) + str('m,') + str(servo_pos) + str('s,'))