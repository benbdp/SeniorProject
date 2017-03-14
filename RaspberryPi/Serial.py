import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)

time.sleep(1)
servo_pos=70

print(str(72) + str('m,') + str(servo_pos) + str('s,'))
ser.write(str(72) + str('m,') + str(servo_pos) + str('s,'))
time.sleep(1)
ser.write(str(0) + str('m,') + str(servo_pos) + str('s,'))
print(str(0) + str('m,') + str(servo_pos) + str('s,'))