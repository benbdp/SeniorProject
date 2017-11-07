import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
servo_center = 83

time.sleep(1)


while True:
    entry = int(input("Enter change servo pos: "))
    ser.write(str(100) + str('m,') + str(servo_center+entry) + str('s,'))
    print"running!"
    time.sleep(1)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))
    time.sleep(1)
    print "reversing!"
    ser.write(str(-150) + str('m,') + str(servo_center+entry) + str('s,'))
    time.sleep(1)
    ser.write(str(0) + str('m,') + str(servo_center) + str('s,'))
    print "stopped!"