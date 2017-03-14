import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)

time.sleep(2)
ser.write(str(80) + str('m,') + str(68) + str('s,'))
print"running!"
time.sleep(2.75)
ser.write(str(80) + str('m,') + str(60) + str('s,'))
time.sleep(1)
ser.write(str(80) + str('m,') + str(68) + str('s,'))
time.sleep(4)
ser.write(str(0) + str('m,') + str(68) + str('s,'))
print "stopped!"