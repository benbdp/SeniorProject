import serial
import time
ser = serial.Serial(‘/dev/ttyAMA0’, 9600, timeout=1)
ser.open()

ser.write(str('100m,'))
time.sleep(1)
ser.write(str('0m,'))
ser.close()