# import serial
# ser = serial.Serial('/dev/ttyACM0', 9600)
import time
import cv2

distance = 5
distance_limit = 10
motor_speed = 0
#right_distance = int(input('enter right_distance: '))
#left_distance = int(input('enter left_distance: '))

while True:
    time.sleep(0.1) # sampling rate
    # Ultrasonic Code

    #
    #
    #
    #
    #


    #Outputs:
    #right_distance
    #left_distance
    right_distance = int(input('enter right_distance: '))
    left_distance = int(input('enter left_distance: '))


    if (right_distance > distance_limit) and (left_distance > distance_limit):
        motor_speed = 100
        # ser.write('m%d'%motor_speed)
        print('m%d' % motor_speed)
        num = 1

    elif (right_distance < distance_limit) and (left_distance < distance_limit):
        motor_speed = 0
        # ser.write('m%d'%0)
        print('m%d' % motor_speed)
        num = 0

    else:
        motor_speed=0
        print('m%d' % motor_speed)
        num = 0

    while num == 1:
        print()