# Self-Driving Car Senior Project

This repository is for the work done on my Senior Project.

My intention with this project is to use inexpensive hardware to build a car that can be made to drive autonomously on a simulated road. I intened to use this project to learn about computer vision and programming in general.

#### How it works:
Raspberry Pi serves as main on-board computer. There are two ultrasonic sensors connected to the Pi used for obstacle avoidance. An Ardino with a servo/motor control shield communicates with the Pi over serial connection. The Pi reads images from a connected USB webcam, image processing is done on these images to determine how the car should react according to its position on the road. The car is powered by 4-cell Li-Po battery conneted to UBEC to regulate 6v to motors and 5v to electronics.

#### How images are processed:
1. Image is captured.
2. Image is undisorted.
3. Image is warped so that its appears top-down.
4. Image is converted to HSV colorspace.
5. Image is thresholded based on values for desired lane lines. This creates binary image.
6. Function to find lane lines (white pixels) is run and returns contours.
7. Countours are fed into function that determines center of each contour.
8. Tests run on contours to ensure they represent lane line.
9. If two lines in one image drive forward.
10. If line on right side of image turn left.
11. If line on left side of image turn right.
12. If no lane lines stop.
13. Repeat. This executes about 10 times/second.

#### Notes:
To run: execute *main.py* in */src/* with python 2.7

Code in other directories is test/example code.

![alt text](https://github.com/benbdp/SeniorProject/blob/master/car.jpg?raw=true)
