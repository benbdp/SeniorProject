//will set both servo position and motor speeds over serial
//use format: ###s, for servo and ###m, for motor
//code adapted from zoomkat
#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myrightmotor = AFMS.getMotor(1);
Adafruit_DCMotor *myleftmotor = AFMS.getMotor(2);
String readString;
Servo myservo;  // creating servo

void setup() {
  AFMS.begin(); 
  Serial.begin(9600);
  myservo.attach(9);  //the pin for the servo control
  myservo.write(97); //set initial servo position if desired
  Serial.println("Ready"); //prints "Ready" when connection established
  myrightmotor->run(BACKWARD);
  myrightmotor->setSpeed(0);
  myleftmotor->run(BACKWARD);
  myleftmotor->setSpeed(0);
}

void loop() {

  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == ',') {
      if (readString.length() >1) {
        Serial.println(readString); //prints string to serial port out

        int n = readString.toInt();  //convert readString into a number

        // auto select appropriate value, copied from someone elses code.
        if(n <= 250)
        {
          if(readString.indexOf('s') >0) myservo.write(n);
          if(readString.indexOf('m') >0) myleftmotor->setSpeed(n);
          if(readString.indexOf('m') >0) myrightmotor->setSpeed(n);
          if(readString.indexOf('m') >0) myleftmotor->run(BACKWARD);
          if(readString.indexOf('m') >0) myrightmotor->run(BACKWARD);
        }
        else
        {
          Serial.print("Value too great: ");
          Serial.println(n);
        }
         readString=""; //clears variable for new input
      }
    }
    else {
      readString += c; //makes the string readString
    }
  }
}