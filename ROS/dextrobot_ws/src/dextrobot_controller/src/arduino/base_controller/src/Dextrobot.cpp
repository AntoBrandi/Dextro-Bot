/*
  Dextrobot.cpp - DextroBot
  Class that cointains the implementation of the low level omnidirectional mouvements
  for the robot based on the willing direction of the mouvement the stepper motors are controlled
  differently and independently in order to achieve the desired mouvement.
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include "Dextrobot.h"

Dextrobot::Dextrobot(/* args */)
{
    // Enable the CNC shield
    pinMode(ENABLE_SHIELD, OUTPUT);
    digitalWrite(ENABLE_SHIELD, LOW);
}

Dextrobot::~Dextrobot()
{
}

void Dextrobot::goForward(float velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goBackward(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(velocity);
    motor_4.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goForwardRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goForwardLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(velocity);
    motor_4.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goBackwardRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::goBackwardLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::rotateClockwise(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(velocity);
    motor_4.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::rotateCounterClockwise(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

void Dextrobot::stop(){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}

// Update the reading of each sensor
void Dextrobot::sense(){
    imu.sense();
    sonar_1.sense();
    sonar_2.sense();
    sonar_3.sense();
    sonar_4.sense();
}