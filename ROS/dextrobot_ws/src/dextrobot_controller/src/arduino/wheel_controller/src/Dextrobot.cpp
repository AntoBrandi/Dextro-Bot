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

void Dextrobot::goBackward(float velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(velocity);
}

void Dextrobot::goForward(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(velocity);
}

void Dextrobot::goRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
}

void Dextrobot::goLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
}

void Dextrobot::goForwardRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(velocity);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(-velocity);
}

void Dextrobot::goForwardLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(velocity);
    motor_4.setSpeed(0);
}

void Dextrobot::goBackwardRight(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(0);
}

void Dextrobot::goBackwardLeft(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(velocity);
}

void Dextrobot::rotateClockwise(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
}

void Dextrobot::rotateCounterClockwise(float velocity){
    // Set the speed in steps per second:
    motor_1.setSpeed(-velocity);
    motor_2.setSpeed(-velocity);
    motor_3.setSpeed(-velocity);
    motor_4.setSpeed(-velocity);
}

void Dextrobot::stop(){
    // Set the speed in steps per second:
    motor_1.setSpeed(0);
    motor_2.setSpeed(0);
    motor_3.setSpeed(0);
    motor_4.setSpeed(0);
}

// Run the motors
void Dextrobot::run(){
    // Step the motor with a constant speed as set by setSpeed():
    motor_1.runSpeed();
    motor_2.runSpeed();
    motor_3.runSpeed();
    motor_4.runSpeed();
}