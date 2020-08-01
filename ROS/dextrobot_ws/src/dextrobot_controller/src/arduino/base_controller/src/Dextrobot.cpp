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
    // Init the motors connection
    motor_1 = AccelStepper(motorInterfaceType, stepPin_1, dirPin_1);
    motor_2 = AccelStepper(motorInterfaceType, stepPin_2, dirPin_2);
    motor_3 = AccelStepper(motorInterfaceType, stepPin_3, dirPin_3);
    motor_4 = AccelStepper(motorInterfaceType, stepPin_4, dirPin_4);

    // Limit the maximum speed 
    motor_1.setMaxSpeed(MAX_SPEED);
    motor_2.setMaxSpeed(MAX_SPEED);
    motor_3.setMaxSpeed(MAX_SPEED);
    motor_4.setMaxSpeed(MAX_SPEED);

    // Limit the maximum acceleration
    motor_1.setAcceleration(MAX_ACCELERATION);
    motor_2.setAcceleration(MAX_ACCELERATION);
    motor_3.setAcceleration(MAX_ACCELERATION);
    motor_4.setAcceleration(MAX_ACCELERATION);
}

Dextrobot::~Dextrobot()
{
}

void Dextrobot::goForward(int velocity){
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

void Dextrobot::goBackward(int velocity){
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

void Dextrobot::goRight(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goLeft(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goForwardRight(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goForwardLeft(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(0);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(0);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goBackwardRight(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(0);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(0);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::goBackwardLeft(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void Dextrobot::rotateClockwise(int velocity){
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

void Dextrobot::rotateCounterClockwise(int velocity){
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