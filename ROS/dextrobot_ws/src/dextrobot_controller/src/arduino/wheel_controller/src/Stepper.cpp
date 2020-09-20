/*
  Stepper.cpp - DextroBot
  Class that cointains the implementation of the low level stepper mouvements
  based on the velocity command provided from the higher level controller
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include "Stepper.h"

Stepper::Stepper(int dirPin, int stepPin) : dirPin(dirPin), stepPin(stepPin)
{
    // Init the motors connection
    motor = AccelStepper(motorInterfaceType, dirPin, stepPin);

    // Limit the maximum speed 
    motor.setMaxSpeed(MAX_SPEED);

    // Limit the maximum acceleration
    motor.setAcceleration(MAX_ACCELERATION);
}

Stepper::~Stepper()
{
}

float Stepper::convertToStepsPerSecond(float ms){
    // float time = (2*3.1416*WHEEL_RADIUS)/ms; // seconds for one rotation
    // float rpm = 60/time; // rotation per min
    // int stepsPerSecond = (int)(rpm/60)*STEP_PER_REVOLUTION; 
    // compact form
    return (ms/(2*3.1416*WHEEL_RADIUS))*STEP_PER_REVOLUTION;
}

// Receives a speed in m/s and convert it to steps/s before applying it to the stepper
void Stepper::setSpeed(float ms){
    speed = convertToStepsPerSecond(ms);
    // filter the speed so taht all the motors can apply it
    if(speed>MAX_SPEED){
        speed = MAX_SPEED;
    }
    motor.setSpeed(speed);
}


void Stepper::runSpeed(){
    motor.runSpeed();
}