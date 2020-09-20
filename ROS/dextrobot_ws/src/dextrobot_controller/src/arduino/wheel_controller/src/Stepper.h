/*
  Stepper.h - DextroBot
  Class that cointains the implementation of the low level stepper mouvements
  based on the velocity command provided from the higher level controller
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <AccelStepper.h>

#ifndef Stepper_h
#define Stepper_h

#define motorInterfaceType 1
#define STEP_ANGLE 1.8 // degrees
#define STEP_PER_REVOLUTION 360/STEP_ANGLE
#define WHEEL_RADIUS 0.04 // meters
// Max speed of the steppers in steps per second
#define MAX_SPEED 600
// Max acceleration of the steppers in steps per second ^2
#define MAX_ACCELERATION 1000

class Stepper
{
    private:
        int dirPin;
        int stepPin;
        int speed; // steps per second
        AccelStepper motor;
        float convertToStepsPerSecond(float ms);
    public:
        Stepper(int dir, int step);
        ~Stepper();   
        void setSpeed(float speed);
        void runSpeed();
};

#endif
