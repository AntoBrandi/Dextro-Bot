/*
  Dextrobot.h - DextroBot
  Class that cointains the implementation of the low level omnidirectional mouvements
  for the robot based on the willing direction of the mouvement the stepper motors are controlled
  differently and independently in order to achieve the desired mouvement.
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <AccelStepper.h>

// Mechanical and Electrical parameters
#define STEP_ANGLE 1.8
#define STEP_PER_REVOLUTION 360/STEP_ANGLE
#define WHEEL_RADIUS 0.04 // meters

// Define the PIN number of the Arduino board connected with the driver motors
// MOTOR 1
#define dirPin_1 2
#define stepPin_1 3
// MOTOR 2
#define dirPin_2 4
#define stepPin_2 5
// MOTOR 3
#define dirPin_3 6
#define stepPin_3 7
// MOTOR 4
#define dirPin_4 8
#define stepPin_4 9
// AccelStepper parameter
#define motorInterfaceType 1
// Stepper Motor control parameters
// Max speed of the steppers in steps per second
#define MAX_SPEED 1500
// Max acceleration of the steppers in steps per second ^2
#define MAX_ACCELERATION 1000
// Speed levels for the robot mouvements
// TODO: tune this
#define SUPERSONIC 1500
#define INSANE 1000
#define SUPERFAST 800
#define FAST 600
#define NORMAL 400
#define SLOW 300

#ifndef Dextrobot_h
#define Dextrobot_h

class Dextrobot
{
private:
    AccelStepper motor_1;
    AccelStepper motor_2;
    AccelStepper motor_3;
    AccelStepper motor_4;

public:
    Dextrobot(/* args */);
    ~Dextrobot();

    // omnidirectional mouvement functions
    void goForward(int velocity);
    void goBackward(int velocity);
    void goRight(int velocity);
    void goLeft(int velocity);
    void goForwardRight(int velocity);
    void goForwardLeft(int velocity);
    void goBackwardRight(int velocity);
    void goBackwardLeft(int velocity);
    void rotateClockwise(int velocity);
    void rotateCounterClockwise(int velocity);
    void stop();
    float convertToStepsPerSecond(float ms);
};


#endif