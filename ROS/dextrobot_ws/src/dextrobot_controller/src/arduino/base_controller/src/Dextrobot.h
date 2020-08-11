/*
  Dextrobot.h - DextroBot
  Class that cointains the implementation of the low level omnidirectional mouvements
  for the robot based on the willing direction of the mouvement the stepper motors are controlled
  differently and independently in order to achieve the desired mouvement.
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <Stepper.h>
#include <Imu.h>

// Arduino PINS 
// MOTOR 1 - Front Left
#define dirPin_1 2
#define stepPin_1 3
// MOTOR 2 - Front Right
#define dirPin_2 4
#define stepPin_2 5
// MOTOR 3 - Back Left
#define dirPin_3 6
#define stepPin_3 7
// MOTOR 4 - Back Right
#define dirPin_4 8
#define stepPin_4 9

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
    // Motors
    Stepper motor_1 = Stepper(stepPin_1, dirPin_1);
    Stepper motor_2 = Stepper(stepPin_2, dirPin_2);
    Stepper motor_3 = Stepper(stepPin_3, dirPin_3);
    Stepper motor_4 = Stepper(stepPin_4, dirPin_4);
    Imu imu = Imu();

public:
    // Functions
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
};


#endif