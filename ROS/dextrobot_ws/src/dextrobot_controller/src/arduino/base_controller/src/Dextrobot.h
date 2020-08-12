/*
  Dextrobot.h - DextroBot
  Class that cointains the implementation of the low level omnidirectional mouvements
  for the robot based on the willing direction of the mouvement the stepper motors are controlled
  differently and independently in order to achieve the desired mouvement.
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <Stepper.h>
#include <Imu.h>
#include <Sonar.h>

// Arduino PINS 
// MOTOR 1 - Front Left
#define DIR_STEPPER_1 2
#define STEP_STEPPER_1 3
// MOTOR 2 - Front Right
#define DIR_STEPPER_2 4
#define STEP_STEPPER_2 5
// MOTOR 3 - Back Left
#define DIR_STEPPER_3 6
#define STEP_STEPPER_3 7
// MOTOR 4 - Back Right
#define DIR_STEPPER_4 8
#define STEP_STEPPER_4 9
// SONAR 1 - Front sonar
#define TRIGGER_FRONT_SONAR 3
#define ECHO_FRONT_SONAR 2
// SONAR 2 - Left sonar
#define TRIGGER_LEFT_SONAR 5
#define ECHO_LEFT_SONAR 4
// SONAR 3 - Right sonar
#define TRIGGER_RIGHT_SONAR 7
#define ECHO_RIGHT_SONAR 6
// SONAR 4 - Back sonar
#define TRIGGER_BACK_SONAR 9
#define ECHO_BACK_SONAR 8

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
    Stepper motor_1 = Stepper(STEP_STEPPER_1, DIR_STEPPER_1);
    Stepper motor_2 = Stepper(STEP_STEPPER_2, DIR_STEPPER_2);
    Stepper motor_3 = Stepper(STEP_STEPPER_3, DIR_STEPPER_3);
    Stepper motor_4 = Stepper(STEP_STEPPER_4, DIR_STEPPER_4);
    Imu imu = Imu();
    Sonar sonar_1 = Sonar(TRIGGER_FRONT_SONAR, ECHO_FRONT_SONAR);
    Sonar sonar_2 = Sonar(TRIGGER_LEFT_SONAR, ECHO_LEFT_SONAR);
    Sonar sonar_3 = Sonar(TRIGGER_RIGHT_SONAR, ECHO_RIGHT_SONAR);
    Sonar sonar_4 = Sonar(TRIGGER_BACK_SONAR, ECHO_BACK_SONAR);
    

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