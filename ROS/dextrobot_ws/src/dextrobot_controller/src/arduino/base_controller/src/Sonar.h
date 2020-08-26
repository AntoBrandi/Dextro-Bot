/*
  Sonar.h - DextroBot
  Class that cointains the implementation of a sonar sensor 
  connected to the Arduino that will sense the enviroment and 
  compose ROS messages for the communication
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <NewPing.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <ros.h>

#ifndef Sonar_h
#define Sonar_h

// Sonar parameters
#define MIN_DISTANCE 0
#define MAX_DISTANCE 200 // centimeters - Max distance for obstacle detection
#define FIELD_OF_VIEW 0.26
#define PING_INTERVAL 50 // Milliseconds between two consequent readings

// Ros parameters
#define FRAME_ID "sonar_ranger"


class Sonar
{
    private:
        uint8_t triggerPin;
        uint8_t echoPin;
        

    public:
        NewPing sonar = NewPing(triggerPin, echoPin, MAX_DISTANCE);
        Sonar(uint8_t triggerPin, uint8_t echoPin);
        ~Sonar();
        std_msgs::String composeStringMessage(uint8_t distance);
        sensor_msgs::Range composeRangeMessage(ros::Time now, uint8_t distance);
};


#endif