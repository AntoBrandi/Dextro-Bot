/*
  Sonar.h - DextroBot
  Class that cointains the implementation of a sonar sensor 
  connected to the Arduino that will sense the enviroment and 
  compose ROS messages for the communication
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <sensor_msgs/Range.h>
#include <ros.h>

#ifndef Sonar_h
#define Sonar_h

// Sonar parameters
#define MIN_DISTANCE 0
#define MAX_DISTANCE 200 // centimeters - Max distance for obstacle detection
#define FIELD_OF_VIEW 0.26
#define PING_INTERVAL 33 // Milliseconds between two consequent readings
// Kalman Filter parameters
#define KALMAN_E_MEA 2
#define KALMAN_E_EST 2
#define KALMAN_Q 0.01

// Ros parameters
#define FRAME_ID "sonar_ranger"


class Sonar
{
    private:
        uint8_t triggerPin;
        uint8_t echoPin;
        long lastScan = 0;
        NewPing sonar = NewPing(triggerPin, echoPin, MAX_DISTANCE);

        // Create a Kalman Filter that will be applied to the readings of all the sonar sensors
        // e_mea: Measurement Uncertainty
        // e_est: Estimation Uncertainty
        // q: Process Noise
        SimpleKalmanFilter KFilter = SimpleKalmanFilter(KALMAN_E_MEA, KALMAN_E_EST, KALMAN_Q);

        // readings 
        uint8_t distance = 0;

    public:
        Sonar(uint8_t triggerPin, uint8_t echoPin);
        ~Sonar();
        void sense();
        sensor_msgs::Range composeRangeMessage(ros::Time now);
};


#endif