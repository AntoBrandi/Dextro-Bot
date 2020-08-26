/*
  Sonar.cpp - DextroBot
  Class that cointains the implementation of a sonar sensor 
  connected to the Arduino that will sense the enviroment and 
  compose ROS messages for the communication
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Sonar.h>


Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin) : triggerPin(triggerPin), echoPin(echoPin)
{
}

Sonar::~Sonar()
{
}


// Compose the ROS message with the sensor data
sensor_msgs::Range Sonar::composeRangeMessage(ros::Time now, uint8_t distance){
    sensor_msgs::Range range_msg;
    // compose the header
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = FRAME_ID;
    range_msg.field_of_view = FIELD_OF_VIEW;
    range_msg.min_range = MIN_DISTANCE;
    range_msg.max_range = MAX_DISTANCE/100;
    range_msg.header.stamp = now;

    // compose the body
    range_msg.range = (float)distance/100;  


    return range_msg;
}

// Compose the ROS message with the sensor data in a string
std_msgs::String Sonar::composeStringMessage(uint8_t distance){
    std_msgs::String string_msg;
    String data = String((float)distance/100);
    int length = data.length();
    char data_final[length+1];
    data.toCharArray(data_final, length+1);
    string_msg.data = data_final;
    return string_msg;
}