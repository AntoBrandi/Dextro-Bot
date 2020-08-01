/*
  Arm_Configuration.h - Test library for the computation
  of the inverse kinematic for antropomorphous robot with 3 DoF contorlled with Arduino
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Arduino.h>
#include <Dextrobot.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>



// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Twist& msg){
  
}


// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onCmdVelMsg );

// Create an instance of the Robot with its methods
Dextrobot robot;

void setup() {
  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  nh.subscribe(sub);

  robot = Dextrobot();
}

void loop() {

  // Keep ROS Node Up & Running
  nh.spinOnce();
}