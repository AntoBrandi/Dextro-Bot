/*
  main.cpp - DextroBot
  Omnidirectional robot powered by ROS and controlled via Arduino Mega.

  Main function that will constantly run on the Arduino Mega and will serve
  as ROS node for publishing sensors readings and for control the stepper motors
  that will act directly on the wheels of the robot based on the messages
  that are published on the ROS topic /cmd_vel
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Arduino.h>
#include <Dextrobot.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publisher Node
std_msgs::String message;
ros::Publisher pub("chatter", &message);


// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Twist& msg){
  geometry_msgs::Vector3 linear = msg.linear;
  geometry_msgs::Vector3 angular = msg.angular;
  float x_lin = linear.x;
  float y_lin = linear.y;
  float z_lin = linear.z;
  float x_ang = angular.x;
  float y_ang = angular.y;
  float z_ang = angular.z;

  // Ping back
  // std_msgs/String does not accept string but char array
  String response = "Received X: " + String(x_lin) + " Y: " + String(y_lin) + " Z: " + String(z_lin);
  char p[response.length()];
  for (int i=0; i<sizeof(p);i++){
    p[i] = response[i];
  }
  message.data = p;
  pub.publish(&message);
}


// Subscriber Node
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onCmdVelMsg );



// Create an instance of the Robot with its methods
Dextrobot robot;

void setup() {
  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  // Register the subscriber
  nh.subscribe(sub);
  // Register the Publisher
  nh.advertise(pub);

  // Init the robot and it's stepper motor
  robot = Dextrobot();
}

void loop() { 

  // Keep ROS Node Up & Running
  nh.spinOnce();
}