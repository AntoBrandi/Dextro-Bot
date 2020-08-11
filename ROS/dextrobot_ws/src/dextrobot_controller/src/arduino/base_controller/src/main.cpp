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

// Topic Names
#define ROS_TOPIC_IMU "imu_raw"
#define ROS_TOPIC_SONAR_FRONT "sonar_front"
#define ROS_TOPIC_SONAR_LEFT "sonar_left"
#define ROS_TOPIC_SONAR_RIGHT "sonar_right"
#define ROS_TOPIC_SONAR_BACK "sonar_back"
#define ROS_SONAR_FRAME_ID "sonar_ranger"

// ROS message publish frequency
#define PUBLISH_DELAY 100 // 10Hz

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publishers


// Create an instance of the Robot with its methods
Dextrobot robot;

// Record the last time a publish operation happened
long publisher_timer;


// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Twist& msg){
  geometry_msgs::Vector3 linear = msg.linear;
  geometry_msgs::Vector3 angular = msg.angular;
  // extract only useful parameters. No linear mouvement possible over z axis
  // and no rotation mouvements possible over x and y axis
  float x_lin = linear.x;
  float y_lin = linear.y;
  float z_ang = angular.z;
  // int x_lin_steps = robot.convertToStepsPerSecond(x_lin);
  // int y_lin_steps = robot.convertToStepsPerSecond(y_lin);
  // int z_ang_steps = robot.convertToStepsPerSecond(z_ang);

  // TODO: move the robot according to the received velocity message

  // Ping back
  // std_msgs/String does not accept string but char array
  // String response = "Applying X: " + String(x_lin_steps) + " Y: " + String(y_lin_steps) + " Rotation Z: " + String(z_ang_steps);
  // char p[response.length()];
  // for (int i=0; i<sizeof(p);i++){
  //   p[i] = response[i];
  // }
  // message.data = p;
  // pub.publish(&message);
}


// Subscribers
// CMD_VEL subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onCmdVelMsg );


void setup() {
  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  // Register the Subscribers
  nh.subscribe(sub);
  // Register the Publishers

  // Init the robot and it's stepper motor
  robot = Dextrobot();
}

void loop() { 

  if (millis() > publisher_timer) {

    // update the last time a message has been published via ROS
    publisher_timer = millis() + PUBLISH_DELAY;
  }

  // Keep ROS Node Up & Running
  nh.spinOnce();
}