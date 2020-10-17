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
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <Dextrobot.h>

// Topic Names
// SUBSCRIBE TOPIC
#define ROS_TOPIC_CMD_VEL "cmd_vel"
// PUBLISH TOPIC
#define ROS_TOPIC_DEBUG "debug"

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publishers
// DEBUG
std_msgs::String debug_msg;
ros::Publisher pub_debug(ROS_TOPIC_DEBUG, &debug_msg);

// Create an instance of the Robot with its methods
Dextrobot robot;

// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Vector3& msg){
  // extract only useful parameters. No linear mouvement possible over z axis
  // and no rotation mouvements possible over x and y axis
  float x_lin = msg.x;
  float y_lin = msg.y;
  float z_ang = msg.z;

  String data = "x:"+String(x_lin)+"y:"+String(y_lin)+"z:"+String(z_ang);
  int length = data.length();
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  debug_msg.data = data_final;
  pub_debug.publish(&debug_msg);

  // Move the robot according to the received velocity message
  if((abs(x_lin)>abs(z_ang))&&(abs(x_lin)>abs(y_lin))){
    if(x_lin>0){
      robot.goForward(x_lin);
    }

    if(x_lin<0){
      robot.goBackward(x_lin);
    }
  }
  else if(abs(y_lin)>abs(z_ang)){
    if(y_lin>0){
      robot.goLeft(y_lin);
    }
    if(y_lin<0){
      robot.goRight(y_lin);
    }
  }
  else{
    if(z_ang>0){
      // rotate clockwise
      robot.rotateClockwise(z_ang);
    } else if(z_ang<0){
      // rotate counterclockwise
      robot.rotateCounterClockwise(z_ang);
    }
  }
 

  if(x_lin==0 && y_lin==0 && z_ang==0){
    // stop the robot
    robot.stop();
  }
}

// Subscribers
// CMD_VEL 
ros::Subscriber<geometry_msgs::Vector3> sub_cmd_vel(ROS_TOPIC_CMD_VEL, onCmdVelMsg );



void setup() {
  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  // Register the Subscribers
  nh.subscribe(sub_cmd_vel);
  // Register the Publishers
  nh.advertise(pub_debug);

  // Init the robot and it's stepper motor
  robot = Dextrobot();
}

void loop() { 
  robot.run();
  // Keep ROS Node Up & Running
  nh.spinOnce();
}