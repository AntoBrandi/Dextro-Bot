#include <Arduino.h>
#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>



// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Twist& msg){
  
}


// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onCmdVelMsg );


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}