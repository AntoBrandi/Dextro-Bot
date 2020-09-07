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
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

// Topic Names
// PUBLISH TOPIC
#define ROS_TOPIC_IMU "imu_raw"
// SUBSCRIBE TOPIC
#define ROS_TOPIC_CMD_VEL "cmd_vel"

// ROS message publish frequency
#define PUBLISH_DELAY 50 // 10Hz

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publishers
std_msgs::String imu_msg;
ros::Publisher pub_imu(ROS_TOPIC_IMU, &imu_msg);

// Create an instance of the Robot with its methods
Dextrobot robot;

// Record the last time a publish operation happened
unsigned long publisher_timer;

// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Vector3& msg){
  // extract only useful parameters. No linear mouvement possible over z axis
  // and no rotation mouvements possible over x and y axis
  float x_lin = msg.x;
  float y_lin = msg.y;
  float z_ang = msg.z;

  // Move the robot according to the received velocity message
  if(x_lin>0){
    if(y_lin>0){
      // go forward right
      robot.goForwardRight(x_lin);
    } else if (y_lin<0){
      // go forward left
      robot.goForwardLeft(x_lin);
    } else{
      // go forward
      robot.goForward(x_lin);
    }
  }

  if(x_lin<0){
    if(y_lin>0){
      // go backward right
      robot.goBackwardRight(x_lin);
    } else if (y_lin<0){
      // go backward left
      robot.goBackwardLeft(x_lin);
    } else{
      // go backward
      robot.goBackward(x_lin);
    }
  }

  if(z_ang>0){
    // rotate clockwise
    robot.rotateClockwise(z_ang);
  } else if(z_ang<0){
    // rotate counterclockwise
    robot.rotateCounterClockwise(z_ang);
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
  nh.advertise(pub_imu);

  // Init the robot and it's stepper motor
  robot = Dextrobot();

  // Init the accelerometer
  while(!robot.imu.mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
      delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  robot.imu.mpu.calibrateGyro(); 
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  robot.imu.mpu.setThreshold(1);

  publisher_timer = millis();
}

void loop() { 
  // move the stepper motor if needed
  robot.run();

  // read the actual status of the sensors
  robot.sense();

  if (millis() >= publisher_timer) {
    String data = robot.imu.composeStringMessage();
    int length = data.length();
    char data_final[length+1];
    data.toCharArray(data_final, length+1);
    imu_msg.data = data_final;

    // publish the messages
    pub_imu.publish(&imu_msg);    

    // update the last time a message has been published via ROS
    publisher_timer = millis() + PUBLISH_DELAY;
  }

  // Keep ROS Node Up & Running
  nh.spinOnce();
}