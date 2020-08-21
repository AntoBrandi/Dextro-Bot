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
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

// Topic Names
// PUBLISH TOPIC
#define ROS_TOPIC_IMU "imu_raw"
#define ROS_TOPIC_RANGE_FRONT "range_front"
#define ROS_TOPIC_RANGE_LEFT "range_left"
#define ROS_TOPIC_RANGE_RIGHT "range_right"
#define ROS_TOPIC_RANGE_BACK "range_back"
#define ROS_TOPIC_DEBUG "debug"
// SUBSCRIBE TOPIC
#define ROS_TOPIC_CMD_VEL "cmd_vel"

// ROS message publish frequency
#define PUBLISH_DELAY 100 // 10Hz

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publishers
sensor_msgs::Imu imu_msg;
sensor_msgs::Range range_front_msg;
sensor_msgs::Range range_left_msg;
sensor_msgs::Range range_right_msg;
sensor_msgs::Range range_back_msg;
std_msgs::String debug_msg;
ros::Publisher pub_range_front(ROS_TOPIC_RANGE_FRONT, &range_front_msg);
ros::Publisher pub_range_left(ROS_TOPIC_RANGE_LEFT, &range_left_msg);
ros::Publisher pub_range_right(ROS_TOPIC_RANGE_RIGHT, &range_right_msg);
ros::Publisher pub_range_back(ROS_TOPIC_RANGE_BACK, &range_back_msg);
ros::Publisher pub_imu(ROS_TOPIC_IMU, &imu_msg);
ros::Publisher pub_debug(ROS_TOPIC_DEBUG, &debug_msg);

// Create an instance of the Robot with its methods
Dextrobot robot;

// Record the last time a publish operation happened
unsigned long publisher_timer;

// Keep track of the next action that each stepper should take
int action = 0;
float velocity = 0;


// Callback function that is called once a message is published on the topic /cmd_vel on which this Arduino is subscribed
void onCmdVelMsg(const geometry_msgs::Twist& msg){
  geometry_msgs::Vector3 linear = msg.linear;
  geometry_msgs::Vector3 angular = msg.angular;
  // extract only useful parameters. No linear mouvement possible over z axis
  // and no rotation mouvements possible over x and y axis
  float x_lin = linear.x;
  float y_lin = linear.y;
  float z_ang = angular.z;

  // Move the robot according to the received velocity message
  if(x_lin>0){
    velocity = x_lin;
    if(y_lin>0){
      // go forward right
      action = 3;
    } else if (y_lin<0){
      // go forward left
      action = 2;
    } else{
      // go forward
      action = 1;
    }
  }

  if(x_lin<0){
    velocity = x_lin;
    if(y_lin>0){
      // go backward right
      action = 6;
    } else if (y_lin<0){
      // go backward left
      action = 5;
    } else{
      // go backward
      action = 4;
    }
  }

  if(z_ang>0){
    velocity = z_ang;
    // rotate clockwise
    action = 7;
  } else if(z_ang<0){
    velocity = z_ang;
    // rotate counterclockwise
    action = 8;
  }

  if(x_lin==0 && y_lin==0 && z_ang==0){
    velocity = 0;
    // stop the robot
    action = 0;
  }
}


// Subscribers
// CMD_VEL 
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(ROS_TOPIC_CMD_VEL, onCmdVelMsg );


void setup() {
  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  // Register the Subscribers
  nh.subscribe(sub_cmd_vel);
  // Register the Publishers
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);
  nh.advertise(pub_range_back);
  nh.advertise(pub_imu);
  nh.advertise(pub_debug);

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
  switch (action)
  {
    case 0:
      robot.stop();
      break;
    case 1:
      robot.goForward(velocity);
      break;
    case 2:
      robot.goForwardLeft(velocity);
      break;
    case 3:
      robot.goForwardRight(velocity);
      break;
    case 4:
      robot.goBackward(velocity);
      break;
    case 5:
      robot.goBackwardLeft(velocity);
      break;
    case 6:
      robot.goBackwardRight(velocity);
      break;
    case 7:
      robot.rotateClockwise(velocity);
      break;
    case 8:
      robot.rotateCounterClockwise(velocity);
      break; 
    default:
      robot.stop();
      break;
  }
  // read the actual status of the sensors
  robot.sense();

  if (millis() >= publisher_timer) {
    // compose and publish the sensor messages
    range_front_msg = robot.sonar_1.composeRangeMessage(nh.now());
    range_left_msg = robot.sonar_2.composeRangeMessage(nh.now());
    range_right_msg = robot.sonar_3.composeRangeMessage(nh.now());
    range_back_msg = robot.sonar_4.composeRangeMessage(nh.now());
    imu_msg = robot.imu.composeImuMessage(nh.now());

    // publish the messages
    pub_range_front.publish(&range_front_msg);
    pub_range_left.publish(&range_left_msg);
    pub_range_right.publish(&range_right_msg);
    pub_range_back.publish(&range_back_msg);
    pub_imu.publish(&imu_msg);    

    // update the last time a message has been published via ROS
    publisher_timer = millis() + PUBLISH_DELAY;
  }

  // Keep ROS Node Up & Running
  nh.spinOnce();
}