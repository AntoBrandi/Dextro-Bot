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
// PUBLISH TOPIC
#define ROS_TOPIC_IMU "imu_raw"
#define ROS_TOPIC_RANGE_FRONT "range_front_raw"
#define ROS_TOPIC_RANGE_LEFT "range_left_raw"
#define ROS_TOPIC_RANGE_RIGHT "range_right_raw"
#define ROS_TOPIC_RANGE_BACK "range_back_raw"
#define ROS_TOPIC_DEBUG "debug"
// SUBSCRIBE TOPIC
#define ROS_TOPIC_CMD_VEL "cmd_vel"

// ROS message publish frequency
// TODO: tune the frequency
#define PUBLISH_DELAY 100 // 10Hz
#define SONAR_DELAY 33

// Init a ROS node on the Arduino controller
ros::NodeHandle nh;
// Publishers
std_msgs::String imu_msg;
std_msgs::String range_front_msg;
std_msgs::String range_left_msg;
std_msgs::String range_right_msg;
std_msgs::String range_back_msg;
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

// Record the last time the sonar sensor have been triggered
unsigned long sonar_timer;

// sonars readings
uint8_t distances[SONAR_NUM]={0,0,0,0};

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

  // debug message
  String dbg = "Motors speeds:/n1: "+String(robot.motor_1.speed)+"/n2: "+String(robot.motor_2.speed)+"/n3: "+String(robot.motor_3.speed)+"/n4: "+String(robot.motor_4.speed);
  int length = dbg.length();
  char data_final[length+1];
  dbg.toCharArray(data_final, length+1);
  debug_msg.data = data_final;
  pub_debug.publish(&debug_msg);
  
}

void echoSonar1Check(){
  distances[0] = robot.sonar_1.sonar.ping_result / US_ROUNDTRIP_CM;
}
void echoSonar2Check(){
  distances[1] = robot.sonar_2.sonar.ping_result / US_ROUNDTRIP_CM;
}
void echoSonar3Check(){
  distances[2] = robot.sonar_3.sonar.ping_result / US_ROUNDTRIP_CM;
}
void echoSonar4Check(){
  distances[3] = robot.sonar_4.sonar.ping_result / US_ROUNDTRIP_CM;
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
  sonar_timer = millis();
}

void loop() { 
  // move the stepper motor if needed
  robot.run();

  // read the actual status of the sensors
  robot.sense();
  if(millis() >= sonar_timer){
    robot.sonar_1.sonar.timer_stop(); 
    robot.sonar_1.sonar.ping_timer(echoSonar1Check);

    robot.sonar_2.sonar.timer_stop(); 
    robot.sonar_2.sonar.ping_timer(echoSonar2Check);

    robot.sonar_3.sonar.timer_stop(); 
    robot.sonar_3.sonar.ping_timer(echoSonar3Check);

    robot.sonar_4.sonar.timer_stop(); 
    robot.sonar_4.sonar.ping_timer(echoSonar4Check);

    sonar_timer = millis() + SONAR_DELAY;
  }

  // publish the sensor readings to ROS
  if (millis() >= publisher_timer) {
    // compose and publish the sensor messages
    range_front_msg = robot.sonar_1.composeStringMessage(distances[0]);
    range_left_msg = robot.sonar_2.composeStringMessage(distances[1]);
    range_right_msg = robot.sonar_3.composeStringMessage(distances[2]);
    range_back_msg = robot.sonar_4.composeStringMessage(distances[3]);
    imu_msg = robot.imu.composeStringMessage();

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


