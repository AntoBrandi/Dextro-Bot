#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <MPU6050.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

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
// Arduino PINS 
// CNC Shield
#define ENABLE_SHIELD 8
// MOTOR 1 - Front Left
#define DIR_STEPPER_1 5
#define STEP_STEPPER_1 2
// MOTOR 2 - Front Right
#define DIR_STEPPER_2 6
#define STEP_STEPPER_2 3
// MOTOR 3 - Back Left
#define DIR_STEPPER_3 7
#define STEP_STEPPER_3 4
// MOTOR 4 - Back Right
#define DIR_STEPPER_4 13
#define STEP_STEPPER_4 12
// SONAR 1 - Front sonar
#define SONAR_NUM 4
#define TRIGGER_FRONT_SONAR 30
#define ECHO_FRONT_SONAR 31
// SONAR 2 - Left sonar
#define TRIGGER_LEFT_SONAR 22
#define ECHO_LEFT_SONAR 23
// SONAR 3 - Right sonar
#define TRIGGER_RIGHT_SONAR 26
#define ECHO_RIGHT_SONAR 27
// SONAR 4 - Back sonar
#define TRIGGER_BACK_SONAR 28
#define ECHO_BACK_SONAR 29


#define PUBLISH_DELAY 100
#define SONAR_DELAY 33
#define PI 3.1416

const int MPU_addr=0x68;  // I2C address of the MPU-6050
MPU6050 mpu;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// Record the last time a publish operation happened
unsigned long publisher_timer;

// Record the last time the sonar sensor have been triggered
unsigned long sonar_timer;

// sonars readings
int distances[SONAR_NUM]={0,0,0,0};

// angle calculation to rpy
int pitch = 0;
int roll = 0;
float yaw = 0;


//Set up the ros node and publisher
// Init a ROS node on the Arduino controller
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
ros::NodeHandle nh;



// convert RPY degrees angles to Radians
float toRadians(float degree){
  return degree*PI/180;
}

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
  // if(x_lin>0){
  //   if(y_lin>0){
  //     // go forward right
  //     robot.goForwardRight(x_lin);
  //   } else if (y_lin<0){
  //     // go forward left
  //     robot.goForwardLeft(x_lin);
  //   } else{
  //     // go forward
  //     robot.goForward(x_lin);
  //   }
  // }

  // if(x_lin<0){
  //   if(y_lin>0){
  //     // go backward right
  //     robot.goBackwardRight(x_lin);
  //   } else if (y_lin<0){
  //     // go backward left
  //     robot.goBackwardLeft(x_lin);
  //   } else{
  //     // go backward
  //     robot.goBackward(x_lin);
  //   }
  // }

  // if(z_ang>0){
  //   // rotate clockwise
  //   robot.rotateClockwise(z_ang);
  // } else if(z_ang<0){
  //   // rotate counterclockwise
  //   robot.rotateCounterClockwise(z_ang);
  // }

  // if(x_lin==0 && y_lin==0 && z_ang==0){
  //   // stop the robot
  //   robot.stop();
  // }

  // // debug message
  // String dbg = "Motors speeds:/n1: "+String(robot.motor_1.speed)+"/n2: "+String(robot.motor_2.speed)+"/n3: "+String(robot.motor_3.speed)+"/n4: "+String(robot.motor_4.speed);
  // int length = dbg.length();
  // char data_final[length+1];
  // dbg.toCharArray(data_final, length+1);
  // debug_msg.data = data_final;
  // pub_debug.publish(&debug_msg);
  
}

// Subscribers
// CMD_VEL 
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(ROS_TOPIC_CMD_VEL, onCmdVelMsg );

                
void setup()
{
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
  
  // Init and configure the communication with the IMU via I2C
  Serial.begin(57600);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);

  publisher_timer = millis();
  sonar_timer = millis();
}

void loop()
{
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  
  //Ignore the gyro if our angular velocity does not meet our threshold
  if (normGyro.ZAxis > 1 || normGyro.ZAxis < -1) {
    normGyro.ZAxis /= 100;
    yaw += normGyro.ZAxis;
  }

   //Keep our angle between 0-359 degrees
  if (yaw < 0)
    yaw += 360;
  else if (yaw > 359)
    yaw -= 360;

  String data = String(normAccel.XAxis) + "," + String(normAccel.YAxis) + "," + String(normAccel.ZAxis) + "," + String(toRadians(roll)) + ","+ String(toRadians(pitch)) + "," + String(toRadians(yaw));

  int length = data.length();
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  
  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    imu_msg.data = data_final;
    pub_imu.publish(&imu_msg);
    publisher_timer = millis() + PUBLISH_DELAY; //publish ten times a second
    nh.spinOnce();
  }
  
}