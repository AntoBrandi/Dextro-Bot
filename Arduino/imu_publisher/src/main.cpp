#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

// Topic name
#define TOPIC_IMU "/imu"

// Init IMU parameters
const int MPU = 0x68; 
// not filtered sensor readings
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// Init the ROS node
ros::NodeHandle nh;

// Message that will be created and published on ros
sensor_msgs::Imu imu;
 
// Topics on which the sonar messages will be published
ros::Publisher pub_imu(TOPIC_IMU, &imu);

// Functions
void composeMessage(){
  imu.header.frame_id = TOPIC_IMU;
  imu.header.stamp = nh.now();
  geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
  // Create quaternion from euler calculation
  float cy = cos(GyZ * 0.5);
  float sy = sin(GyZ * 0.5);
  float cp = cos(GyY * 0.5);
  float sp = sin(GyY * 0.5);
  float cr = cos(GyX * 0.5);
  float sr = sin(GyX * 0.5);
  quat.x = sr * cp * cy - cr * sp * sy;
  quat.y = cr * sp * cy + sr * cp * sy;
  quat.z = cr * cp * sy - sr * sp * cy;
  quat.w = cr * cp * cy + sr * sp * sy;
  imu.orientation = quat;
  geometry_msgs::Vector3 lin_acc = geometry_msgs::Vector3();
  lin_acc.x = AcX;
  lin_acc.y = AcY;
  lin_acc.z = AcZ;
  imu.linear_acceleration = lin_acc;
}


void setup() {
  // setup the communication with the IMU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  // init the publisher node
  nh.initNode();

  // register a publisher for each topic for each sensors
  nh.advertise(pub_imu);
}

void loop() {
  // Read values from the IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX = Wire.read()<<8|Wire.read();    
  AcY = Wire.read()<<8|Wire.read();  
  AcZ = Wire.read()<<8|Wire.read();  
  GyX = Wire.read()<<8|Wire.read();  
  GyY = Wire.read()<<8|Wire.read();  
  GyZ = Wire.read()<<8|Wire.read(); 


  // compose the ROS message of type sensor_msgs/Imu
  composeMessage();

  // publish the message
  pub_imu.publish(&imu);


  // keep the ROS node up and running
  nh.spinOnce();

  // not that fast
  delay(100);
}