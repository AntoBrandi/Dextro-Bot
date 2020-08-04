#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <MPU6050.h>

#define TOPIC_NAME "imu_raw"
#define PUBLISH_DELAY 100
#define PI 3.1416

const int MPU_addr=0x68;  // I2C address of the MPU-6050
MPU6050 mpu;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
long publisher_timer;

// angle calculation to rpy
int pitch = 0;
int roll = 0;
float yaw = 0;


//Set up the ros node and publisher
std_msgs::String imu_msg;
ros::Publisher imu(TOPIC_NAME, &imu_msg);
ros::NodeHandle nh;

// convert RPY degrees angles to Radians
float toRadians(float degree){
  return degree*PI/180;
}

                
void setup()
{
  // Init and Configure the ROS node
  nh.initNode();
  nh.advertise(imu);
  
  // Init and configure the communication with the IMU via I2C
  Serial.begin(57600);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);
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
    imu.publish(&imu_msg);
    publisher_timer = millis() + PUBLISH_DELAY; //publish ten times a second
    nh.spinOnce();
  }
  
}