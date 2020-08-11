#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <Wire.h>
#include <ros.h>
#include <MPU6050.h>

#ifndef Imu_h
#define Imu_h

#define FRAME_ID "imu_frame"

class Imu
{
    private:
        const int MPU_addr = 0x68;  // I2C address of the MPU-6050
        MPU6050 mpu;
        // linear acceleration
        float AcX;
        float AcY;
        float AcZ;
        // orientation
        int pitch = 0;
        int roll = 0;
        float yaw = 0;
        float toRadians(float degree);
        geometry_msgs::Quaternion quaternionFromRPY(float r, float p, float y);
    public:
        Imu(/* args */);
        ~Imu();
        void sense();
        std_msgs::String composeStringMessage();
        sensor_msgs::Imu composeImuMessage(ros::Time now);
};


#endif
