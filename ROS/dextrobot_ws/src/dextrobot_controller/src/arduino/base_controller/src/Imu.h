#include <Wire.h>
#include <MPU6050.h>

#ifndef Imu_h
#define Imu_h

#define FRAME_ID "imu_frame"

class Imu
{
    private:
        static const int MPU_addr = 0x68;  // I2C address of the MPU-6050
        // linear acceleration
        float AcX = 0;
        float AcY = 0;
        float AcZ = 0;
        // orientation
        int pitch = 0;
        int roll = 0;
        float yaw = 0;
        float toRadians(float degree);
    public:
        MPU6050 mpu;
        Imu(/* args */);
        ~Imu();
        void sense();
        String composeStringMessage();
};


#endif
