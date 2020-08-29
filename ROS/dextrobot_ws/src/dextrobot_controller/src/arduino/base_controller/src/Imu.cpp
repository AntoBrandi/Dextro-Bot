#include <Imu.h>

Imu::Imu(/* args */)
{
}

Imu::~Imu()
{
}

// convert RPY degrees angles to Radians
float Imu::toRadians(float degree){
  return degree*PI/180;
}

// reads the value coming from the IMU sensor and update the class parameters
void Imu::sense(){
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

    AcX = normAccel.XAxis;
    AcY = normAccel.YAxis;
    AcZ = normAccel.ZAxis;
}


// Get the data coming from the IMU sensor and arrange those in a ROS string message
String Imu::composeStringMessage(){

    String data = String(AcX) + "," + String(AcY) + "," + String(AcZ) + "," + String(toRadians(roll)) + ","+ String(toRadians(pitch)) + "," + String(toRadians(yaw));

    return data;
}
