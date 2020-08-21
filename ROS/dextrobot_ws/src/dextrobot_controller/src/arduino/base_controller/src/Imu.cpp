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
std_msgs::String Imu::composeStringMessage(){
    std_msgs::String imu_msg;

    String data = String(AcX) + "," + String(AcY) + "," + String(AcZ) + "," + String(toRadians(roll)) + ","+ String(toRadians(pitch)) + "," + String(toRadians(yaw));

    int length = data.length();
    char data_final[length+1];
    data.toCharArray(data_final, length+1);
    imu_msg.data = data_final;

    return imu_msg;
}

// !!! ALERT !!!
// Use this function only if you are using and Arduino MEGA or similar. Otherwise
// this message will not be sent over ROS because it is too large for an Arduino UNO or similar
sensor_msgs::Imu Imu::composeImuMessage(ros::Time now){
    sensor_msgs::Imu imu_msg;
    // compose the header
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = FRAME_ID;

    // compose the body
    imu_msg.linear_acceleration.x = AcX;
    imu_msg.linear_acceleration.y = AcY;
    imu_msg.linear_acceleration.z = AcZ;

    // Convert RPY to quaternion
    float r = toRadians(roll);
    float p = toRadians(pitch);
    float y = toRadians(yaw);

    geometry_msgs::Quaternion quaternion;

    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);

    quaternion.w = cr * cp * cy + sr * sp * sy;
    quaternion.x = sr * cp * cy - cr * sp * sy;
    quaternion.y = cr * sp * cy + sr * cp * sy;
    quaternion.z = cr * cp * sy - sr * sp * cy;

    imu_msg.orientation = quaternion;

    return imu_msg;
}

