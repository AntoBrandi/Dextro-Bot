#!/usr/bin/env python

'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
'''
import smbus			#import SMBus module of I2C
from time import sleep          #import
import math
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# useful constants 
M_PI = 3.14159265358979323846

# init ROS publisher
pub = rospy.Publisher('imu', Imu, queue_size=10)
rospy.init_node('imu_sense', anonymous=True)
rate = rospy.Rate(10) # 10hz

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()
roll = 0.0
pitch = 0.0
yaw = 0.0
Ax = 0.0
Ay = 0.0
Az = 0.0


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


def publish_imu():
    global pub, roll, pitch, yaw, Ax, Ay, Az, rate
    
    # compose the message
    sensor_imu = Imu()
    sensor_imu.header.frame_id = "dextrobot_imu_link"
    sensor_imu.header.stamp = rospy.Time.now()

    q = quaternion_from_euler(roll, pitch, yaw)

    sensor_imu.orientation.x = q[0]
    sensor_imu.orientation.y = q[1]
    sensor_imu.orientation.z = q[2]
    sensor_imu.orientation.w = q[3]
    sensor_imu.linear_acceleration.x = float(Ax)
    sensor_imu.linear_acceleration.y = float(Ay)
    sensor_imu.linear_acceleration.z = float(Az)

    # publish the composed message
    if not rospy.is_shutdown():
        pub.publish(sensor_imu)
        rate.sleep()


if __name__ == '__main__':
    print (" Reading Data of Gyroscope and Accelerometer")

    while True:
        
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        
        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        # Calculate Pitch & Roll
        pitch = -(math.atan2(Ax, math.sqrt(Ay*Ay + Az*Az))*180.0)/M_PI
        roll = (math.atan2(Ay, Az)*180.0)/M_PI
            
        # Ignore the gyro if our angular velocity does not meet our threshold
        if (Gz > 1 || Gz < -1) {
            Gz /= 100;
            yaw += Gz;
        }

        # Keep our angle between 0-359 degrees
        if (yaw < 0)
            yaw += 360;
        else if (yaw > 359)
            yaw -= 360;
        
        publish_imu()
        print ("Roll=%.2f" %roll, u'\u00b0'+ "/s", "\tPitch=%.2f" %pitch, u'\u00b0'+ "/s", "\tYaw=%.2f" %yaw, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	