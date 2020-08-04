#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import math


pub = None


def callback(data):
    global pub
    readings = data.data.split(",")
    sensor_imu = Imu()
    # compose the message
    sensor_imu.header.frame_id = "imu"
    sensor_imu.header.stamp = rospy.Time.now()
    cy = math.cos(float(readings[5]) * 0.5)
    sy = math.sin(float(readings[5]) * 0.5)
    cp = math.cos(float(readings[4]) * 0.5)
    sp = math.sin(float(readings[4]) * 0.5)
    cr = math.cos(float(readings[3]) * 0.5)
    sr = math.sin(float(readings[3]) * 0.5)
    sensor_imu.orientation.x = sr * cp * cy - cr * sp * sy
    sensor_imu.orientation.y = cr * sp * cy + sr * cp * sy
    sensor_imu.orientation.z = cr * cp * sy - sr * sp * cy
    sensor_imu.orientation.w = cr * cp * cy + sr * sp * sy
    sensor_imu.linear_acceleration.x = float(readings[0])
    sensor_imu.linear_acceleration.y = float(readings[1])
    sensor_imu.linear_acceleration.z = float(readings[2])
    # publish the composed message
    pub.publish(sensor_imu)

    
def configure():
    global pub
    rospy.init_node('imu_cleaner', anonymous=True)

    rospy.Subscriber("imu_raw", String, callback)
    pub = rospy.Publisher("imu", Imu, queue_size=10)

    # simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    configure()