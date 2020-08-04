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
    quaternion = Quaternion()
    lin_acc = Vector3()
    # compose the message
    sensor_imu.header.frame_id = "imu"
    sensor_imu.header.stamp = rospy.Time.now()
    cy = math.cos(readings[5] * 0.5)
    sy = math.sin(readings[5] * 0.5)
    cp = math.cos(readings[4] * 0.5)
    sp = math.sin(readings[4] * 0.5)
    cr = math.cos(readings[3] * 0.5)
    sr = math.sin(readings[3] * 0.5)
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    quaternion.w = cr * cp * cy + sr * sp * sy
    sensor_imu.orientation = quaternion
    lin_acc.x = readings[0]
    lin_acc.y = readings[1]
    lin_acc.z = readings[2]
    sensor_imu.linear_acceleration = lin_acc
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