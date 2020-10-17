#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import math


pub = None


def callback(data):
    global pub
    readings = data.data.split(",")
    sensor_imu = Imu()
    # compose the message
    sensor_imu.header.frame_id = "dextrobot_imu_link"
    sensor_imu.header.stamp = rospy.Time.now()

    q = quaternion_from_euler(float(readings[3]), float(readings[4]), float(readings[5]))

    sensor_imu.orientation.x = q[0]
    sensor_imu.orientation.y = q[1]
    sensor_imu.orientation.z = q[2]
    sensor_imu.orientation.w = q[3]
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
    while not rospy.is_shutdown():
        configure()