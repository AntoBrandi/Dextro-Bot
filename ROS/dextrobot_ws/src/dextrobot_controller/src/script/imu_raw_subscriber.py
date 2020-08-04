#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    readings = data.data.split(",")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(rospy.get_caller_id() + "Splitted "+ readings[0] + readings[1])
    
def listener():
    rospy.init_node('imu_cleaner', anonymous=True)

    rospy.Subscriber("imu_raw", String, callback)

    # simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()