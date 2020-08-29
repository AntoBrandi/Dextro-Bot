#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3


pub = None


def callback(data):
    global pub
    # compose the message
    arduino_vel = Vector3()
    arduino_vel.x = data.linear.x
    arduino_vel.y = data.linear.y
    arduino_vel.z = data.angular.z
    
    # publish the composed message
    pub.publish(arduino_vel)

    
def configure():
    global pub
    rospy.init_node('cmd_vel_converter', anonymous=True)

    rospy.Subscriber("dextrobot/cmd_vel", Twist, callback)
    pub = rospy.Publisher("cmd_vel", Vector3, queue_size=10)

    # simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    configure()