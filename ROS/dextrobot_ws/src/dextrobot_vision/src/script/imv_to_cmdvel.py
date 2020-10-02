#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

# Motion detections with lower value will be ignored
X_THRESHOLD = 1500
Y_THRESHOLD = 1500
NEW_TRESHOLD = 1500
# Avoid too fast or too frequent mouvements
last_x = 0
last_y = 0

def goLeft():
    twist_left = Twist()
    twist_left.linear.y = 1
    pub.publish(twist_left)
    rospy.sleep(1)
    pub.publish(Twist())

def goRight():
    twist_right = Twist()
    twist_right.linear.y = -1
    pub.publish(twist_right)
    rospy.sleep(1)
    pub.publish(Twist())

def imv_callback(msg):
    global last_x, last_y

    # Move the robot on its left
    if msg.x >= X_THRESHOLD:
        goLeft()

    # Move the robot on its left
    if msg.x <= -X_THRESHOLD:
        goRight()
    
    last_x = msg.x
    last_y = msg.y

if __name__ == '__main__':
	imv_topic = '/imv_aggregate'
	
	pub = rospy.Publisher('dextrobot/cmd_vel', Twist, queue_size=10)
	rospy.init_node('imv_to_cmdvel')
	rospy.Subscriber(imv_topic, Vector3, imv_callback)
	rospy.spin()