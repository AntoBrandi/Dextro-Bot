#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Motion detections with lower value will be ignored
X_THRESHOLD = 1500
Y_THRESHOLD = 1500
TIME_TRESHOLD = 4
# Avoid too fast or too frequent mouvements
last_msg = 0
# Enable or disable the followme
isEnabled = False

def goLeft():
	twist_left = Twist()
	twist_left.linear.y = 0.5
	pub.publish(twist_left)
	rospy.sleep(2)
	pub.publish(Twist())

def goRight():
	twist_right = Twist()
	twist_right.linear.y = -0.5
	pub.publish(twist_right)
	rospy.sleep(2)
	pub.publish(Twist())

def goForward():
	twist_forward = Twist()
	twist_forward.linear.x = 0.5
	pub.publish(twist_forward)
	rospy.sleep(2)
	pub.publish(Twist())

def goBackward():
	twist_backward = Twist()
	twist_backward.linear.x = -0.5
	pub.publish(twist_backward)
	rospy.sleep(2)
	pub.publish(Twist())

def imv_callback(msg):
	global last_msg, isEnabled
	upcoming_msg = rospy.get_time()

	if isEnabled:
		if (upcoming_msg-last_msg)>=TIME_TRESHOLD:
			# Move right or left
			if abs(msg.x)>=abs(msg.y):
				# Move the robot on its left
				if msg.x >= X_THRESHOLD:
					last_msg = upcoming_msg
					goLeft()

				# Move the robot on its right
				if msg.x <= -X_THRESHOLD:
					last_msg = upcoming_msg
					goRight()
			# Move forward or backward
			else:
				# Move the robot on its back
				if msg.y >= Y_THRESHOLD:
					last_msg = upcoming_msg
					goBackward()

				# Move the robot on its front
				if msg.y <= -Y_THRESHOLD:
					last_msg = upcoming_msg
					goForward()

def enable_callback(msg):
	global isEnabled
	isEnabled = msg.data


if __name__ == '__main__':
	imv_topic = '/imv_aggregate'
	enable_topic = '/dextrobot/followme'

	pub = rospy.Publisher('dextrobot/cmd_vel', Twist, queue_size=10)
	rospy.init_node('imv_to_cmdvel')
	rospy.Subscriber(imv_topic, Vector3, imv_callback)
	rospy.Subscriber(enable_topic, Bool, enable_callback)
	rospy.spin()