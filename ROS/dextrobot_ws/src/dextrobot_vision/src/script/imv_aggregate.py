#!/usr/bin/python

import rospy
import math
import struct
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from raspicam_node.msg import MotionVectors
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

# Motion vectors with SAD values above threshold will be ignored:
SAD_THRESHOLD = 650

bridge = CvBridge()
last_imv = None
cumulative_x = 0
cumulative_y = 0

def aggregate_imv(img, imv):
	global cumulative_x, cumulative_y
	height, width, channels = img.shape

	# Draw colored arrow per each macroblock:
	for j in range(0, imv.mby):
		for i in range(0, imv.mbx):
			idx = (i + (imv.mbx + 1) * j)
			dx = imv.x[idx]
			dy = imv.y[idx]
			sad = imv.sad[idx]

			if dx == 0 and dy == 0:
				continue

			if sad >= SAD_THRESHOLD:
				continue
				
			cumulative_x = cumulative_x + dx
			cumulative_y = cumulative_y + dy
			
	aggregate_imv = Vector3()
	aggregate_imv.x = cumulative_x
	aggregate_imv.y = cumulative_y
	pub.publish(aggregate_imv)
	cumulative_y = 0
	cumulative_x = 0

def img_callback(msg):
	try:
		img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
	except CvBridgeError as e:
		print(e)
	else:
		if last_imv is not None:
			aggregate_imv(img, last_imv)

def imv_callback(msg):
	global last_imv
	last_imv = msg

if __name__ == '__main__':
	img_topic = '/raspicam_node/image/compressed'
	imv_topic = '/raspicam_node/motion_vectors'
	
	pub = rospy.Publisher('imv_aggregate', Vector3, queue_size=10)
	rospy.init_node('imv_aggregate')
	rospy.Subscriber(img_topic, CompressedImage, img_callback)
	rospy.Subscriber(imv_topic, MotionVectors, imv_callback)
	rospy.spin()
