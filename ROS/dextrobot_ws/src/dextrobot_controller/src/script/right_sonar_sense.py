#!/usr/bin/env python
#Libraries
import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
#set GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 12

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

rospy.init_node('right_sonar_sense', anonymous=True)
pub = rospy.Publisher("range_right", Range, queue_size=10)

distance = 0.0
 
def get_distance():
    global distance

    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

def publish_range():
    global distance, pub

    # compose the message
    sensor_range = Range()
    sensor_range.header.stamp = rospy.Time.now()
    sensor_range.radiation_type = Range.ULTRASOUND
    sensor_range.field_of_view = 0.26
    sensor_range.min_range = 0
    sensor_range.max_range = 2
    sensor_range.range = distance/100 # distance in meters
    sensor_range.header.frame_id = "dextrobot_right_sonar"

    # publish the composed message
    pub.publish(sensor_range)
 
if __name__ == '__main__':
    while not rospy.is_shutdown():
        get_distance()
        publish_range()
        # wait before the next itheration
        time.sleep(0.05)