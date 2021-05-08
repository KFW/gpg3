#! /usr/bin/env python

# program to wander space randomly using distance sensor and lidar to prevent collisions
# use it to test battery life

import rospy 
import sys
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

FWD_speed = 1
REV_speed = 0.5


def callback(msg):
    



rospy.Subscriber('/distance_sensor/distance', Range, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.spin()