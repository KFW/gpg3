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

# globals to hold ultrasound distance so it can be used in lidar callback
us_dist = 6 


rospy.init_node("wander")

def callback_lidar(lidar_msg):
    global us_dist

# def callback_dist(dist_msg):
#    global us_dist 
#    pass
    


sub_lidar = rospy.Subscriber('/scan', LaserScan, callback_lidar)
# sub_dist =  rospy.Subscriber('/distance_sensor/distance', Range, callback_dist)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.spin()