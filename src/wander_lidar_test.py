#! /usr/bin/env python

# program to wander space randomly using lidar (and maybe distance sensor) to prevent collisions


import rospy 
import sys
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

rospy.init_node("wander_lidar_test")

# def min_range(ranges):
#     min_r = 2.0
#     for r in ranges:
#         if r != 0:  # exclude 0 since we know there are a lot of zero values in data
#             if r < min_r:
#                 min_r = r
#     return min_r

def look_ahead(lidar_msg):
    # for testing look L, C, R
    # lidar reports ranges in 0.5 degree increments; 0 is straight back, 360 straight ahead
    l = lidar_msg.ranges[180]
    c = lidar_msg.ranges[360]
    r = lidar_msg.ranges[540]
    rospy.loginfo('range L: %.3f  range C: %.3f min R: %.3f' %(l, c, r) )
    return l,c,r

# BEGIN CALLBACKS
def callback_lidar(lidar_msg):
    # print("received lidar message")
    left_min_r, center_min_r, right_min_r = look_ahead(lidar_msg)


# def callback_dist(dist_msg):
#    pass

# END CALLBACKS
    
# BEGIN SUBSCRIBERS
sub_lidar = rospy.Subscriber('/scan', LaserScan, callback_lidar)
# sub_dist =  rospy.Subscriber('/distance_sensor/distance', Range, callback_dist)   # if we also want to use TOF sensor in front
# END 

rospy.spin()