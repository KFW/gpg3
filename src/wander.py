#! /usr/bin/env python

# program to wander space randomly using lidar (and maybe distance sensor) to prevent collisions


import rospy 
import sys
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

FWD_SPEED = 0.5     # slower to allow lidar to complete more sweeps
REV_SPEED = 0.25
THRESHOLD = 1.0

rospy.init_node("wander")

move = Twist()

def min_range(ranges):
    min_r = 2.0
    for r in ranges:
        if r != 0:  # exclude 0 since we know there are a lot of zero values in data
            if r < min_r:
                min_r = r
    return min_r

def look_ahead(lidar_msg):
    # look at 120 degrees ahead (30-60-30) by taking appropriate slices of message data
    l = min_range(lidar_msg.ranges[240:300])
    c = min_range(lidar_msg.ranges[300:420])
    r = min_range(lidar_msg.ranges[420:480])
    sys.loginfo('min range L: %.3f  min range C: %.3f min range R: %.3f' %(l, c, r) )
    return l,c,r

def callback_lidar(lidar_msg):
    left_min_r, center_min_r, right_min_r = look_ahead(lidar_msg)
    if center_min_r > THRESHOLD:    # no obstacles - keep going
        sys.loginfo('ahead clear')
        move.linear.x = FWD_SPEED
        move.angular.z = 0
        pub.publish(move)
    elif (left_min_r > THRESHOLD) or (right_min_r > THRESHOLD): 
        if left_min_r > right_min_r:
            # turn ~60 degrees left
        else:
            # turn ~60 degrees right
    else:
        # spin not quite 180 and continue wandering



# def callback_dist(dist_msg):
#    pass
    

sub_lidar = rospy.Subscriber('/scan', LaserScan, callback_lidar)
# sub_dist =  rospy.Subscriber('/distance_sensor/distance', Range, callback_dist)   # if we also want to use TOF sensor in front
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.spin()