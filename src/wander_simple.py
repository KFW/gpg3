#! /usr/bin/env python

# program to wander space randomly using just distance sensor
# use it to test battery life

import rospy 
# import sys
# import tf don't think we need this
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

rospy.init_node("wander_simple")

FWD_speed = 0.25 # mps
REV_speed = -0.2 # mps - negative for reverse
ROT_speed = 3.14 # radians per second
THRESHOLD = 0.2 # distance from wall in meters

move = Twist()

def callback(msg):
    if msg.range < THRESHOLD:
        move.linear.x = REV_speed
        move.angular.z = ROT_speed
        pub.publish(move)
        rospy.sleep(0.3) # give some time for robot to back up
        move.linear.x = REV_speed * 0.5
        move.angular.z = ROT_speed  * -0.5 # slight turning jog forward to get unstuck
        rospy.sleep(0.1)
    else:
        move.linear.x = FWD_speed
        move.angular.z = 0
        pub.publish(move)

sub = rospy.Subscriber('/distance_sensor/distance', Range, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.spin()