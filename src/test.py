#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('test')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

move = Twist()

while not rospy.is_shutdown():
    print('Es geht')
    move.linear.x = 0.25
    move.angular.z = 0
    pub.publish(move)
    rate.sleep()