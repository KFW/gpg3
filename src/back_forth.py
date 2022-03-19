#!/usr/bin/env python

# just run the robot back and forth. No turns or nothin'...

import rospy
from geometry_msgs.msg import Twist

def forward():
    move.linear.x = 0.25
    move.angular.z = 0
    pub.publish(move)
    # print('forward')

def reverse():
    move.linear.x = -0.25
    move.angular.z = 0
    pub.publish(move) 
    # print('reverse')

def pause():
    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)   
    # print('pause')

def shutdownhook():
    rospy.loginfo('Shutting down')
    pause()

rospy.init_node('back_forth')
rospy.on_shutdown(shutdownhook)

rospy.sleep(10.0) # delay to get set up

rate = rospy.Rate(50)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
move = Twist()
sequence = [forward, pause, reverse, pause]
duration = [3.0, 0.2, 3.0, 0.2] # use this so each step can have it's own duration

time_mark = rospy.get_time() # initialize
step = 0 # initialize
while not rospy.is_shutdown():
    sequence[step]() # call the step
    now = rospy.get_time()
    if now - time_mark > duration[step]:
        time_mark = now
        step = (step + 1) % 4 # cycle through steps
    rate.sleep()