#! /usr/bin/env python

# program to wander space randomly using lidar (and maybe distance sensor) to prevent collisions


import rospy 
import sys
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

PI = 3.14159        # good enough for this program
CIRCLE_RADIANS = 2 * PI
FWD_SPEED = 0.125   # slower to allow lidar to complete more sweeps
SPIN_SPEED = 0.5    # 0.5 radians/sec - ~12.57 sec to spin around completely
SPIN_ANGLE = 20.0     # amount we want to spin
SPIN_TIME = ((SPIN_ANGLE/360.0) * CIRCLE_RADIANS) / SPIN_SPEED
THRESHOLD = 0.25

rospy.init_node("wander")
# rospy.loginfo("PI: " + str(PI))
# rospy.loginfo("Spin speed: " + str(SPIN_SPEED))
# rospy.loginfo("Spin time:" + str(SPIN_TIME))

move = Twist()

def min_range(ranges):
    min_r = 2.0
    for r in ranges:
        if r != 0:  # exclude 0 since we know there are a lot of zero values in data
            if r < min_r:
                min_r = r
    return min_r

def look_ahead(lidar_msg):
    # look at 90 degrees ahead (40 - 10 - 40 degree slices) by taking appropriate slices of message data
    # lidar reports ranges in 0.5 degree increments; 0 is straight back, 360 straight ahead
    l = min_range(lidar_msg.ranges[370:450])
    c = min_range(lidar_msg.ranges[350:370])
    r = min_range(lidar_msg.ranges[270:350])
    rospy.loginfo('min range L: %.3f  min range C: %.3f min range R: %.3f' %(l, c, r) )
    return l,c,r

# BEGIN CALLBACKS
def callback_lidar(lidar_msg):
    # print("received lidar message")
    left_min_r, center_min_r, right_min_r = look_ahead(lidar_msg)

    if center_min_r > THRESHOLD:    # no obstacles - keep going
        print('ahead clear')
        move.linear.x = FWD_SPEED
        move.angular.z = 0
        pub.publish(move)
    elif (left_min_r > THRESHOLD) or (right_min_r > THRESHOLD): 
        if left_min_r > right_min_r:
            rospy.loginfo('ahead blocked; left clearer than right - turn left')
            move.linear.x = 0
            move.angular.z = SPIN_SPEED # counter-clockwise
            pub.publish(move)
            rospy.sleep(SPIN_TIME)
            move.linear.x = 0
            move.angular.z = 0  
            pub.publish(move)   # stop turn before continuing
            rospy.loginfo('taking a look before proceeding')
            rospy.sleep(1)
            rospy.loginfo('Turned L; taking a look before proceeding')
        else:
            rospy.loginfo('ahead blocked; right clearer than left - turn right')
            move.linear.x = 0
            move.angular.z = SPIN_SPEED * -1.0  # clockwise
            pub.publish(move)
            rospy.sleep(SPIN_TIME)   
            move.linear.x = 0
            move.angular.z = 0  
            pub.publish(move)   # stop turn before continuing
            rospy.loginfo('Turned R; taking a look before proceeding')
            rospy.sleep(1)
    else:
        rospy.loginfo('ahead, L, and R blocked; back up and spin a bit')
        move.linear.x = FWD_SPEED * -1.0
        move.angular.z = 0.0
        pub.publish(move)   # back up
        rospy.sleep(1)      # 2 seconds
        move.linear.x = 0
        move.angular.z = 0  
        pub.publish(move)   # stop 
        move.angular.z = SPIN_SPEED
        pub.publish(move)
        rospy.sleep(1.5 * SPIN_TIME)   # spin left a bit more than usual
        rospy.loginfo('backed up and spun a bit')
        rospy.sleep(1)

# def callback_dist(dist_msg):
#    pass

# END CALLBACKS
    
# BEGIN SUBSCRIBERS
sub_lidar = rospy.Subscriber('/scan', LaserScan, callback_lidar, queue_size=1)
# sub_dist =  rospy.Subscriber('/distance_sensor/distance', Range, callback_dist)   # if we also want to use TOF sensor in front
# END 

# BEGIN PUBLISHERS
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# END

rospy.spin()