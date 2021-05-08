#! /usr/bin/env python

# log the voltage for GoPiGo3 every minute

import rospy
from std_msgs.msg import Float64

# BEGIN CALLBACK
def callback(msg):
    # rospy.loginfo_throttle(60, 'V:s', msg.data)
    print(msg.data)
# END CALLBACK


rospy.init_node('gpg_voltage')

# BEGIN SUBSCRIBER
sub = rospy.Subscriber('battery_voltage', Float64, callback)
# END SUBSCRIBER

rospy.spin()
# END ALL