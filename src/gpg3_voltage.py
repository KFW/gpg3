#! /usr/bin/env python

# log the voltage for GoPiGo3 every 5 seconds

import rospy
from std_msgs.msg import Float64

rospy.init_node('gpg_voltage')

init_time = rospy.get_time()
last_elapsed_time = 0.0


# BEGIN CALLBACK
def callback(msg):
    global last_elapsed_time
    elapsed_time = rospy.get_time() - init_time
    if (elapsed_time - last_elapsed_time) > 36.0: 
        # will trigger about every 36 seconds or 100 times/hr
        last_elapsed_time = elapsed_time # reset timer
        log_string = '%.2f, %.1f' % (elapsed_time, msg.data)
        print(log_string)
# END CALLBACK

# BEGIN SUBSCRIBER
sub = rospy.Subscriber('battery_voltage', Float64, callback)
# END SUBSCRIBER

rospy.spin()
# END ALL