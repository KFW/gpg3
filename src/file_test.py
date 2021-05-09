#! /usr/bin/env python

import rospy

rospy.init_node('file_test')

time = rospy.get_rostime().secs 

filename = '/home/pi/test/voltage_' + str(time) + '.log'

# create file
with open(filename, 'w+') as f:
    f.write('created \n')

# reopen file
with open(filename, 'a') as f:
    f.write('stuff added')
f.close()
