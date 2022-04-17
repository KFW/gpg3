#! /usr/bin/env python

# laser_logger.py
# create csv log file of lidar readings - check for distance and whether lidar is square to robot
# collect 100 readings then stop

import csv
import rospy
from sensor_msgs.msg import LaserScan

log = []
count = 0

def callback(msg):
    global log
    global count
    log.append(msg.ranges)
    count += 1

rospy.init_node('laser_logger')
sub = rospy.Subscriber('/scan', LaserScan, callback)

while count < 250:
    print(count)

with open('/home/pi/laser_log.csv', 'w') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerows(log)

print("Log written - we're done here")