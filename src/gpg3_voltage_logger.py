#! /usr/bin/env python

# log the voltage for GoPiGo3 every 5 seconds

import rospy
from std_msgs.msg import Float64

rospy.init_node('gpg_voltage')

init_time = rospy.get_time()
checkpoint_elapsed_time = 0.0
last_minutes = -1 # can't be 0 because want it to print first time through

file_time = str(int(init_time)) # integer string for filename
filename = '/home/pi/test/voltage_' + file_time + '.csv'

# create file
with open(filename, 'w+') as f:
    f.write('time, voltage \n')

def print_for_monitoring(elapsed_time, v): # elapsed time is float seconds
    global last_minutes
    et_rounded_down = int(elapsed_time) # will round time down to integer seconds
    hours = et_rounded_down // 3600 
    minutes = (et_rounded_down % 3600) // 60
    if minutes != last_minutes: # only print when change in elapsed time
        last_minutes = minutes
        print('Elapsed time %i hours, %i minutes' % (hours, minutes))
        print('Battery level: %.1f V' % v)
        print('-') # just for spacing
    
# BEGIN CALLBACK
def callback(msg):
    global checkpoint_elapsed_time
    elapsed_time = rospy.get_time() - init_time
    if (elapsed_time - checkpoint_elapsed_time) > 36
                                # will trigger about every 36 seconds or 100 times/hr
        checkpoint_elapsed_time = elapsed_time # reset timer
        log_string = '%.2f, %.1f \n' % (elapsed_time, msg.data)
        with open(filename, 'a') as f:
            f.write(log_string)
        f.close()
        # if not monitoring the terminal, can comment out line below
        print_for_monitoring(elapsed_time, msg.data)
# END CALLBACK

# BEGIN SUBSCRIBER
sub = rospy.Subscriber('battery_voltage', Float64, callback)
# END SUBSCRIBER

rospy.spin()
# END ALL