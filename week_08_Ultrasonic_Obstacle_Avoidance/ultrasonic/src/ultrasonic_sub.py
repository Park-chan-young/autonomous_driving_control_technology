#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

# callback function: print the message data to verify sensor performance
def callback(msg):
    print(msg.data)

# initialize the node
rospy.init_node('ultrasonic_sub')
# initialize the topic to subscribe msg named 'ultrasonic'
sub = rospy.Subscriber('ultrasonic', Int32, callback)

rospy.spin()