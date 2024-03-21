#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

# callback function: print the message data to verify sensor performance
def callback(msg):
    print(msg.data)

# initialize the node
rospy.init_node('ultra4_sub')
# initialize the topic to subscribe msg named 'ultrasonic'
sub = rospy.Subscriber('ultra4', Int32MultiArray, callback)

rospy.spin()