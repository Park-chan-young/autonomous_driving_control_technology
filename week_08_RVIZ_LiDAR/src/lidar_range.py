#!/usr/bin/env python

import serial, time, rospy
from sensor_msgs.msg import lidar_range
from std_msgs.msg import Header

# initialize node named 'lidar_range'
rospy.init_node('lidar_range')

# ready to publish 4 topics
pub1 = rospy.Publisher('scan1', Range, queue_size=1)
pub2 = rospy.Publisher('scan2', Range, queue_size=1)
pub3 = rospy.Publisher('scan3', Range, queue_size=1)
pub4 = rospy.Publisher('scan4', Range, queue_size=1)

# fill the Range, header information
msg = Range()
h = Header()
h.frame_id = "sensorXY"
msg.header = h
msg.radiation_type = Range().ULTRASOUND
msg.min_range = 0.02
msg.max_range = 2.0
msg.field_of_view = (30.0/180.0) * 3.14

while not rospy.is_shutdown():
    # current time = header.stamp
    msg.header.stamp = rospy.Time.now()
    # publish the topic named 'scan1' when the distance between obstacle and sensor is 0.4m
    msg.range = 0.4
    pub1.publish(msg)
    # publish the topic named 'scan2' when the distance between obstacle and sensor is 0.8m
    msg.range = 0.8
    pub2.publish(msg)
    # publish the topic named 'scan3' when the distance between obstacle and sensor is 1.2m
    msg.range = 1.2
    pub3.publish(msg)
    # publish the topic named 'scan4' when the distance between obstacle and sensor is 1.6m
    msg.range = 1.6
    pub4.publish(msg)

    # publish one topic per 0.2 sec = publish 5 topic in 1second
    time.sleep(0.2)