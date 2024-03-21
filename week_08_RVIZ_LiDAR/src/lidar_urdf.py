#!/usr/bin/env python

import serial, time, rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Header

lidar_points = None

# when lidar topic arrive, distance data is filled in lidar_points
def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges

# initialize the topic & subscriber
rospy.init_node('lidar')
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

# ready to publish 4 topics
pub1 = rospy.Publisher('scan1', Range, queue_size=1)
pub2 = rospy.Publisher('scan2', Range, queue_size=1)
pub3 = rospy.Publisher('scan3', Range, queue_size=1)
pub4 = rospy.Publisher('scan4', Range, queue_size=1)

msg = Range()
h = Header()

# fill the "Range" message
msg.radiation_type = Range().ULTRASOUND
msg.min_range = 0.02
msg.max_range = 2.0
msg.field_fo_view = (30/180) * 3.14


while not rospy.is_shutdown():
    # publish the message topic only when the lidar_points are detected
    if lidar_points == None:
        continue
    
    # publish the topic named 'scan1' when lidar_points are detected at forward
    h.frame_id = "front"
    msg.header = h
    msg.range = lidar_points[90]
    pub1.publish(msg)

    # publish the topic named 'scan2' when lidar_points are detected at rightside
    h.frame_id = "right"
    msg.header = h
    msg.range = lidar_points[180]
    pub2.publish(msg)

    # publish the topic named 'scan3' when lidar_points are detected at backward
    h.frame_id = "back"
    msg.header = h
    msg.range = lidar_points[270]
    pub3.publish(msg)

    # publish the topic named 'scan4' when lidar_points are detected at leftside
    h.frame_id = "left"
    msg.header = h
    msg.range = lidar_points[0]
    pub4.publish(msg)

    # publish one topic per 0.5 sec = publish 2 topic in 1second
    time.sleep(0.5)