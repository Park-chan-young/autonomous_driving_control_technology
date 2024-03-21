#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

# Motor topic message / LiDAR Distance Info Data
motor_msg = xycar_motor()
distance = []

# when lidar topic come, callback function execute
def callback(data):
    global distance, motor_msg
    distance = data.ranges

# car move forward function
def drive_go():
    global motor_msg
    motor_msg.speed = 5
    motor_msg.angle = 0
    pub.publish(motor_msg)

# car stop function
def drive_stop():
    global motor_msg
    motor_msg.speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)

# initialize node named 'lidar_driver' / create subscriber, publisher
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

# ready to connect lidar
time.sleep(3)

# if the number of the obstacle within 30cm of the vehicle is 3, stop.
while not rospy.is_shutdown():
    ok = 0
    for degree in range(60, 120):
        if distance[degree] <= 0.3:
            ok += 1
        if ok > 3:
            drive_stop()
            break
    if ok <= 3:
        drive_go()