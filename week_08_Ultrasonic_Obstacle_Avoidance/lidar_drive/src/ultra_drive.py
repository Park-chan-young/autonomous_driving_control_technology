#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

# Ultrasonic Distance Info Data / Motor topic message
ultra_msg = None
motor_msg = xycar_motor()

# when ultra topic come, callback function execute
def callback(data):
    global distance, motor_msg
    ultra_msg = data.data

# car move forward function
def drive_go():
    global motor_msg, pub
    motor_msg.speed = 5
    motor_msg.angle = 0
    pub.publish(motor_msg)

# car stop function
def drive_stop():
    global motor_msg, pub
    motor_msg.speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)

# initialize node named 'ultra_driver' / create subscriber, publisher
rospy.init_node('ultra_driver')
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

# ready to connect ultra
time.sleep(3)

# if the distance between car and obstacle is smaller than 10cm, stop.
# when the distance value is 0, there is no obstacle in front of the vehicle.
while not rospy.is_shutdown():
    
    if