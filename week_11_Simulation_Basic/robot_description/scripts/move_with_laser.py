#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    rospy.loginfo('Minimum distance is: %s' % min(msg.ranges))

def main():
    laser_sub = rospy.Subscriber('laser/scan', LaserScan, callback=laser_callback)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.init_node('move_with_laser_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        twist_msg.angular.z = 0.3
        twist_pub.publish(twist_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass