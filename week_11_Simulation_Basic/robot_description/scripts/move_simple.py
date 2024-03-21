#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def odom_callback(msg):
    rospy.loginfo('X: %s / Y: %s' % (round(msg.pose.pose.position.x, 2), round(msg.pose.pose.position.y,2)))

def main():
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.init_node('move_simple_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.3

        twist_pub.publish(vel_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass