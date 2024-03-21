#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class MoveRobot(object):
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.last_cmd_vel = Twist()

        self.pub_rate = rospy.Rate(10)

        self.shutdown_detected = False

    def move_robot(self, cmd_vel_msg):
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.0
        self.move_robot(cmd_vel_msg=cmd_vel)
        self.shutdown_detected = True

def main():
    rospy.init_node('move_robot_node', anonymous=True)

    move_robot_obj = MoveRobot()
    cmd_vel = Twist()

    # Make it start turning
    cmd_vel.angular.z = 0.15

    rate = rospy.Rate(5)

    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        move_robot_obj.stop_robot()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        move_robot_obj.move_robot(cmd_vel_msg=cmd_vel)
        rate.sleep()

if __name__=='__main__':
    main()