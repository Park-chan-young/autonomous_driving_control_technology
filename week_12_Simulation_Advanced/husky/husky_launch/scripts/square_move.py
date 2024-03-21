#! /usr/bin/env python3

import rospy
import time
import math

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyRequest

class MoveHusky():

    def __init__(self):

        # init publisher
        self.huskey_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        # init subscriber
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.amcl_pose_msg = PoseWithCovarianceStamped

        # initialize service client
        rospy.wait_for_service('global_localization')
        self.disperse_particles_service = rospy.ServiceProxy('global_localization', Empty)
        self.srv_request = EmptyRequest()

        # other stuff
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.stop_husky()
        self.ctrl_c = True

    def stop_husky(self):
        rospy.loginfo("Shutdown time! Stop the robot")
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        i = 0

        while i < 20:
            self.huskey_vel_publisher.publish(self.cmd_vel_msg)
            self.rate.sleep()
            i += 1

    def move_forward(self, linear_speed=0.5, angular_speed=0.0):
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = angular_speed
        i = 0

        while i < 50:
            self.huskey_vel_publisher.publish(self.cmd_vel_msg)
            self.rate.sleep()
            i += 1

    def turn(self, linear_speed=0.0, angular_speed=0.8):
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = angular_speed
        i = 0

        while i < 25:
            self.huskey_vel_publisher.publish(self.cmd_vel_msg)
            self.rate.sleep()
            i += 1

    def move_square(self):
        i = 0

        while not self.ctrl_c and i < 4: # square
            # move forwards
            rospy.loginfo("####### Going Forwards...")
            self.move_forward()
            self.stop_husky()

            # turn
            rospy.loginfo("####### Turning...")
            self.turn()
            self.stop_husky()

            i += 1

        self.stop_husky()
        rospy.loginfo("####### Finished Moving in a Square...")

    def call_service(self):
        rospy.loginfo("####### Calling Service...")
        result = self.disperse_particles_service(self.srv_request)

    def amcl_pose_callback(self, msg):
        self.amcl_pose_msg = msg

    def calculate_covariance(self):
        rospy.loginfo("####### Calculating Covariance...")
        # covariance[] -> size 36 -> 6x6 matrix
        cov_x = self.amcl_pose_msg.pose.covariance[0]
        cov_y = self.amcl_pose_msg.pose.covariance[7]
        cov_z = self.amcl_pose_msg.pose.covariance[35]
        rospy.loginfo("\n## Cov X: " + str(cov_x) + "\n## Cov Y: " + str(cov_y) + "\n## Cov Z: " + str(cov_z) )
        cov = (cov_x + cov_y + cov_z) / 3

        return cov


if __name__=='__main__':
    rospy.init_node('move_husky_node', anonymous=True)
    move_husky = MoveHusky()

    cov = 1 # init covariance

    while cov > 0.65:
        move_husky.call_service() # disperse particles
        move_husky.move_square() # move square

        # check covariance
        cov = move_husky.calculate_covariance() # calculate covariance
        rospy.loginfo("####### Total Covariance: " + str(cov))
        if cov > 0.65:
            rospy.logwarn("####### Total Covariance is greater than 0.65. Repeating the process...")
        else:
            rospy.loginfo("####### Total Covariance is lower than 0.65. Robot correctly localized...")
            rospy.loginfo("####### Exiting...")
            break