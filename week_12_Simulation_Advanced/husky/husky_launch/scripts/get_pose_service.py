#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse # Import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

robot_pose = Pose()

def service_callback(request):
    rospy.loginfo("Robot Pose: " + str(robot_pose))

    return EmptyResponse() # the service Response class, in this case EmptyResponse


def sub_callback(msg):
    global robot_pose
    robot_pose = msg.pose.pose

rospy.init_node('get_pose_service')
my_service = rospy.Service('get_pose_service', Empty, service_callback) # create the Service called get_pose_service with the defined callback
sub_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, sub_callback)

rospy.spin() # mantain the service open.