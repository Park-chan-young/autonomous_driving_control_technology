#!/usr/bin/env python3

import threading
import rospy
import actionlib

from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Empty

waypoints = []

def convert_PoseWithCovariancStamped_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id = "map"
    poses.poses = [pose.pose.pose for pose in waypoints]

    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])

        self.pose_array_publisher = rospy.Publisher('waypoints', PoseArray, queue_size=1)
        self.custom_waypoint_topic = rospy.get_param('~custom_waypoints_topic', 'my_waypoints_list')

        def wait_for_path_reset():
            """thread workier function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('path_reset', Empty)
                rospy.looginfo("Recieved path RESET message")
                self.initialize_path_queue()
                rospy.sleep(3) # wait 3(s) for wait_for_message()

        reset_thread = threading.Thread(target = wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # waypoint queue

        self.pose_array_publisher.publish(convert_PoseWithCovariancStamped_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        def wait_for_path_ready(): # for thread
            """thread worker function"""
            data = rospy.wait_for_message('path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True

        ready_thread = threading.Thread(target = wait_for_path_ready)
        ready_thread.start()

        waypoints_topic = self.custom_waypoint_topic
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % waypoints_topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                pose = rospy.wait_for_message(waypoints_topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in str(e):
                    continue
                else:
                    raise e
                
            rospy.loginfo("Recieved new point")
            waypoints.append(pose)

            self.pose_array_publisher.publish(convert_PoseWithCovariancStamped_to_PoseArray(waypoints))

        return 'success'

##############################################################################

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connection to move_base ...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        global waypoints

        for waypoint in waypoints:
            if waypoints == []:
                rospy.loginfo("The waypoint queue has been reset.")
                break

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation

            rospy.loginfo("Executing move_base goal to position (x, y): %s, %s" %
                          (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic put -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal = goal)
            self.client.wait_for_result()
        return 'success'
    
##############################################################################

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo("###############################")
        rospy.loginfo("##### REACHED FINISH GATE #####")
        rospy.loginfo("###############################")
        return 'success'
    
##############################################################################

def main():
    rospy.init_node('custom_follow_waypoints')
    
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                         transitions={'success':'FOLLOW_PATH'},
                         remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                         transitions={'success':'FOLLOW_COMPLETE'},
                         remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_COMPLETE', PathComplete(),
                         transitions={'success':'GET_PATH'})
        
    outcome = sm.execute()

if __name__=='__main__':
    main()