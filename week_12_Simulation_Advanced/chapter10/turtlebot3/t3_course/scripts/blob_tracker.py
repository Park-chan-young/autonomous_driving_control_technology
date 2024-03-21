#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blob, Blobs

# global
turn_speed = 0.0
blob_position = 0

def blobs_callback(msg):
    global turn_speed
    global blob_position

    if len(msg.blobs):
        for obj in msg.blobs:
            if obj.name == "RedBall":
                rospy.loginfo("Blob <" + str(obj.name) + "> Detected!")
                blob_position = obj.x
                rospy.loginfo("Blob is at " + str(blob_position))

                if blob_position > 1200:
                    rospy.loginfo("Turn Right!")
                    turn_speed = -1.0

                if blob_position < 800:
                    rospy.loginfo("Turn Left!")
                    turn_speed = 1.0

                if 800 < blob_position < 1200:
                    rospy.loginfo("Centered")
                    turn_speed = 0.0
    else:
        turn_speed = 0.0


def run():
    rospy.init_node("blob_tracker_node")
    global blob_position

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('blobs', Blobs, blobs_callback)

    global turn_speed
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        if turn_speed != 0.0:
            rospy.loginfo("Turning %s"%turn_speed)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = turn_speed
            turn_speed = 0.0
        else:
            rospy.loginfo("Straight %s"%turn_speed)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # send the message and delay
        cmd_vel_pub.publish(cmd_vel)
        blob_position = 0.0
        rospy.sleep(0.1)


if __name__=='__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass