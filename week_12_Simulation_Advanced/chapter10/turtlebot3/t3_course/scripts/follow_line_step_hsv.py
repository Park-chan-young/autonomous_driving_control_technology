#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from move_robot import MoveRobot

class LineFollower(object):
    def __init__(self):
        self.bridge_obj = CvBridge()
        self.img_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.img_callback)
        self.move_robot_obj = MoveRobot()

    def img_callback(self, msg):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_img = self.bridge_obj.imgmsg_to_cv2(img_msg=msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(str(e))

        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_img.shape
        descentre = 160
        rows_to_watch = 100
        crop_img = cv_img[(height)//2 + descentre:(height)//2 + (descentre + rows_to_watch)][1:width]

        # Convert from BGR to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height//2, width//2

        # Bitwise-AND mask and original image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Draw the centroid in the result image
        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        cv2.imshow("Original", cv_img)
        cv2.imshow("HSV", hsv)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", res)

        cv2.waitKey(1)

        error_x = cx - width//2
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.15
        cmd_vel.angular.z = - error_x / 5000

        rospy.loginfo("angular value is " + str(cmd_vel.angular.z))

        # Make it start turning
        self.move_robot_obj.move_robot(cmd_vel_msg=cmd_vel)

    def clean_up(self):
        self.move_robot_obj.stop_robot()
        cv2.destroyAllWindows()

def main():
    rospy.init_node("line_follower_node", anonymous=True)

    line_follower = LineFollower()

    rate = rospy.Rate(5)

    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__=='__main__':
    main()