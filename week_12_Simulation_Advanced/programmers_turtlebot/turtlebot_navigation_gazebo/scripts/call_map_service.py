#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap, GetMapRequest
import sys

rospy.init_node('map_service_client')
rospy.wait_for_service("static_map")

rospy.loginfo("init complete")

get_map_service = rospy.ServiceProxy("static_map", GetMap)
get_map = GetMapRequest()

result=get_map_service(get_map)

print(result)