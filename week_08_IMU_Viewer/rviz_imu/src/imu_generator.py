#!usr/bin/env python

# import
import rospy, math, os, rospkg
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler

# rad <-> deg angle transformation
degrees2rad = float(math.pi)/float(180.0)
rad2degrees = float(180.0)/float(math.pi)

# node initialization
rospy.init_node("imu_generator")

# publish '/imu' topic
pub = rospy.Publisher('imu', Imu, queue_size=1)

data = [] 

# open the file located at "path" and read it line by line
path = rospkg.RosPack().get_path('rviz_imu') + "/src/imu_data.txt"
f = file(path, "r")
lines = f.readlines()

# only extract the roll, pitch, yaw numbers and put it into 'data' list
for line in lines:
    tmp = line.split(",")
    extract = []
    for i in tmp:
        extract.append(float(i.split(":")[1]))
    data.append(extract)

# create "Imu" message and add the frame_id as 'map'
imuMsg = Imu()
imuMsg.header.frame_id = 'map'

r = rospy.Rate(10)
seq = 0

# loop time = line number
for j in range(len(data)):

    # convert euler type data to quaternion type data for sending msg to imu
    # imu use quaternion expression to deal with the orietation info
    msg_data = quaternion_from_euler(data[j][0], data[j][1], data[j][2])

    imuMsg.orientation.x = msg_data[0]
    imuMsg.orientation.y = msg_data[1]
    imuMsg.orientation.z = msg_data[2]
    imuMsg.orientation.w = msg_data[3]

    # imu message header / stamp: time info, seq: sequence number
    imuMsg.header.stamp = rospy.Time.now()
    imuMsg.header.seq = seq
    seq = seq + 1

    # publish imu topic at 10times/1sec
    pub.publish(imuMsg)
    r.sleep()
