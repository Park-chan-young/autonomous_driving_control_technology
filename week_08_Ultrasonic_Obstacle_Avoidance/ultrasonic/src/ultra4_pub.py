#!/usr/bin/env python

import serial, time, rospy
from std_msgs.msg import Int32MultiArray, Int32

FRONT = [0,0,0,0]

# port connected with arduino, baudrate is 9600
ser_front = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
)

# read sensor data
def read_sensor():
    # serial_data came from serial port named '/dev/ttyUSB0'
    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput()
    FRONT = read_Sdata(sensor_data)
    msg.data = FRONT

def read_Sdata(s):
    s = s.replace(" ", "")
    s_data = s.split("mm")
    s_data.remove('\r\n')
    s_data = list(map(int, s_data))
    return s_data

# main
if __name__ == '__main__':
    # initialize the node
    rospy.init_node('ultra4_pub', anonymous=False)
    # initialize the topic to publish msg named 'ultrasonic'
    pub = rospy.Publisher('ultra4', Int32MultiArray, queue_size=1)

    # msg type
    msg = Int32MultiArray()
    # main loop
    while not rospy.is_shutdown():
        read_sensor()
        pub.publish(msg)
        time.sleep(0.2)

    ser_front.close()