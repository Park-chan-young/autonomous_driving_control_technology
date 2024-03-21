#!/usr/bin/env python

import serial, time, rospy
from std_msgs.msg import Int32

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
    # conver the type of data (string to integer)
    ultrasonic_data = int(filter(str.isdigit, serial_data))
    msg.data = ultrasonic_data

# main
if __name__ == '__main__':
    # initialize the node
    rospy.init_node('ultrasonic_pub', anonymous=False)
    # initialize the topic to publish msg named 'ultrasonic'
    pub = rospy.Publisher('ultrasonic', Int32, queue_size=1)

    # msg type
    msg = Int32()
    # main loop
    while not rospy.is_shutdown():
        read_sensor()
        pub.publish(msg)
        time.sleep(0.2)

    ser_front.close()