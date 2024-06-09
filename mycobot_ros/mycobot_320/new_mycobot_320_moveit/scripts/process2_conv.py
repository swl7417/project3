#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import serial
import time

flag = False
stop = False

def flag_callback(msg):
    global flag
    if msg.data == 1 and not flag:
        print("Sending start signal to arduino")
        flag = True
        read_and_publish()  # Start the server request process

def stop_callback(msg):
    global flag, stop
    if msg.data == 1 and not flag:
        print("Sending start signal to arduino")
        flag = True
        stop = True
        read_and_publish()  # Start the server request process
        

def read_and_publish():
    global flag, stop
    if flag:
        try:
            if stop:
                ser.write(b'S')
                print('conv Stop!')
                time.sleep(1)
                flag = False
                stop = False
            
            else:
                ser.write(b'1')
                print('conv Start!')
                time.sleep(1)
                flag = False
                
        except ValueError:
            rospy.logwarn(f"Received invalid data")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('conv_publisher_node', anonymous=True)

    # Setup the serial connection (adjust '/dev/ttyUSB0' to your serial port)
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    rospy.Subscriber("start_conv", Int32, flag_callback)
    rospy.Subscriber("stop_conv", Int32, stop_callback)

    rospy.spin()