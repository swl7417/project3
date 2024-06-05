#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32
import serial
import time

flag = False

def flag_callback(msg):
    global flag
    if msg.data == 1 and not flag:
        print("Sending start signal to arduino")
        flag = True
        read_and_publish()  # Start the server request process
        

def read_and_publish():
    global flag
    distance_str = ""
    if flag:
        if ser.in_waiting > 0:
            try:
                distance_str = ser.readline().decode('utf-8').strip()
                
                try:
                    distance = float(distance_str)
                    if distance < 150:
                        real_distance = distance * 2
                    elif distance < 260:
                        real_distance = 260
                    else:
                        real_distance = distance + 15
                    rospy.loginfo(f"Publishing distance: {real_distance}")
                    z = real_distance / 20
                    if z > 13.8:
                        z = 13.8
                    z_pub.publish(z)
                
                    rospy.loginfo("Received z data: %f", z)
                    completion_pub.publish(Int32(1))
                    flag = False
                    print(flag)
                    distance_str = ""
                    distance = 0
                    z = 0
                    # keyboard_input()
                except ValueError:
                    rospy.logwarn("Received invalid data: %s", distance_str)
            except ValueError:
                rospy.logwarn(f"Received invalid data: {distance_str}")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('z_publisher_node', anonymous=True)
    # Create a publisher object
    z_pub = rospy.Publisher('z_input', Float32, queue_size=10)
    completion_pub = rospy.Publisher("z_call_done", Int32, queue_size=10)
    # Setup the serial connection (adjust '/dev/ttyUSB0' to your serial port)
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    rospy.Subscriber("get_z", Int32, flag_callback)

    rospy.spin()
