# test_mycobot_320_slider.py
#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""
import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
from pymycobot.mycobot import MyCobot
from std_msgs.msg import MultiArrayLayout

moveing_completion_pub = rospy.Publisher("moving_completed", Int32, queue_size=10)
completion_pub = rospy.Publisher("gripper_close_action_completed", Int32, queue_size=10)
completion_pub1 = rospy.Publisher("gripper_open_action_completed", Int32, queue_size=10)
completion_pub2 = rospy.Publisher("link6_action_completed", Int32, queue_size=10)
completion_pub3 = rospy.Publisher("link6_callibration_completed", Int32, queue_size=10)
getangle_pub = rospy.Publisher("get_angle_completed", Float32MultiArray, queue_size=10)
getcoord_pub = rospy.Publisher("move_coord_completed", Int32, queue_size=10)


data_list = []
state = True
mc = None

def getangle(data):
    global angles
    rospy.loginfo("get angles..")
    angles = mc.get_angles()
    print(angles)
    time.sleep(1)
    rospy.loginfo("get angles completed")
   
    angle_data = Float32MultiArray(layout=MultiArrayLayout(), data=angles)
    getangle_pub.publish(angle_data)

def movecoord(data):
    global coords
    z = data.data
    move_z = z / 0.1

    rospy.loginfo("get coords..")
    coords = mc.get_coords()
    print(coords)
    time.sleep(1)

    mc.send_coords([int(coords[0]), int(coords[1]), coords[2] - (0.652 * move_z), int(coords[3]), int(coords[4]), int(coords[5])], 25, 0)

    time.sleep(1)

    rospy.loginfo("send coords completed")
        
    getcoord_pub.publish(1)

def callback(data):
    global state, tolerance_list
   
    if state==True:
        
        data_list = []
        for index, value in enumerate(data.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
        mc.send_angles(data_list, 30)
        
        # Get actual angles each time data is received
        actual_list = mc.get_angles()
        rospy.loginfo(actual_list)
        
               

def callback1_state(data):
    global state
    state = False
    
def callback2_state(data):
    global state
    state = True

def gripper_close(data):
    # Check if x is within the valid range
    if data.data < 0 or data.data > 100:
        rospy.logerr("Invalid value for gripper close. Value must be between 0 and 100.")
        return
    rospy.loginfo("gripper close start")
    mc.set_gripper_value(data.data, 20)  # Set the gripper value to x
    time.sleep(1)
    rospy.loginfo("close completed")
    completion_pub.publish(Int32(1))

def gripper_open(data):
    rospy.loginfo("gripper open start")
    mc.set_gripper_value(80,20)
    time.sleep(1)
    rospy.loginfo("open completed")
    completion_pub1.publish(Int32(1))

def listener():
    global mc
   
    rospy.init_node("project3_slider_node", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.Subscriber("state_check", Int32 , callback1_state)
    rospy.Subscriber("state_check2", Int32 , callback2_state)
    rospy.Subscriber("gripper_close", Int32 , gripper_close)
    rospy.Subscriber("gripper_open", Int32 , gripper_open)
    rospy.Subscriber("get_angles", Int32 , getangle)
    rospy.Subscriber("move_coords", Float32 , movecoord)
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    mc.set_gripper_mode(0)
    time.sleep(1)
    mc.init_eletric_gripper()
    time.sleep(1)
    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
