#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg
import time

# 노드 초기화
rospy.init_node('move_to_pose_node', anonymous=True)
state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
state_check2_pub = rospy.Publisher("state_check2", Int32, queue_size=10)
gripper_pub = rospy.Publisher("gripper_check", Int32, queue_size=10)
gripper_close_pub = rospy.Publisher("gripper_close", Int32, queue_size=10)
gripper_open_pub = rospy.Publisher("gripper_open", Int32, queue_size=10)
link6_set_pub = rospy.Publisher("link6_set", Int32, queue_size=10)
link6_cal_pub = rospy.Publisher("link6_callibration_set", Int32, queue_size=10)
busy = False  # 동작 중인지 여부를 나타내는 플래그

# MoveIt! 초기화
moveit_commander.roscpp_initialize(sys.argv)

# 로봇 모델 로딩
robot = moveit_commander.RobotCommander()

# 로봇 팔 그룹 초기화
arm_group = moveit_commander.MoveGroupCommander("arm_group")


def move_to_pose(x, y, z, link6_angle, gripper):
    print("state = True")
    state_check2_pub.publish(1) # state = True
    time.sleep(3)
    
    global busy
    busy = True  # 동작 시작

    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    # 목표 포즈 생성
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.x = 0.00229375
    pose_goal.orientation.y = 0.999996
    pose_goal.orientation.z = -0.00172712
    pose_goal.orientation.w = -0.000785574
    
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    
    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)
        
        state_check_pub.publish(1) # state_check topic 1 publish state = False
        print("state = False")
        time.sleep(2)
        
        print("link6 move start")
        link6_set_pub.publish(Int32(link6_angle))
        rospy.wait_for_message("link6_action_completed", Int32)
        print("link6 move done")
        time.sleep(3)


        print("gripper close start")
        gripper_close_pub.publish(Int32(gripper))
        rospy.wait_for_message("gripper_close_action_completed", Int32)
        print("gripper close done")
        time.sleep(2)

   
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    time.sleep(3)

    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(3)

    pose_goal.position.x = 0
    pose_goal.position.y = -0.31868
    pose_goal.position.z = 0.237568
    pose_goal.orientation.x = 0.000161176
    pose_goal.orientation.y = 0.9999996
    pose_goal.orientation.z = 0.00279677
    pose_goal.orientation.w = 0.0000014813

    arm_group.set_pose_target(pose_goal)
    plan1 = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기


    # 동작이 완료되었는지 확인
    if plan1:
        
        print("Movement successful!")
        time.sleep(5)
        
        state_check_pub.publish(1) # state_check topic 1 publish state = False
        print("state = False")
        time.sleep(2)

        print("gripper open start")
        gripper_open_pub.publish(1)
        rospy.wait_for_message("gripper_open_action_completed", Int32)
        print("gripper open done")
        time.sleep(2)
  
    
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()


    busy = False  # 동작 종료


def move_to_startpose():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(3)
    
    global busy
    busy = True  # 동작 시작

    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    # 목표 포즈 생성
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = -0.219721
    pose_goal.position.y = -0.00227073
    pose_goal.position.z = 0.24565
    pose_goal.orientation.x = 0.00229375
    pose_goal.orientation.y = 0.999996
    pose_goal.orientation.z = -0.00172712
    pose_goal.orientation.w = -0.000785574


    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    
    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)
        
        state_check_pub.publish(1)
        print("state = False")
        time.sleep(2)

        # print("link6 cal start")
        # link6_cal_pub.publish(1)
        # rospy.wait_for_message("link6_callibration_completed", Int32)
        # print("link6 cal done")
        # time.sleep(3)
        
        print("gripper open start")
        gripper_open_pub.publish(1)
        rospy.wait_for_message("gripper_open_action_completed", Int32)
        print("gripper open done")
        time.sleep(2)
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    busy = False  # 동작 종료


def move_to_middle(): # barcode reading method
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(3)   
   
    global busy
    busy = True  # 동작 시작
    
    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    # 목표 포즈 생성
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = -0.27011
    pose_goal.position.y = -0.0398727
    pose_goal.position.z = 0.237818
    pose_goal.orientation.x = -0.00427998
    pose_goal.orientation.y = 0.999702
    pose_goal.orientation.z = 0.0181952
    pose_goal.orientation.w = 0.0157242

    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    
    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)

        # state_check_pub.publish(1) # state_check topic 1 publish state = False
        # print("state = False")
        # time.sleep(2)
        
        # print("gripper move start")
        # gripper_pub.publish(1)
        # rospy.wait_for_message("gripper_close_action_completed", Int32)
        # print("gripper move done")
        # time.sleep(2)
        
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()


    time.sleep(3)

    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(3)

    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)

    pose_goal.position.x = -0.27011
    pose_goal.position.y = -0.0398727
    pose_goal.position.z = 0.237818
    pose_goal.orientation.x = -0.003152
    pose_goal.orientation.y = 0.999702
    pose_goal.orientation.z = 0.0181952
    pose_goal.orientation.w = 0.0157242

    arm_group.set_pose_target(pose_goal)
    plan1 = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기


    # 동작이 완료되었는지 확인
    if plan1:
        
        print("Movement successful!")
        time.sleep(5)
        
        # state_check_pub.publish(1) # state_check topic 1 publish state = False
        # print("state = False")
        # time.sleep(2)

        # print("gripper open start")
        # gripper_open_pub.publish(1)
        # rospy.wait_for_message("gripper_open_action_completed", Int32)
        # print("gripper open done")
        # time.sleep(2)
  
    
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    busy = False  # 동작 종료
    

def keyboard_callback(data):
    global busy, link6_angle, gripper
    x = 0.0
    y = 0.0
    z = 0.0
    link6_angle = 0.0
    gripper = 0.0

    if not busy:  # Check if not busy
        if len(data.data) == 5:  # Check if all variables are present
            try:
                # Extract x, y, z, link6_angle, and gripper from data
                x, y, z, link6_angle, gripper = map(float, data.data)

                print("Moving to pose!")
                move_to_pose(x, y, z, int(link6_angle), int(gripper))
                time.sleep(1)

                # put barcode reading code

                print("Moving to startpose!")
                move_to_startpose()
                time.sleep(1)

                print("Finish!")
            except ValueError:
                print("Invalid data received, all elements must be numbers.")
        else:
            print("Incomplete data received, all variables must be present.")

if __name__ == '__main__':
    # rospy.Subscriber("step_input", Int32, keyboard_callback)
    print("move to startpose!")
    # move_to_middle()
    move_to_startpose()
    rospy.Subscriber("step_input", Float32MultiArray, keyboard_callback)
    rospy.spin()