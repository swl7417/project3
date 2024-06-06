# test_movepose.py
#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32, Float32MultiArray, Float64MultiArray, String
import geometry_msgs.msg
import math
import time

# 노드 초기화
rospy.init_node('project3_main_node', anonymous=True)
state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
state_check2_pub = rospy.Publisher("state_check2", Int32, queue_size=10)
gripper_pub = rospy.Publisher("gripper_check", Int32, queue_size=10)
gripper_close_pub = rospy.Publisher("gripper_close", Int32, queue_size=10)
gripper_open_pub = rospy.Publisher("gripper_open", Int32, queue_size=10)
get_angle_call_pub = rospy.Publisher("get_angles", Int32, queue_size=10)
move_coord_call_pub = rospy.Publisher("move_coords", Float32, queue_size=10)
get_z_call_pub = rospy.Publisher("get_z", Int32, queue_size=10)
send_start_pub = rospy.Publisher("start", Int32, queue_size=10)
send_stop_pub = rospy.Publisher("stop", Int32, queue_size=10)
send_value_pub = rospy.Publisher("done_value", String, queue_size=10)
# process2_start_pub = rospy.Publisher("pro2_start", Int32, queue_size=10)
busy = False  # 동작 중인지 여부를 나타내는 플래그
process_start = False
count = 0
z_state = False

# MoveIt! 초기화
moveit_commander.roscpp_initialize(sys.argv)

# 로봇 모델 로딩
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

# 로봇 팔 그룹 초기화
arm_group = moveit_commander.MoveGroupCommander("arm_group")


def start_state(data):
    global process_start
    if not process_start:
        print("procees_start: ", process_start)
        print("move to startpose!")
        move_to_startpose()
        print("send start signal!")
        send_start_pub.publish(1)
        process_start = True
        print("procees_start: ", process_start)


def radian(angle_degrees):
    return angle_degrees * math.pi / 180.0

def z_callback(data):
    global z, z_state
    z = 0.0
    try:
        z = data.data
        print("z value received:", z)
        time.sleep(1)
        z_state = True
        print(z_state)

    except ValueError:
        print("Invalid data received, z value must be a number.")


def data_callback(data):
    global data_input
    data_input = ""
    try:
        data_input = data.data
        print("data: ", data_input)
        time.sleep(1)

    except ValueError:
        print("Invalid data received, data value must be a string.")

def value_callback(data):
    global value_input
    value_input = ""
    try:
        value_input = data.data
        print("value: ", value_input)
        time.sleep(1)

    except ValueError:
        print("Invalid data received, value must be a string.")



def angle_callback(data):
    global j1, j2, j3, j4, j5, j6
    j1 = 0.00
    j2 = 0.00
    j3 = 0.00
    j4 = 0.00
    j5 = 0.00 
    j6 = 0.00
    if len(data.data) == 6:
        try:
            
            j1, j2, j3, j4, j5, j6 = map(float, data.data)

        except ValueError:
                print("Invalid data received, all elements must be numbers.")
        else:
            print("Incomplete data received, all variables must be present.")

def move_to_pose(a1,a2,a3,a4,a5,a6):
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True
    
    # 조인트 각도 설정
    joint_goal = [radian(a1), radian(a2), radian(a3),radian(a4), radian(a5), radian(a6)] 

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)


    # 속도 및 가속도 제한 설정
    arm_group.set_max_velocity_scaling_factor(0.4)  # 속도를 25%로 설정
    arm_group.set_max_acceleration_scaling_factor(0.4)  # 가속도를 25%로 설정

    # Execute the planned motion and wait until it's complete
    plan = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(1)
        
    else:
        print("Movement failed!")
    
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    time.sleep(0.5)

    busy = False



def open_gripper():

    state_check_pub.publish(1) # state = False
    print("state = False")
    time.sleep(0.1)
        
    print("gripper open start")
    gripper_open_pub.publish(1)
    rospy.wait_for_message("gripper_open_action_completed", Int32)
    print("gripper open done")


def move_to_startpose():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    move_to_pose(-32,-39,126,2,-88,-33)
    time.sleep(0.5)
    open_gripper()
    
    busy = False  # 동작 종료



def move_to_failpose():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    move_to_pose(-20,0,101,-12,-88,-21)
    time.sleep(0.5)
    open_gripper()
    
    busy = False  # 동작 종료


def move_to_goalpose(x, y, link6_angle):
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    move_to_pose(-11,-4,83,4,-90,-12)
    time.sleep(0.5)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = 0.289381
    pose_goal.orientation.x = 0.697496
    pose_goal.orientation.y = 0.713852
    pose_goal.orientation.z = -0.036017
    pose_goal.orientation.w = -0.0511595

    arm_group.set_pose_target(pose_goal)
    # 속도 및 가속도 제한 설정
    arm_group.set_max_velocity_scaling_factor(0.4)  # 속도를 25%로 설정
    arm_group.set_max_acceleration_scaling_factor(0.4)  # 가속도를 25%로 설정

    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기


    # 동작이 완료되었는지 확인
    if plan:
        
        print("Movement successful!")
        time.sleep(1)

    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    time.sleep(0.5)

    state_check_pub.publish(1) # state = True
    print("state = False")
    time.sleep(0.1)

    get_angle_call_pub.publish(1)
    print("get_angle...")

    rospy.wait_for_message("get_angle_completed", Float32MultiArray)
    print("get_angle_completed")

    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)

    print(j1, j2, j3, j4, j5, j6)

    joint_goal = [radian(int(j1)), radian(int(j2)), radian(int(j3)), radian(int(j4)), radian(int(j5)), radian(int(j6) - int(link6_angle))]

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)

    # 속도 및 가속도 제한 설정
    arm_group.set_max_velocity_scaling_factor(0.4)  # 속도를 25%로 설정
    arm_group.set_max_acceleration_scaling_factor(0.4)  # 가속도를 25%로 설정

    # Execute the planned motion and wait until it's complete
    plan2 = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan2:
        print("Movement successful!")
        time.sleep(1)
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    time.sleep(0.5)
    busy = False  # 동작 종료

    ###############################################


def move_to_zpose(z):
    
    global busy
    busy = True  # 동작 시작

    state_check_pub.publish(1) # state = False
    print("state = False")
    time.sleep(0.1)

    move_coord_call_pub.publish(z)
    print("move_coord...")

    rospy.wait_for_message("move_coord_completed", Int32)
    print("move_coord_completed")


    busy = False  # 동작 종료



def move_gripper(gripper):
    global busy
    busy = True  # 동작 시작

    state_check_pub.publish(1) # state = False
    print("state = False")
    time.sleep(0.1)
        
    print("gripper close start")
    gripper_close_pub.publish(gripper)
    rospy.wait_for_message("gripper_close_action_completed", Int32)
    print("gripper close done")

    busy = False  # 동작 종료

def move_to_barcode():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy, data_input
    busy = True

    move_to_pose(-70,104,-85,-33,-22,12)
    
    if data_input == "":
        move_to_pose(-51,100,-72,-35,-41,2)

        if data_input == "":

            move_to_pose(-51,100,-72,-35,-41,92)

            if data_input == "":

                move_to_pose(-51,100,-72,-35,-41,-88)

                if data_input == "":
                        
                    move_to_pose(-51,100,-72,-35,-41,2)
                    busy = False  # 동작 종료
                    
                elif not data_input == "":
                        move_to_pose(-51,100,-72,-35,-41,2)
                        busy = False

            elif not data_input == "":
                move_to_pose(-51,100,-72,-35,-41,2)
                busy = False

        elif not data_input == "":
            move_to_pose(-51,100,-72,-35,-41,2)
            busy = False 
        
    elif not data_input == "":
        busy = False


def move_to_abox():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    move_to_pose(-71,68,11,10,-91,15)

    busy = False  # 동작 종료


def move_to_bbox():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    move_to_pose(-113,42,53,-7,-92,-25)
    
    busy = False  # 동작 종료

def step_callback(data):
    global link6_angle, x, y, z, gripper, step, process_start, data_input, value_input, count, z_state
    x = 0.0
    y = 0.0
    z = 0.0
    link6_angle = 0.0
    step = 0
    gripper = 0
    data_input = ""
    value_input = ""

    
    if step == 0:
        if len(data.data) == 4:  # Check if all variables are present
            try:
                # Extract x, y, z, link6_angle, and gripper from data
                x, y, link6_angle, gripper = map(float, data.data)
                step += 1
                
                if step == 1:
                    print("step: ",step)
                    if not busy:
                        print("Moving to pose!")
                        move_to_goalpose(x, y, int(link6_angle))
                        time.sleep(0.5)
                        move_gripper(int(gripper))
                        time.sleep(0.5)
                        get_z_call_pub.publish(1)
                        while not z_state:
                            rospy.sleep(0.1)  # z_state가 True가 될 때까지 대기

                        if z_state:
                            print("Moving to z pose!")
                            time.sleep(0.5)
                            move_to_zpose(float(z))
                            time.sleep(0.5)
                            print("move gripper!")
                            if (gripper - 30) < 0:
                                move_gripper(2)
                            elif gripper > 70:
                                move_gripper(2)
                            else:
                                move_gripper(int(gripper)-30)
                            time.sleep(0.5)
                            step += 1
                            z_state = False

                if step == 2:
                    print("step: ",step)
                    if not busy:
                        if data_input == "":
                            print("moving to barcode scanner!")
                            move_to_barcode()
                            time.sleep(0.5)
                            if data_input=='A':
                                print("move to a box")
                                move_to_abox()
                                time.sleep(0.5)
                                move_to_zpose(15)
                                time.sleep(0.5)
                                open_gripper()
                                time.sleep(0.5)
                                if value_input=='':
                                    step+=1
                                elif value_input=='A':
                                    send_value_pub.publish("A")
                                    count+=1
                                    step+=1

                            elif data_input=='B':
                                print("move to b box")
                                move_to_bbox()
                                time.sleep(0.5)
                                move_to_zpose(15)
                                time.sleep(0.5)
                                open_gripper()
                                time.sleep(0.5)
                                if value_input=='':
                                    step+=1
                                elif value_input=='B':
                                    send_value_pub.publish("B")
                                    count+=1
                                    step+=1
                                    
                            else:
                                move_to_failpose()
                                print("move to failpose")
                                time.sleep(0.5)
                                step += 1
                        
                        elif not data_input == "":
                            if data_input=='A':
                                print("move to a box")
                                move_to_pose(-11,-4,83,4,-90,-12)
                                time.sleep(0.5)
                                move_to_abox()
                                time.sleep(0.5)
                                open_gripper()
                                time.sleep(0.5)
                                if value_input=='':
                                    step+=1
                                elif value_input=='A':
                                    send_value_pub.publish("A")
                                    count+=1
                                    step+=1

                            elif data_input=='B':
                                print("move to b box")
                                move_to_pose(-11,-4,83,4,-90,-12)
                                time.sleep(0.5)
                                move_to_bbox()
                                time.sleep(0.5)
                                open_gripper()
                                time.sleep(0.5)
                                if value_input=='':
                                    step+=1
                                elif value_input=='B':
                                    send_value_pub.publish("B")
                                    count+=1
                                    step+=1
                            
                            else:
                                move_to_failpose()
                                print("move to failpose")
                                time.sleep(0.5)
                                step += 1
           
                if step == 3:
                    print("count: ",count)
                    print("step: ",step)
                    if not busy:
                        print("moving to start pose!")
                        move_to_startpose()
                        time.sleep(0.5)
                        if count != 2:
                            print("send start signal")
                            send_start_pub.publish(1)
                            step = 0
                        elif count == 2:
                            print("send stop signal")
                            send_stop_pub.publish(1)
                            count = 0
                            process_start = False
                            print("process_start: ", process_start)
                            step = 0

            except ValueError:
                print("Invalid data received, all elements must be numbers.")
        else:
            print("Incomplete data received, all variables must be present.")


if __name__ == '__main__':
    rospy.Subscriber("step_input", Float64MultiArray, step_callback)
    rospy.Subscriber("get_angle_completed", Float32MultiArray, angle_callback)
    rospy.Subscriber("z_input", Float32, z_callback)
    rospy.Subscriber("process_start", Int32, start_state)
    rospy.Subscriber("data_input", String, data_callback)
    rospy.Subscriber("value_input", String, value_callback)
    rospy.spin()
