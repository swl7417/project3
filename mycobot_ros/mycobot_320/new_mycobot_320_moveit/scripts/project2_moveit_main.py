# # test_movepose.py
# #!/usr/bin/env python
# import sys
# import rospy
# import moveit_commander
# from moveit_msgs.msg import MoveItErrorCodes
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Int32
# from std_msgs.msg import Float32
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import Int32MultiArray
# import geometry_msgs.msg
# import math
# import time

# # 노드 초기화
# rospy.init_node('project2_moveit_node', anonymous=True)
# state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
# state_check2_pub = rospy.Publisher("state_check2", Int32, queue_size=10)
# gripper_close_pub = rospy.Publisher("gripper_close", Int32, queue_size=10)
# gripper_open_pub = rospy.Publisher("gripper_open", Int32, queue_size=10)
# get_angle_call_pub = rospy.Publisher("get_angles", Int32, queue_size=10)
# move_coord_call_pub = rospy.Publisher("move_coords", Float32, queue_size=10)
# send_start_pub = rospy.Publisher("start", Int32, queue_size=10)
# send_movexy_pub = rospy.Publisher("move_xy", Int32MultiArray, queue_size=10)
# busy = False  # 동작 중인지 여부를 나타내는 플래그

# global color1_count, color2_count, color3_count

# color1_count = 0
# color2_count = 0
# color3_count = 0

# # MoveIt! 초기화
# moveit_commander.roscpp_initialize(sys.argv)

# # 로봇 모델 로딩
# robot = moveit_commander.RobotCommander()

# scene = moveit_commander.PlanningSceneInterface()

# # 로봇 팔 그룹 초기화
# arm_group = moveit_commander.MoveGroupCommander("arm_group")
# gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

# def gripper_control():
#     state_check2_pub.publish(1) # state = True
#     print("state = True")
#     time.sleep(0.1)

#     global busy
#     busy = True

#     target_joint_value = [0.6]
#     gripper_group.set_joint_value_target(target_joint_value)
#     plan = gripper_group.go(wait= True)

#     # 동작이 완료되었는지 확인
#     if plan:
#         print("Movement successful!")
#         time.sleep(1)
        
        
#     else:
#         print("Movement failed!")
    
#     gripper_group.stop()
#     gripper_group.clear_pose_targets()
#     busy = False  # 동작 종료




# def radian(angle_degrees):
#     return angle_degrees * math.pi / 180.0

# def move_to_startpose():
#     state_check2_pub.publish(1) # state = True
#     print("state = True")
#     time.sleep(0.1)
    
#     global busy
#     busy = True  # 동작 시작
    
#     # 조인트 각도 설정
#     joint_goal = [radian(-55), radian(-10), radian(38),radian(60), radian(-91), radian(-56)] 

#     # 매니퓰레이터에 조인트 각도 목표 설정
#     arm_group.set_joint_value_target(joint_goal)

#     # Execute the planned motion and wait until it's complete
#     plan = arm_group.go(wait=True)

#     # 동작이 완료되었는지 확인
#     if plan:
#         print("Movement successful!")
#         time.sleep(1)
        
        
#     else:
#         print("Movement failed!")
    
#     arm_group.stop()
#     arm_group.clear_pose_targets()
#     busy = False  # 동작 종료


# def move_to_middlepose():
#     state_check2_pub.publish(1) # state = True
#     print("state = True")
#     time.sleep(0.1)
    
#     global busy
#     busy = True  # 동작 시작

#     # 조인트 각도 설정
#     joint_goal = [radian(-33), radian(21), radian(18), radian(49), radian(-90), radian(-34)]

#     # 매니퓰레이터에 조인트 각도 목표 설정
#     arm_group.set_joint_value_target(joint_goal)

#     # Execute the planned motion and wait until it's complete
#     plan = arm_group.go(wait=True)

#     # 동작이 완료되었는지 확인
#     if plan:
#         print("Movement successful!")
#         time.sleep(1)
        
        
#     else:
#         print("Movement failed!")
    
#     arm_group.stop()
#     arm_group.clear_pose_targets()
#     busy = False  # 동작 종료



# def move_to_pose(link6_angle):
#     state_check2_pub.publish(1) # state = False
#     print("state = True")
#     time.sleep(0.1)
    
#     global busy
#     busy = True  # 동작 시작

#     # 조인트 각도 설정
#     joint_goal = [radian(-33), radian(6), radian(62), radian(19), radian(-90), radian(-34 + link6_angle)] 

#     # 매니퓰레이터에 조인트 각도 목표 설정
#     arm_group.set_joint_value_target(joint_goal)

#     # # Plan the motion
#     # arm_group.plan()

#     # Execute the planned motion and wait until it's complete
#     plan = arm_group.go(wait=True)

#     # 동작이 완료되었는지 확인
#     if plan:
#         print("Movement successful!")
#         time.sleep(1)
        
#     else:
#         print("Movement failed!")
    
#     arm_group.stop()
#     arm_group.clear_pose_targets()

#     busy = False  # 동작 종료

#     ###############################################3


# def move_to_zpose(z):
    
#     global busy
#     busy = True  # 동작 시작

#     state_check_pub.publish(1) # state = False
#     print("state = False")
#     time.sleep(0.1)

#     move_coord_call_pub.publish(z)
#     print("move_coord...")

#     rospy.wait_for_message("move_coord_completed", Int32)
#     print("move_coord_completed")


#     busy = False  # 동작 종료


# def move_gripper(gripper):
#     global busy
#     busy = True  # 동작 시작

#     state_check_pub.publish(1) # state = False
#     print("state = False")
#     time.sleep(0.1)
        
#     print("gripper close start")
#     gripper_close_pub.publish(gripper)
#     rospy.wait_for_message("gripper_close_action_completed", Int32)
#     print("gripper close done")

#     busy = False  # 동작 종료


# def open_gripper():
#     global busy
#     busy = True
#     state_check_pub.publish(1) # state = False
#     print("state = False")
#     time.sleep(0.1)
        
#     print("gripper open start")
#     gripper_open_pub.publish(1)
#     rospy.wait_for_message("gripper_open_action_completed", Int32)
#     print("gripper open done")

#     busy = False



# def move_to_space(l1,l2,l3,l4,l5,l6):
#     state_check2_pub.publish(1) # state = True
#     print("state = True")
#     time.sleep(0.1)
    
#     global busy
#     busy = True  # 동작 시작

#     # 조인트 각도 설정
#     joint_goal = [radian(l1), radian(l2), radian(l3), radian(l4), radian(l5), radian(l6)] 

#     # 매니퓰레이터에 조인트 각도 목표 설정
#     arm_group.set_joint_value_target(joint_goal)

#     # # Plan the motion
#     # arm_group.plan()

#     # Execute the planned motion and wait until it's complete
#     plan = arm_group.go(wait=True)

#     # 동작이 완료되었는지 확인
#     if plan:
        
#         print("Movement successful!")
#         time.sleep(1)

#     else:
#         print("Movement failed!")
    
#     arm_group.stop()
#     arm_group.clear_pose_targets()

#     busy = False


# def angle_callback(data):
#     global j1, j2, j3, j4, j5, j6
#     j1 = 0.00
#     j2 = 0.00
#     j3 = 0.00
#     j4 = 0.00
#     j5 = 0.00 
#     j6 = 0.00
#     if len(data.data) == 6:
#         try:
#             # Extract x, y, z, link6_angle, and gripper from data
#             j1, j2, j3, j4, j5, j6 = map(float, data.data)

#         except ValueError:
#                 print("Invalid data received, all elements must be numbers.")
#         else:
#             print("Incomplete data received, all variables must be present.")




# def keyboard_callback(data):
#     global moveit_x, moveit_y, angle, id, step, color1_count, color2_count, color3_count
#     moveit_x = 0.0
#     moveit_y = 0.0
#     angle = 0.0
#     id = 0
#     step = 0
    
#     if step == 0:
#         if len(data.data) == 4:  # Check if all variables are present
#             try:
#                 # Extract x, y, z, link6_angle, and gripper from data
#                 moveit_x, moveit_y, angle, id = data.data[0], data.data[1], data.data[2], data.data[3]
#                 step += 1
#                 print(moveit_x, moveit_y, angle, id)
#                 print(type(moveit_x), type(moveit_y), type(angle), type(id))


#                 if step == 1:
#                     print(step)
#                     if not busy:
#                         print("Moving to pose!")
#                         move_to_pose(int(angle))
#                         print("move gripper!")
#                         move_gripper(10)
#                         if id == 4:
#                             step += 1
#                         else:
#                             move_to_middlepose()
#                             step += 1

#                 if step == 2:
#                     if not busy:
#                         if id == 1: # blue
#                             print(step)
#                             print("move to space1!")
#                             move_to_space(-105,-2,89,-1,-93,-104)
#                             color1_count += 1
#                             if color1_count == 1:
#                                 print(id)
#                                 print(color1_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(12.1)                                                   
#                                 open_gripper()
#                                 step += 1
#                                 id = 0

#                             if color1_count == 2:
#                                 print(id)
#                                 print(color1_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(8.3)                         
#                                 open_gripper()
#                                 step += 1
#                                 id = 0

#                             if color1_count == 3:
#                                 print(id)
#                                 print(color1_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(4.3)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0
#                                 color1_count = 0

#                         if id == 2: # orange
#                             print(step)
#                             print("move to space2!")
#                             move_to_space(-88, 5, 81, 2, -92, -87)
#                             color2_count += 1
#                             if color2_count == 1:
#                                 print(id)
#                                 print(color2_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(12.1)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0

#                             if color2_count == 2:
#                                 print(id)
#                                 print(color2_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(8)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0
                                
#                             if color2_count == 3:
#                                 print(id)
#                                 print(color2_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(4.3)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0
#                                 color2_count = 0

#                         if id == 3: # green
#                             print(step)
#                             print("move to space3!")
#                             move_to_space(-71,20,62,5,-91,-73)
#                             color3_count += 1
#                             if color3_count == 1:
#                                 print(id)
#                                 print(color3_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(12.1)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0

#                             if color3_count == 2:
#                                 print(id)
#                                 print(color3_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(8.4)
#                                 open_gripper()
#                                 step += 1
#                                 id = 0
                                
#                             if color3_count == 3:
#                                 print(id)
#                                 print(color3_count)
#                                 print("Moving to z pose")
#                                 move_to_zpose(4.3)
#                                 open_gripper() 
#                                 step += 1
#                                 id = 0
#                                 color3_count = 0

#                         if id == 4: # purple
#                             print(step)
#                             print("move to space4!")
#                             move_to_space(-51,33,22,33,-89,-52)
#                             open_gripper()
#                             step += 1     
        
#                 if step == 3:
#                     if not busy:
#                         print(step) 
#                         move_to_startpose()
#                         time.sleep(1)
#                         print("send start signal!")
#                         send_start_pub.publish(1) # 보내는 데이터 꼭 1이어야 함
#                         step = 0

#             except ValueError:
#                 print("Invalid data received, all elements must be numbers.")
#         else:
#             print("Incomplete data received, all variables must be present.")

# if __name__ == '__main__':
#     # rospy.Subscriber("step_input", Int32, keyboard_callback)
#     print("move to startpose!")
#     move_to_startpose()
#     print("send start signal!")
#     if not busy: 
#         send_start_pub.publish(1)
#     rospy.Subscriber("step_input", Int32MultiArray, keyboard_callback)
#     rospy.Subscriber("get_angle_completed", Float32MultiArray, angle_callback)
#     rospy.spin()

# test_movepose.py
#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
import math
import time

# 노드 초기화
rospy.init_node('project2_moveit_node', anonymous=True)
state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
state_check2_pub = rospy.Publisher("state_check2", Int32, queue_size=10)
gripper_close_pub = rospy.Publisher("gripper_close", Int32, queue_size=10)
gripper_open_pub = rospy.Publisher("gripper_open", Int32, queue_size=10)
get_angle_call_pub = rospy.Publisher("get_angles", Int32, queue_size=10)
move_coord_call_pub = rospy.Publisher("move_coords", Float32, queue_size=10)
send_start_pub = rospy.Publisher("start", Int32, queue_size=10)
send_movexy_pub = rospy.Publisher("move_xy", Int32MultiArray, queue_size=10)
busy = False  # 동작 중인지 여부를 나타내는 플래그

global color1_count, color2_count, color3_count

color1_count = 0
color2_count = 0
color3_count = 0

# MoveIt! 초기화
moveit_commander.roscpp_initialize(sys.argv)

# 로봇 모델 로딩
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

# 로봇 팔 그룹 초기화
arm_group = moveit_commander.MoveGroupCommander("arm_group")
gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

def control_gripper(position):
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.1)

    # gripper 관절 이름 설정
    joint_names = ['gripper_joint']

    # 목표 위치 설정
    point = JointTrajectoryPoint()
    point.positions = [position]  # position은 gripper를 원하는 위치로 지정

    # trajectory 메시지 설정
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(point)

    # publish trajectory 메시지
    pub.publish(trajectory)


def radian(angle_degrees):
    return angle_degrees * math.pi / 180.0

def move_to_startpose():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작
    
    # 조인트 각도 설정
    joint_goal = [radian(-55), radian(-10), radian(38),radian(60), radian(-91), radian(-56)] 

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)

    # Execute the planned motion and wait until it's complete
    plan = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)
        
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    busy = False  # 동작 종료


def move_to_middlepose():
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    # 조인트 각도 설정
    joint_goal = [radian(-33), radian(21), radian(18), radian(49), radian(-90), radian(-34)]

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)

    # Execute the planned motion and wait until it's complete
    plan = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)
        
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    busy = False  # 동작 종료



def move_to_pose(link6_angle):
    state_check2_pub.publish(1) # state = False
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    # 조인트 각도 설정
    joint_goal = [radian(-33), radian(6), radian(62), radian(19), radian(-90), radian(-34 + link6_angle)] 

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)

    # # Plan the motion
    # arm_group.plan()

    # Execute the planned motion and wait until it's complete
    plan = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan:
        print("Movement successful!")
        time.sleep(5)
        
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    busy = False  # 동작 종료

    ###############################################3


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


def open_gripper():
    global busy
    busy = True
    state_check_pub.publish(1) # state = False
    print("state = False")
    time.sleep(0.1)
        
    print("gripper open start")
    gripper_open_pub.publish(1)
    rospy.wait_for_message("gripper_open_action_completed", Int32)
    print("gripper open done")

    busy = False



def move_to_space(l1,l2,l3,l4,l5,l6):
    state_check2_pub.publish(1) # state = True
    print("state = True")
    time.sleep(0.1)
    
    global busy
    busy = True  # 동작 시작

    # 조인트 각도 설정
    joint_goal = [radian(l1), radian(l2), radian(l3), radian(l4), radian(l5), radian(l6)] 

    # 매니퓰레이터에 조인트 각도 목표 설정
    arm_group.set_joint_value_target(joint_goal)

    # # Plan the motion
    # arm_group.plan()

    # Execute the planned motion and wait until it's complete
    plan = arm_group.go(wait=True)

    # 동작이 완료되었는지 확인
    if plan:
        
        print("Movement successful!")
        time.sleep(5)

    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    busy = False


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
            # Extract x, y, z, link6_angle, and gripper from data
            j1, j2, j3, j4, j5, j6 = map(float, data.data)

        except ValueError:
                print("Invalid data received, all elements must be numbers.")
        else:
            print("Incomplete data received, all variables must be present.")




def keyboard_callback(data):
    global moveit_x, moveit_y, angle, id, step, color1_count, color2_count, color3_count
    moveit_x = 0.0
    moveit_y = 0.0
    angle = 0.0
    id = 0
    step = 0
    
    if step == 0:
        if len(data.data) == 4:  # Check if all variables are present
            try:
                # Extract x, y, z, link6_angle, and gripper from data
                moveit_x, moveit_y, angle, id = data.data[0], data.data[1], data.data[2], data.data[3]
                step += 1
                print(moveit_x, moveit_y, angle, id)
                print(type(moveit_x), type(moveit_y), type(angle), type(id))


                if step == 1:
                    print(step)
                    if not busy:
                        print("Moving to pose!")
                        move_to_pose(int(angle))
                        print("move gripper!")
                        
                        # move_gripper(10)
                        if id == 4:
                            step += 1
                        else:
                            move_to_middlepose()
                            step += 1

                if step == 2:
                    if not busy:
                        if id == 1: # blue
                            print(step)
                            print("move to space1!")
                            move_to_space(-105,-2,89,-1,-93,-104)
                            color1_count += 1
                            if color1_count == 1:
                                print(id)
                                print(color1_count)
                                print("Moving to z pose")
                                # move_to_zpose(12.1)                                                   
                                # open_gripper()
                                step += 1
                                id = 0

                            if color1_count == 2:
                                print(id)
                                print(color1_count)
                                print("Moving to z pose")
                                # move_to_zpose(8.3)                         
                                # open_gripper()
                                step += 1
                                id = 0

                            if color1_count == 3:
                                print(id)
                                print(color1_count)
                                print("Moving to z pose")
                                # move_to_zpose(4.3)
                                # open_gripper()
                                step += 1
                                id = 0
                                color1_count = 0

                        if id == 2: # orange
                            print(step)
                            print("move to space2!")
                            move_to_space(-88, 5, 81, 2, -92, -87)
                            color2_count += 1
                            if color2_count == 1:
                                print(id)
                                print(color2_count)
                                print("Moving to z pose")
                                # move_to_zpose(12.1)
                                # open_gripper()
                                step += 1
                                id = 0

                            if color2_count == 2:
                                print(id)
                                print(color2_count)
                                print("Moving to z pose")
                                # move_to_zpose(8)
                                # open_gripper()
                                step += 1
                                id = 0
                                
                            if color2_count == 3:
                                print(id)
                                print(color2_count)
                                print("Moving to z pose")
                                # move_to_zpose(4.3)
                                # open_gripper()
                                step += 1
                                id = 0
                                color2_count = 0

                        if id == 3: # green
                            print(step)
                            print("move to space3!")
                            move_to_space(-71,20,62,5,-91,-73)
                            color3_count += 1
                            if color3_count == 1:
                                print(id)
                                print(color3_count)
                                print("Moving to z pose")
                                # move_to_zpose(12.1)
                                # open_gripper()
                                step += 1
                                id = 0

                            if color3_count == 2:
                                print(id)
                                print(color3_count)
                                print("Moving to z pose")
                                # move_to_zpose(8.4)
                                # open_gripper()
                                step += 1
                                id = 0
                                
                            if color3_count == 3:
                                print(id)
                                print(color3_count)
                                print("Moving to z pose")
                                # move_to_zpose(4.3)
                                # open_gripper() 
                                step += 1
                                id = 0
                                color3_count = 0

                        if id == 4: # purple
                            print(step)
                            print("move to space4!")
                            move_to_space(-51,33,22,33,-89,-52)
                            # open_gripper()
                            step += 1     
        
                if step == 3:
                    if not busy:
                        print(step) 
                        move_to_startpose()
                        time.sleep(1)
                        print("send start signal!")
                        send_start_pub.publish(1) # 보내는 데이터 꼭 1이어야 함
                        step = 0

            except ValueError:
                print("Invalid data received, all elements must be numbers.")
        else:
            print("Incomplete data received, all variables must be present.")

if __name__ == '__main__':
    print("move to startpose!")
    move_to_startpose()
    print("control gripper!")
    control_gripper(radian(60))
    move_to_pose(int(30))
    move_to_middlepose()
    move_to_space(-71,20,62,5,-91,-73)
    print("send start signal!")
    if not busy: 
        send_start_pub.publish(1)
    rospy.Subscriber("step_input", Int32MultiArray, keyboard_callback)
    rospy.Subscriber("get_angle_completed", Float32MultiArray, angle_callback)
    rospy.spin()