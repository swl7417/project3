from pymycobot.mycobot import MyCobot
import time
import keyboard



mc = MyCobot('/dev/ttyACM0',115200)


# time.sleep(2)
# mc.send_angles([0, 0, 0, 0, 0, 0],30)
# time.sleep(4)
# coords = mc.get_coords()
# print(coords)


# mc.send_coords([-232.8, -49.4, 191.5, 180, 0, 90], 20, 0)
# time.sleep(5)
# mc.send_coords([-169.8, 225.7, 229.0-(0.632*0), 178.14, -4, 90.62], 20, 0)
# time.sleep(3)
# angles = mc.get_angles()
# time.sleep(2)
# print(angles)





# # # #gripper control
# mc.set_gripper_calibration()

mc.set_gripper_mode(0)
time.sleep(1)


mc.init_eletric_gripper()
time.sleep(1)

# mc.set_eletric_gripper(0)
mc.set_gripper_value(0,20)
time.sleep(3)

# mc.set_eletric_gripper(1)
mc.set_gripper_value(100,20)
time.sleep(3)


# mc.release_all_servos()

# # angles = mc.get_angles()
# # print("angle: ", angles)
# time.sleep(1)
# coding=utf-8

# #IO與▼폀

# mc.set_gripper_mode(1)
# for i in range(3):
#      mc.set_digital_output(23,0)
#      time.sleep(2)
#      mc.set_digital_output(23,1)
#      time.sleep(2)
#      mc.set_digital_output(33,0)#�뿭�릦鸚밭닼
#      time.sleep(2)
#      mc.set_digital_output(33,1)#IO�걿鸚띴퐥�뵷亮�
#      time.sleep(2)   


# mc.set_gripper_mode(1)#IO與▼폀
# # for i in range(3):
# #      mc.set_digital_output(23,0)#IO�걿鸚띴퐥�뵷亮�
# #      time.sleep(2)
# #      mc.set_digital_output(23,1)
# #      time.sleep(2)
# #      mc.set_digital_output(33,0)#�뿭�릦鸚밭닼
# #      time.sleep(2)
# #      mc.set_digital_output(33,1)#IO�걿鸚띴퐥�뵷亮�
# #      time.sleep(2)   

# mc.set_digital_output(23,0)#open gripper
# time.sleep(2)
# mc.set_digital_output(33,0)#close gripper
# mc.set_gripper_mode(0)
# print("set 0")
# while mc.is_gripper_moving():
#      print("ggripper movvvvvvvveeeing")
#      time.sleep(0.5)
# mc.set_gripper_mode(1)
# mc.set_digital_output(33,1)#stop gripper close
