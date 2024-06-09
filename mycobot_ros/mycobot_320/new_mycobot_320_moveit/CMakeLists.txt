# cmake_minimum_required(VERSION 3.0.2)
# project(new_mycobot_320_moveit)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)


# find_package(catkin REQUIRED)

# catkin_package()

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN "setup_assistant.launch" EXCLUDE)
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


cmake_minimum_required(VERSION 3.0.2)
project(new_mycobot_320_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  rospy
  std_msgs
)

# catkin_python_setup()

catkin_package(
  # CATKIN_DEPENDS std_msgs actionlib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories(include${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/test_movetopose2.py
  scripts/main.py
  scripts/project2_moveit_main.py
  scripts/step.py
  scripts/send_z.py
  scripts/project2_yolo.py
  scripts/barcode.py
  scripts/test_node.py
  scripts/process2.py
  scripts/process2_conv.py
  scripts/agv_socket.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


