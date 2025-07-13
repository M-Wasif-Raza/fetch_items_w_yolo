cmake_minimum_required(VERSION 3.0.2)
project(resit_coursework)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  message_generation
)

add_message_files(
  FILES
  HotelRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/main_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)