cmake_minimum_required(VERSION 3.0.2)
project(second_coursework)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
)

add_action_files(
  FILES
  CreateSemanticMap.action
  EmptyAction.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/main_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
