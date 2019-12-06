cmake_minimum_required(VERSION 2.8.3)
project(move_gripper_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  actionlib_msgs
  giskard_msgs
  giskardpy
  rospy
  std_msgs
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveGripper.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs 
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES move_gripper_action_server
  CATKIN_DEPENDS actionlib geometry_msgs giskard_msgs giskardpy message_generation rospy
  DEPENDS system_lib
  DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Install scripts and executables
install(PROGRAMS
  src/move_gripper_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
