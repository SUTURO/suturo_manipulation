cmake_minimum_required(VERSION 2.8.3)
project(perceive_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  giskardpy
  rospy
  manipulation_action_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES perceive_action_server
  CATKIN_DEPENDS actionlib giskardpy rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Install scripts and executables
install(PROGRAMS
  src/perceive_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
