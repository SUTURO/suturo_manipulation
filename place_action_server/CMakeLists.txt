cmake_minimum_required(VERSION 2.8.3)
project(place_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  giskardpy
  manipulation_action_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES place_action_server
  CATKIN_DEPENDS actionlib giskardpy manipulation_action_msgs rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

