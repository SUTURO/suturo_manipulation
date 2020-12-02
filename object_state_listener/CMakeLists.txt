cmake_minimum_required(VERSION 2.8.3)
project(object_state_listener)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  giskardpy
  knowledge_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_state_listener
  CATKIN_DEPENDS geometry_msgs giskardpy knowledge_msgs rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
