cmake_minimum_required(VERSION 2.8.3)
project(take_pose_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  giskardpy
  rospy
  manipulation_action_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES take_pose_action_server
  CATKIN_DEPENDS actionlib giskardpy rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


