cmake_minimum_required(VERSION 2.8.3)
project(manipulation_test_marker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  manipulation_action_msgs
  interactive_markers
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grasps_object_server
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
