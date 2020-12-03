#! /usr/bin/env python

import actionlib
from geometry_msgs.msg import PoseStamped, Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from manipulation_action_msgs.msg import TakePoseAction, TakePoseGoal, GraspAction, GraspGoal, PlaceAction, PlaceGoal
from visualization_msgs.msg import *

from giskardpy.python_interface import GiskardWrapper

server = None
menu_handler = MenuHandler()

take_pose_client = None
grasp_client = None
place_client = None
giskard_wrapper = None


def make_int_marker():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.scale = 0.5
    int_marker.pose.position.z = 1
    int_marker.pose.orientation.w = 1
    int_marker.name = "manipulation_test_marker"
    int_marker.description = str(int_marker.pose.position.x) + ", " + str(int_marker.pose.position.y) + ", " + str(
        int_marker.pose.position.z)

    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    control.always_visible = True

    control.markers.append(marker)
    int_marker.controls.append(control)
    return int_marker


def init_menu():
    pose_men = menu_handler.insert("Take pose")
    menu_handler.insert("Neutral", parent=pose_men, callback=take_neutral_pose_cb)
    menu_handler.insert("Look low", parent=pose_men, callback=take_look_low_pose_cb)
    menu_handler.insert("Look high", parent=pose_men, callback=take_look_high_pose_cb)
    menu_handler.insert("Look floor", parent=pose_men, callback=take_look_floor_pose_cb)
    menu_handler.insert("Look at marker", parent=pose_men, callback=take_look_at_marker_pose_cb)

    grasp_men = menu_handler.insert("Grasp here")
    menu_handler.insert("Front", parent=grasp_men, callback=grasp_front_cb)
    menu_handler.insert("Top", parent=grasp_men, callback=grasp_top_cb)

    place_men = menu_handler.insert("Place here")
    menu_handler.insert("Front", parent=place_men, callback=place_front_cb)
    menu_handler.insert("Top", parent=place_men, callback=place_top_cb)


def take_neutral_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.NEUTRAL)


def take_look_low_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.LOOK_LOW)


def take_look_high_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.LOOK_HIGH)


def take_look_floor_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.LOOK_FLOOR)


def take_look_at_marker_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.GAZE)


def grasp_front_cb(feedback):
    grasp_object(feedback.pose, GraspGoal.FRONT)


def grasp_top_cb(feedback):
    grasp_object(feedback.pose, GraspGoal.TOP)


def place_front_cb(feedback):
    place_object(feedback.pose, PlaceGoal.FRONT)


def place_top_cb(feedback):
    place_object(feedback.pose, PlaceGoal.TOP)


def marker_moved_cb(feedback):
    marker = server.get(feedback.marker_name)
    marker.description = str(feedback.pose.position.x) + ", " + str(feedback.pose.position.y) + ", " + str(
        feedback.pose.position.z)
    print(marker)
    server.applyChanges()


def take_pose(pose, mode):
    goal = TakePoseGoal()
    goal.pose_mode = mode
    goal.gaze_point = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
    take_pose_client.send_goal(goal)
    result = take_pose_client.wait_for_result()
    print("Result:", result)


def grasp_object(pose, mode):
    goal = GraspGoal()
    goal.grasp_mode = mode
    goal.object_frame_id = "test"

    poseS = PoseStamped()
    poseS.header.frame_id = "map"
    poseS.header.stamp = rospy.Time.now()
    poseS.pose = pose
    giskard_wrapper.add_cylinder(
        name="test",
        height=0.2,
        radius=0.07,
        pose=poseS
    )

    goal.goal_pose = poseS
    goal.object_size.x = 0.07
    grasp_client.send_goal(goal)
    result = grasp_client.wait_for_result()
    print("Result:", result)


def place_object(pose, mode):
    goal = PlaceGoal()
    goal.place_mode = mode
    goal.object_frame_id = "test"

    poseS = PoseStamped()
    poseS.header.frame_id = "map"
    poseS.header.stamp = rospy.Time.now()
    poseS.pose = pose

    goal.goal_pose = poseS
    place_client.send_goal(goal)
    result = place_client.wait_for_result()
    print("Result:", result)


if __name__ == '__main__':
    rospy.init_node("manipulation_test_marker")

    giskard_wrapper = GiskardWrapper()

    take_pose_client = actionlib.SimpleActionClient('take_pose_server', TakePoseAction)
    grasp_client = actionlib.SimpleActionClient('grasps_server', GraspAction)
    place_client = actionlib.SimpleActionClient('place_server', PlaceAction)

    take_pose_client.wait_for_server()
    grasp_client.wait_for_server()
    place_client.wait_for_server()

    server = InteractiveMarkerServer("manipulation_test_marker")

    init_menu()
    int_marker = make_int_marker()
    server.insert(int_marker, marker_moved_cb)

    menu_handler.apply(server, int_marker.name)
    server.applyChanges()

    rospy.spin()