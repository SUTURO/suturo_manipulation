#! /usr/bin/env python

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from manipulation_msgs.msg import TakePoseAction, TakePoseGoal, GraspAction, GraspGoal, PlaceAction, PlaceGoal, \
    MakePlanAction, MakePlanGoal
from visualization_msgs.msg import *

from giskardpy.python_interface import GiskardWrapper

from suturo_manipulation.gripper import Gripper

server = None
menu_handler = MenuHandler()

take_pose_client = None
grasp_client = None
place_client = None
giskard_wrapper = None
gripper = None
test_object_name = "test_object"


def make_int_marker():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.scale = 0.5
    int_marker.pose.position.z = 1
    int_marker.pose.orientation.w = 1
    int_marker.name = "manipulation_test_marker"
    int_marker.description = "{}, {}, {}".format(int_marker.pose.position.x, int_marker.pose.position.y,
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
    menu_handler.insert("Give Take", parent=pose_men, callback=take_give_take_pose_cb)

    grasp_men = menu_handler.insert("Grasp here")
    menu_handler.insert("Front", parent=grasp_men, callback=grasp_front_cb)
    menu_handler.insert("Top", parent=grasp_men, callback=grasp_top_cb)

    place_men = menu_handler.insert("Place here")
    menu_handler.insert("Front", parent=place_men, callback=place_front_cb)
    menu_handler.insert("Top", parent=place_men, callback=place_top_cb)

    plan_men = menu_handler.insert("Plan here")
    plan_grasp_men = menu_handler.insert("Grasp", parent=plan_men)
    menu_handler.insert("Front", parent=plan_grasp_men, callback=plan_grasp_front_cb)
    menu_handler.insert("Top", parent=plan_grasp_men, callback=plan_grasp_top_cb)
    plan_place_men = menu_handler.insert("Place", parent=plan_men)
    menu_handler.insert("Front", parent=plan_place_men, callback=plan_place_front_cb)
    menu_handler.insert("Top", parent=plan_place_men, callback=plan_place_top_cb)

    gripper_men = menu_handler.insert("Gripper")
    menu_handler.insert("Open", parent=gripper_men, callback=open_gripper_cb)
    menu_handler.insert("Close", parent=gripper_men, callback=close_gripper_cb)
    menu_handler.insert("Close with force", parent=gripper_men, callback=close_gripper_force_cb)
    menu_handler.insert("Get joint state", parent=gripper_men, callback=get_gripper_joint_state_cb)

    test_object_men = menu_handler.insert("Test Object")
    menu_handler.insert("Spawn Object", parent=test_object_men, callback=spawn_test_object_cb)
    menu_handler.insert("Remove Object", parent=test_object_men, callback=remove_test_object_cb)
    menu_handler.insert("Attach Object", parent=test_object_men, callback=attach_test_object_cb)
    menu_handler.insert("Detach Object", parent=test_object_men, callback=detach_test_object_cb)

    utility_men = menu_handler.insert("Utility")
    menu_handler.insert("Get All Object", parent=utility_men, callback=get_all_objects_cb)
    menu_handler.insert("Get Attached Object", parent=utility_men, callback=get_attached_objects_cb)
    menu_handler.insert("Get Robot Links", parent=utility_men, callback=get_robot_links_cb)


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


def take_give_take_pose_cb(feedback):
    take_pose(feedback.pose, TakePoseGoal.GIVE_TAKE)


def grasp_front_cb(feedback):
    grasp_object(feedback.pose, GraspGoal.FRONT)


def grasp_top_cb(feedback):
    grasp_object(feedback.pose, GraspGoal.TOP)


def place_front_cb(feedback):
    place_object(feedback.pose, PlaceGoal.FRONT)


def place_top_cb(feedback):
    place_object(feedback.pose, PlaceGoal.TOP)


def plan_grasp_front_cb(feedback):
    make_plan(feedback.pose, MakePlanGoal.FRONT, MakePlanGoal.GRASP)


def plan_grasp_top_cb(feedback):
    make_plan(feedback.pose, MakePlanGoal.TOP, MakePlanGoal.GRASP)


def plan_place_front_cb(feedback):
    make_plan(feedback.pose, MakePlanGoal.FRONT, MakePlanGoal.PLACE)


def plan_place_top_cb(feedback):
    make_plan(feedback.pose, MakePlanGoal.TOP, MakePlanGoal.PLACE)


def marker_moved_cb(feedback):
    marker = server.get(feedback.marker_name)
    marker.description = "{}, {}, {}".format(feedback.pose.position.x, feedback.pose.position.y,
                                             feedback.pose.position.z)
    server.applyChanges()


def get_all_objects_cb(feedback):
    rospy.loginfo("All Objects: {}".format(giskard_wrapper.get_object_names().object_names))


def get_attached_objects_cb(feedback):
    rospy.loginfo("Attached Objects: {}".format(giskard_wrapper.get_attached_objects().object_names))


def get_robot_links_cb(feedback):
    rospy.loginfo("Robot Links: {}".format(giskard_wrapper.get_robot_links()))


def spawn_test_object_cb(feedback):
    spawn_test_object(feedback.pose)


def remove_test_object_cb(feedback):
    remove_test_object()


def attach_test_object_cb(feedback):
    attach_test_object()


def detach_test_object_cb(feedback):
    detach_test_object()


def open_gripper_cb(feedback):
    open_gripper()


def close_gripper_cb(feedback):
    close_gripper()


def close_gripper_force_cb(feedback):
    close_gripper_force()


def get_gripper_joint_state_cb(feedback):
    get_gripper_joint_state()


def make_plan(goal_pose, gripper_mode, action_mode):
    goal = MakePlanGoal()
    goal.gripper_mode = gripper_mode
    goal.action_mode = action_mode
    goal.object_size = Vector3(x=0.05, y=0.05, z=0.2)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose = goal_pose
    goal.goal_pose = pose
    start = rospy.Time.now()
    make_plan_client.send_goal(goal)
    make_plan_client.wait_for_result()
    result = make_plan_client.get_result()
    rospy.loginfo("Make plan result: {}".format(result.error_code))
    rospy.loginfo("Execution time: {:.2f}s".format((rospy.Time.now() - start).to_sec()))


def take_pose(pose, mode):
    goal = TakePoseGoal()
    goal.pose_mode = mode
    goal.gaze_point = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
    start = rospy.Time.now()
    take_pose_client.send_goal(goal)
    take_pose_client.wait_for_result()
    result = take_pose_client.get_result()
    rospy.loginfo("take_pose result: {}".format(result.error_code))
    rospy.loginfo("Execution time: {:.2f}s".format((rospy.Time.now() - start).to_sec()))


def grasp_object(pose, mode):
    goal = GraspGoal()
    goal.grasp_mode = mode
    goal.object_frame_id = test_object_name

    pose_grasp = PoseStamped()
    pose_grasp.header.frame_id = "map"
    pose_grasp.header.stamp = rospy.Time.now()
    pose_grasp.pose = pose

    goal.goal_pose = pose_grasp
    goal.object_size.x = 0.07
    start = rospy.Time.now()
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()
    result = grasp_client.get_result()
    rospy.loginfo("grasp_object result: {}".format(result.error_code))
    rospy.loginfo("Execution time: {:.2f}s".format((rospy.Time.now() - start).to_sec()))


def place_object(pose, mode):
    goal = PlaceGoal()
    goal.place_mode = mode
    goal.object_frame_id = test_object_name

    place_pose = PoseStamped()
    place_pose.header.frame_id = "map"
    place_pose.header.stamp = rospy.Time.now()
    place_pose.pose = pose

    goal.goal_pose = place_pose
    start = rospy.Time.now()
    place_client.send_goal(goal)
    place_client.wait_for_result()
    result = place_client.get_result()
    rospy.loginfo("place_object result: {}".format(result.error_code))
    rospy.loginfo("Execution time: {:.2f}s".format((rospy.Time.now() - start).to_sec()))


def get_gripper_joint_state():
    joint_states = giskard_wrapper.get_joint_states(u'/hsrb/joint_states')
    if joint_states.has_key(u'hand_motor_joint'):
        result = joint_states[u'hand_motor_joint']
        rospy.loginfo("Gripper hand_motor_joint: {}".format(result))
    else:
        rospy.loginfo("Unable to get Gripper hand_motor_joint state.")


def open_gripper():
    gripper.set_gripper_joint_position(1.2)


def close_gripper():
    gripper.set_gripper_joint_position(-0.8)


def close_gripper_force():
    gripper.close_gripper_force()


def spawn_test_object(pose):
    remove_test_object()
    pose_S = PoseStamped()
    pose_S.header.frame_id = "map"
    pose_S.header.stamp = rospy.Time.now()
    pose_S.pose = pose
    giskard_wrapper.add_cylinder(name=test_object_name, height=0.2, radius=0.07, pose=pose_S)


def remove_test_object():
    if test_object_name in giskard_wrapper.get_object_names().object_names:
        giskard_wrapper.remove_object(test_object_name)


def attach_test_object():
    if test_object_name in giskard_wrapper.get_object_names().object_names:
        giskard_wrapper.attach_object(test_object_name, "hand_palm_link")


def detach_test_object():
    if test_object_name in giskard_wrapper.get_attached_objects().object_names:
        giskard_wrapper.detach_object(test_object_name)


if __name__ == '__main__':
    rospy.init_node("manipulation_test_marker")
    giskard_wrapper = GiskardWrapper()
    gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                      follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
    # connect to servers
    take_pose_client = actionlib.SimpleActionClient('take_pose_server', TakePoseAction)
    grasp_client = actionlib.SimpleActionClient('grasp_server', GraspAction)
    place_client = actionlib.SimpleActionClient('place_server', PlaceAction)
    make_plan_client = actionlib.SimpleActionClient('make_plan_server', MakePlanAction)
    take_pose_client.wait_for_server()
    grasp_client.wait_for_server()
    place_client.wait_for_server()
    make_plan_client.wait_for_server()
    # create interactive marker
    server = InteractiveMarkerServer("manipulation_test_marker")
    init_menu()
    manipulation_test_marker = make_int_marker()
    server.insert(manipulation_test_marker, marker_moved_cb)
    menu_handler.apply(server, manipulation_test_marker.name)
    server.applyChanges()
    # spin node
    rospy.spin()
