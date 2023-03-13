#!/usr/bin/env python
import time

import rospy
import math  # for sin and cos
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3

from giskardpy.goals.suturo import SetBasePosition
from giskardpy.python_interface import GiskardWrapper
import actionlib
import tf
import hsrb_interface
# Giskard benutzen!
from manipulation_msgs.msg import MoveGripperAction, MoveGripperActionFeedback, MoveGripperActionResult, \
    MoveGripperActionGoal

from suturo_manipulation.gripper import Gripper


class MoveGripperServer:
    """
    Action Server, which handles the pose taking.
    """
    _feedback = MoveGripperActionFeedback()
    _result = MoveGripperActionResult()
    _goal = MoveGripperActionGoal()

    def __init__(self, name):
        """
        Initializes the Server.
        :param name The server name
        :type name string
        """
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveGripperAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

        self.listener = tf.TransformListener()

        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')

        '''
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'arm_roll_link'
        base_goal.pose.position = Point(0, 0, 0)
        base_goal.pose.orientation = Quaternion(0, 0, 0, 1)

        self._giskard_wrapper.set_cart_goal(root_link='arm_roll_link', tip_link='hand_palm_link', goal_pose=base_goal)
        '''

        self._giskard_wrapper.set_joint_goal({'arm_roll_joint': 1.6})
        self._giskard_wrapper.plan_and_execute()

        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

        self.robot_x = -2.5
        self.robot_y = 2.5
        '''
        # definition mueslibox
        mueslibox_center = PointStamped()
        mueslibox_center.point.x = 0 #-0.434721 - self.robot_x
        mueslibox_center.point.y = 1.8
        mueslibox_center.point.z = 0.6

        mueslibox = Vector3Stamped()
        mueslibox.vector.z = 1
        print(mueslibox)

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.vector.x = 1
        print(tip_grasp_axis)
        self._giskard_wrapper.add_box(name='asdf')
        self._giskard_wrapper.allow_collision(group1='asdf', group2='hsrb')

        self._giskard_wrapper.set_json_goal(bar_center=mueslibox_center,
                                                 bar_axis=mueslibox,
                                                 bar_length=0.05,
                                                 root_link='map',
                                                 tip_link='hand_palm_link',
                                                 tip_grasp_axis=tip_grasp_axis)
        self._giskard_wrapper.plan_and_execute()
        #'''

    def execute_cb(self, goal):
        """
        Executes the pose taking
        :param goal The goal of this action
        :type goal TakePoseGoal
        """
        rospy.loginfo("Take pose: {}".format(goal))

        new_goal = self.listener.transformPose('map', goal.goal_pose)
        rospy.loginfo("Take pose: {}".format(new_goal))
        self._result.result.error_code = self._result.result.FAILED

        new_goal.pose.position.z += 0.3
        new_goal.pose.orientation.x = math.sin(270 / 2)
        new_goal.pose.orientation.y = 0
        new_goal.pose.orientation.z = 0
        new_goal.pose.orientation.w = math.cos(270 / 2)

        self._giskard_wrapper.set_cart_goal(root_link='map', tip_link='hand_palm_link', goal_pose=new_goal)

        self._giskard_wrapper.allow_all_collisions()
        # self._giskard_wrapper.avoid_all_collisions(0.1)
        # self._giskard_wrapper.allow_collision('iai_kitchen')
        self._giskard_wrapper.plan_and_execute()

        result = self._giskard_wrapper.get_result(rospy.Duration(120))
        if result.SUCCESS in result.error_codes:
            self._result.result.error_code = self._result.result.SUCCESS
        self._as.set_succeeded(self._result)


def prepare_variables():
    _giskard_wrapper = GiskardWrapper()

    # mueslibox variables
    mueslibox_center = PoseStamped()
    mueslibox_center.header.frame_id = 'map'
    mueslibox_center.pose.position.x = - 0.1
    mueslibox_center.pose.position.y = 1.68
    mueslibox_center.pose.position.z = 0.7

    mueslibox_tip_link = "hand_palm_link"

    box_z_length = 0.2

    floor_pose = PoseStamped()
    floor_pose.header.frame_id = 'map'
    floor_pose.pose.position.x = - 0.1
    floor_pose.pose.position.y = 1.68
    floor_pose.pose.position.z = 0.6

    pick_direction = Vector3()
    pick_direction.z = 1

    pick_distance = 0.1

    mueslibox_vars = (mueslibox_center, mueslibox_tip_link, box_z_length, floor_pose, pick_direction, pick_distance)

    # drawer variables

    # y = -0.02 for test without gripper
    # y = -0.225 for real test.
    drawer_point = PoseStamped()
    drawer_point.header.frame_id = 'map'
    drawer_point.pose.position.x = 0.18
    drawer_point.pose.position.y = -0.225
    drawer_point.pose.position.z = 0.282

    drawer_open_direction = Vector3()
    drawer_open_direction.y = 1

    drawer_close_direction = Vector3()
    drawer_close_direction.y = -1

    drawer_distance = 0.1

    drawer_vars = (drawer_point, drawer_open_direction, drawer_distance, drawer_close_direction)

    return _giskard_wrapper, mueslibox_vars, drawer_vars


def test_muesli(point):
    goal_pose = PoseStamped()
    goal_pose.header = point.header
    # goal_pose.position = point.point
    goal_pose.pose.position = point.point
    _giskard_wrapper.set_cart_goal(goal_pose, "hand_palm_link", "map")


def test_open_drawer(point):
    goal_pose = PoseStamped()
    goal_pose.header = point.header
    # goal_pose.position = point.point
    goal_pose.pose.position = point.point
    _giskard_wrapper.set_cart_goal(goal_pose, "hand_palm_link", "map")


def test_open_gripper():
    g.set_gripper_joint_position(1)


def test_close_gripper():
    g.close_gripper_force(1)


def show_open_close_drawer():
    # move to drawer and open gripper
    _giskard_wrapper.grasp_box(box_pose=drawer_variables[0], box_z=0.001, grasp_type=False)
    test_open_gripper()
    _giskard_wrapper.plan_and_execute()

    time.sleep(20)

    # close gripper
    test_close_gripper()
    _giskard_wrapper.plan_and_execute()

    time.sleep(5)

    # open drawer
    _giskard_wrapper.move_drawer(knob_pose=drawer_variables[0],
                                 direction=drawer_variables[1],
                                 distance=drawer_variables[2])
    _giskard_wrapper.plan_and_execute()

    time.sleep(5)

    # close drawer
    _giskard_wrapper.move_drawer(knob_pose=drawer_variables[0],
                                 direction=drawer_variables[3],
                                 distance=drawer_variables[2])
    _giskard_wrapper.plan_and_execute()


def show_pick_place_mueslibox():
    _giskard_wrapper.grasp_box(box_pose=mueslibox_variables[0], box_z=0.001, grasp_type=True)
    _giskard_wrapper.move_gripper(open_gripper=True)
    _giskard_wrapper.plan_and_execute()

    time.sleep(15)

    # close gripper
    _giskard_wrapper.move_gripper(open_gripper=False)
    _giskard_wrapper.plan_and_execute()

    time.sleep(10)

    _giskard_wrapper.move_drawer(mueslibox_variables[0], mueslibox_variables[4], mueslibox_variables[5], align_horizontal=True)
    _giskard_wrapper.plan_and_execute()

    time.sleep(15)

    _giskard_wrapper.place_object(goal_pose=mueslibox_variables[3], object_height=mueslibox_variables[2])

    _giskard_wrapper.plan_and_execute()

    print("End")
    #time.sleep(10)




if __name__ == '__main__':
    rospy.init_node('milestone0_server')

    _giskard_wrapper, mueslibox_variables, drawer_variables = prepare_variables()

    # Add box to giskard

    box_testing = False

    box_name = 'boxy'
    if box_testing:
        box_size = (0.04, 0.1, 0.2)
        box_pose = mueslibox_variables[0]

        _giskard_wrapper.add_box(name=box_name,
                                 size=box_size,
                                 pose=box_pose)

    # Testing: Open drawer
    # test_open_drawer(drawer_variables[0])

    # Testing: Mueslibox
    # test_muesli(mueslibox_point)

    # test_close_gripper()
    # test_open_gripper()

    #_giskard_wrapper.move_gripper(True)
    #_giskard_wrapper.move_gripper(False)
    # Hand init
    #_giskard_wrapper.set_base_position()

    #_giskard_wrapper.move_gripper(True)
    #_giskard_wrapper.move_gripper(False)

    # Hand init
    # _giskard_wrapper.set_base_position()

    # Grab mueslibox

    muesli_size = [0.04, 0.1, 0.2]
    _giskard_wrapper.grasp_box(box_name=box_name, box_pose=mueslibox_variables[0], testing=True)

    # Place mueslibox
    height = 0.2
    #_giskard_wrapper.place_object(object_name=box_name, goal_pose=mueslibox_variables[3], object_height=height, testing=True)

    #_giskard_wrapper.drive_back(distance=0.5)

    # Drawer

    knob_size = [0.04, 0.01, 0.02]
    # Move to drawer
    #_giskard_wrapper.grasp_box(box_pose=drawer_variables[0], box_size=knob_size)
    #_giskard_wrapper.grasp_box(box_pose=drawer_variables[0], box_z=0.001, mueslibox=False, grasp_vertical=False)

    # Open drawer
    #_giskard_wrapper.move_drawer(knob_pose=drawer_variables[0], direction=drawer_variables[1], distance=drawer_variables[2])
    # Close drawer
    # _giskard_wrapper.move_drawer(knob_pose=drawer_variables[0], direction=drawer_variables[3], distance=drawer_variables[2])

    # Run
    _giskard_wrapper.plan_and_execute()

    # Drawer example
    #show_open_close_drawer()

    # Mueslibox example
    #show_pick_place_mueslibox()
