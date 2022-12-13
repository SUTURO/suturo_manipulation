#!/usr/bin/env python

import rospy
import math # for sin and cos
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped

from giskardpy.goals.suturo import MoveHandOutOfSight
from giskardpy.python_interface import GiskardWrapper
import actionlib
import tf
import hsrb_interface
# Giskard benutzen!
from manipulation_msgs.msg import MoveGripperAction, MoveGripperActionFeedback, MoveGripperActionResult, MoveGripperActionGoal


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
        new_goal.pose.orientation.x = math.sin(270/2)
        new_goal.pose.orientation.y = 0
        new_goal.pose.orientation.z = 0
        new_goal.pose.orientation.w = math.cos(270/2)

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

    #mueslibox variables
    mueslibox_center = PointStamped()
    mueslibox_center.header.frame_id = 'map'
    mueslibox_center.point.x = - 0.1
    mueslibox_center.point.y = 1.75
    mueslibox_center.point.z = 0.7

    mueslibox_tip_link = "hand_palm_link"

    box_z_length = 0.1

    mueslibox_vars = (mueslibox_center, mueslibox_tip_link, box_z_length)

    #drawer variables
    drawer_point = PointStamped()
    drawer_point.header.frame_id = 'map'
    drawer_point.point.x = 0.18
    drawer_point.point.y = -0.2
    drawer_point.point.z = 0.282

    drawer_direction = Vector3Stamped()

    drawer_distance = 0.1

    drawer_vars = (drawer_point, drawer_direction, drawer_distance)

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



if __name__ == '__main__':
    rospy.init_node('milestone0_server')

    _giskard_wrapper, mueslibox_variables, drawer_variables = prepare_variables()

    # Testing: Open drawer
    #test_open_drawer(drawer_variables[0])

    # Testing: Mueslibox
    #test_muesli(mueslibox_point)
    #_giskard_wrapper.set_joint_goal({'hand_r_distal_joint': -0.5})  #hand_motor_joint

    #/hsrb/gripper_controller/grasp/goal

    # Hand init
    #_giskard_wrapper.set_hand_out_of_sight()

    # Grab mueslibox
    _giskard_wrapper.grab_box(box_pose=mueslibox_variables[0])
    #_giskard_wrapper.grab_box(box_pose=mueslibox_variables[0], tip_link=mueslibox_variables[1], box_z=mueslibox_variables[2])

    # Open drawer
    #_giskard_wrapper.grab_box(box_pose=drawer_variables[0])

    #_giskard_wrapper.open_drawer(knob_pose=drawer_variables[0], direction=drawer_variables[1], distance=drawer_variables[2])

    # Close drawer
    # _giskard_wrapper.close_drawer()

    # Run
    _giskard_wrapper.plan_and_execute()

