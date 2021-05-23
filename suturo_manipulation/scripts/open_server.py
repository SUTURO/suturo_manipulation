#!/usr/bin/env python
import math
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Quaternion, PoseStamped
from manipulation_msgs.msg import OpenAction, OpenFeedback, OpenResult
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper
from suturo_manipulation.manipulator import Manipulator
from giskardpy.python_interface import GiskardWrapper
from tf.transformations import euler_from_quaternion, quaternion_from_matrix


class OpenServer:
    _feedback = OpenFeedback()
    _result = OpenResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, OpenAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                                follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
        self._manipulator = Manipulator(collision_distance=0.01)
        self._giskard_wrapper = GiskardWrapper()
        self._as.start()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def execute_cb(self, goal):
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Opening: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED

        collision_whitelist = []

        # get current robot_pose
        robot_pose = tfwrapper.lookup_pose('map', 'base_footprint')
        # open gripper
        self._gripper.set_gripper_joint_position(1.2)
        # move to handle of the object
        goal_pose = self.calculate_goal_pose(goal.object_link_name)
        success &= self._manipulator.grasp_bar(u'odom', u'hand_gripper_tool_frame', goal.object_link_name,
                                               goal.object_link_name, goal_pose)
        # closing the gripper
        self._gripper.set_gripper_joint_position(-0.1)

        goal_angle = 1.4
        limit_base_scan = True
        if 'shelf' in goal.object_name:
            goal_angle = -1.4
            limit_base_scan = False
        # opens the door
        success &= self._manipulator.open(u'hand_gripper_tool_frame', goal.object_name, goal_angle, limit_base_scan)
        # opens the gripper again
        self._gripper.set_gripper_joint_position(1.2)

        # return to initial pose
        if success:
            success &= self._manipulator.take_robot_pose(rospy.get_param(u'/manipulation/robot_poses/transport'))
            self._gripper.close_gripper_force()
            success &= self._manipulator.move_to_goal(root_link=self._root,
                                                      tip_link=u'base_footprint',
                                                      goal_pose=robot_pose)
        if success:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)

    def calculate_goal_pose(self, object_name):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = object_name
        rotation_matrix = np.eye(4)
        if 'shelf' in object_name:
            rotation_matrix = np.array([[0, -1, 0, 0],
                                        [0, 0, -1, 0],
                                        [1, 0, 0, 0],
                                        [0, 0, 0, 1]])
        elif 'door' in object_name:
            rotation_matrix = np.array([[1, 0, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]])
        goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix(rotation_matrix))
        return goal_pose

if __name__ == '__main__':
    rospy.init_node('open_server')
    server = OpenServer(rospy.get_name())
    rospy.spin()
