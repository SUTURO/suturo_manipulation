#!/usr/bin/env python
import math
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Quaternion, PoseStamped
from manipulation_msgs.msg import CloseAction, CloseActionFeedback, CloseActionResult
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper
from suturo_manipulation.manipulator import Manipulator
from giskardpy.python_interface import GiskardWrapper
from tf.transformations import euler_from_quaternion, quaternion_from_matrix


class CloseServer:
    """
    Action Server, which handles the closing of objects.
    """
    _feedback = CloseActionFeedback()
    _result = CloseActionResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CloseAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                                follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
        self._manipulator = Manipulator(collision_distance=0.01)
        self._giskard_wrapper = GiskardWrapper()
        self._as.start()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def execute_cb(self, goal):
        """
        Executes the closing of objects.
        :param goal The grasp goal
        :type goal GraspGoal
        """
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Closing: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED

        collision_whitelist = []

        # open gripper
        self._gripper.set_gripper_joint_position(1.2)
        # move to handle of the object
        goal_pose = self.calculate_goal_pose(goal.object_link_name)
        success &= self._manipulator.grasp_bar(u'odom', u'hand_gripper_tool_frame', goal_pose)
        # closing the gripper
        self._gripper.set_gripper_joint_position(-0.1)


        if 'drawer' in goal.object_name:
            limit_base_scan = False


        # closes the drawer
        if 'drawer' in goal.object_name:
            print("oobject" + goal.object_name)
            success &= self._manipulator.openclosedrawer(tip_link=u'hand_gripper_tool_frame',
                                                         object_name_prefix=u'iai_kitchen',
                                                         object_link_name=goal.object_name,
                                                         distance_goal=0.3,
                                                         use_limitation=limit_base_scan)



        # closes the gripper again
        self._gripper.set_gripper_joint_position(1.2)

        # return to initial transport pose
        if success:
            success &= self._manipulator.take_robot_pose(rospy.get_param(u'/manipulation/robot_poses/transport'))

        if success:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)

    def calculate_goal_pose(self, object_name):
        """
        Calculates the goal orientation pose depending on the object type.
        :param object_name the object name
        :type object_name string
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = object_name
        rotation_matrix = np.eye(4)
        if 'drawer' in object_name:
            rotation_matrix = np.array(rospy.get_param(u'/manipulation/gripper_opening_matrices/drawer'))
        goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix(rotation_matrix))
        return goal_pose

if __name__ == '__main__':
    rospy.init_node('close_server')
    server = CloseServer(rospy.get_name())
    rospy.spin()