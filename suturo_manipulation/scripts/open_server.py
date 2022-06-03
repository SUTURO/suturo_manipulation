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
    """
    Action Server, which handles the opening of objects.
    """
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
        """
        Executes the opening of objects.
        :param goal The grasp goal
        :type goal OpenGoal
        """
        openorclose = goal.openorclose
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Opening: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED

        collision_whitelist = []

        # open gripper
        self._manipulator.set_collision(-1)
        self._gripper.set_gripper_joint_position(1.2)
        # move to knob of the object
        self._manipulator.move_to_drawer(u'odom', u'hand_gripper_tool_frame', goal.goal_pose)
        #goal_pose = self.calculate_goal_pose(goal.object_link_name)
        #success &= self._manipulator.grasp_bar(u'odom', u'hand_gripper_tool_frame', goal_pose)

        # closing the gripper
        self._gripper.set_gripper_joint_position(-0.1)

        limit_base_scan = False
        if 'shelf' in goal.object_name:
            limit_base_scan = False

        if 'drawer' in goal.object_name:
            limit_base_scan = False

        # open the drawer/shelf
        if openorclose == 1:

            if 'shelf' in goal.object_name:
                success &= self._manipulator.open(tip_link=u'hand_gripper_tool_frame',
                                                  object_name_prefix=u'iai_kitchen',
                                                  object_link_name=goal.object_name,
                                                  angle_goal=1.4,
                                                  use_limitation=limit_base_scan)


            elif 'drawer' in goal.object_name:
                print("oobject" + goal.object_name)
                success &= self._manipulator.manipulatedrawer(tip_link=u'hand_gripper_tool_frame',
                                                              object_name_prefix=u'iai_kitchen',
                                                              object_link_name=goal.object_name,
                                                              distance_goal=-0.3,
                                                              use_limitation=limit_base_scan)
        #closes the drawer/shelf
        if openorclose == 0:
            if 'shelf' in goal.object_name:
                success &= self._manipulator.open(tip_link=u'hand_gripper_tool_frame',
                                                  object_name_prefix=u'iai_kitchen',
                                                  object_link_name=goal.object_name,
                                                  angle_goal=-1.4,
                                                  use_limitation=limit_base_scan)


            elif 'drawer' in goal.object_name:
                print("oobject" + goal.object_name)
                success &= self._manipulator.manipulatedrawer(tip_link=u'hand_gripper_tool_frame',
                                                              object_name_prefix=u'iai_kitchen',
                                                              object_link_name=goal.object_name,
                                                              distance_goal=0.3,
                                                              use_limitation=limit_base_scan)





        # opens the gripper again
        self._gripper.set_gripper_joint_position(1.2)


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
        if 'shelf' in object_name:
            rotation_matrix = np.array(rospy.get_param(u'/manipulation/gripper_opening_matrices/shelf'))
        elif 'drawer' in object_name:
            rotation_matrix = np.array(rospy.get_param(u'/manipulation/gripper_opening_matrices/drawer-link'))
        elif 'door' in object_name:
            rotation_matrix = np.array(rospy.get_param(u'/manipulation/gripper_opening_matrices/door'))
        goal_pose.pose.orientation = Quaternion(*quaternion_from_matrix(rotation_matrix))
        return goal_pose


if __name__ == '__main__':
    rospy.init_node('open_server')
    server = OpenServer(rospy.get_name())
    rospy.spin()
