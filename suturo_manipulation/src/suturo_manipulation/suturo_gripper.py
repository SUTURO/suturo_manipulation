#!/usr/bin/env python
import rospy
import actionlib
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from manipulation_msgs.msg import ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from giskardpy.utils.tfwrapper import normalize_quaternion_msg


class SuturoGripper:
    """
    Handles the movement of the gripper.
    """

    def __init__(self, apply_force_action_server, follow_joint_trajectory_server):
        self._gripper_apply_force_client = actionlib.SimpleActionClient(apply_force_action_server,
                                                                        GripperApplyEffortAction)
        self._gripper_apply_force_client.wait_for_server()

    def close_gripper_force(self, force=0.8):
        """
        Closes the gripper with the given force.
        :param force: force to grasp with should be between 0.2 and 0.8 (N)
        :return: applied effort
        """
        rospy.loginfo("Closing gripper with force: {}".format(force))
        f = force # max(min(0.8, force), 0.2)
        goal = GripperApplyEffortGoal()
        goal.effort = f
        self._gripper_apply_force_client.send_goal(goal)
        if self._gripper_apply_force_client.wait_for_result(rospy.Duration(secs=5)):
            result = self._gripper_apply_force_client.get_result()
            rospy.loginfo("close_gripper: force {}".format(result.effort))
        else:
            rospy.logwarn("Close Gripper Server timed out.")

