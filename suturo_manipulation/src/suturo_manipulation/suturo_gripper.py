#!/usr/bin/env python
import rospy
import actionlib
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class SuturoGripper:
    """
    Handles the movement of the gripper.
    """

    def __init__(self, apply_force_action_server, follow_joint_trajectory_server):
        self._gripper_apply_force_client = actionlib.SimpleActionClient(apply_force_action_server,
                                                                        GripperApplyEffortAction)
        self._gripper_apply_force_client.wait_for_server()

        self._gripper_controller = actionlib.SimpleActionClient(follow_joint_trajectory_server,
                                                                FollowJointTrajectoryAction)
        self._gripper_controller.wait_for_server()

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

    def set_gripper_joint_position(self, position):
        """
        Sets the gripper joint to the given  position
        :param position: goal position of the joint -0.105 to 1.239 rad
        :return: error_code of FollowJointTrajectoryResult
        """
        pos = max(min(1.239, position), -0.105)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [u'hand_motor_joint']
        p = JointTrajectoryPoint()
        p.positions = [pos]
        p.velocities = [0]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(3)
        goal.trajectory.points = [p]
        self._gripper_controller.send_goal(goal)
