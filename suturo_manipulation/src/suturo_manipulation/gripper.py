#!/usr/bin/env python
import rospy
import actionlib
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from manipulation_msgs.msg import ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from giskardpy.utils import normalize_quaternion_msg


class Gripper:
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
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper, queue_size=10)
        self._giskard_wrapper = GiskardWrapper()

    def publish_object_in_gripper(self, object_frame_id, pose_stamped, mode):
        """
        publishes the object in gripper state
        :param object_frame_id: name of the object
        :param pose: pose the object was taken from/placed at
        :param mode: placed or grasped
        :return:
        """
        rospy.loginfo("Publishing: object_in_gripper: Name: {}; Pose: {}; Mode: {}".format(object_frame_id, pose_stamped, mode))
        orientation = normalize_quaternion_msg(pose_stamped.pose.orientation)
        obj_in_gri = ObjectInGripper()
        obj_in_gri.object_frame_id = object_frame_id
        obj_in_gri.goal_pose = pose_stamped
        obj_in_gri.goal_pose.pose.orientation = orientation
        obj_in_gri.mode = mode
        self._obj_in_gripper_pub.publish(obj_in_gri)

    def object_in_gripper(self):
        """
        This method checks if an object is in the gripper
            :return: false or true, boolean
        """
        joint_states = self._giskard_wrapper.get_joint_states(u'/hsrb/joint_states')
        if joint_states.has_key(u'hand_motor_joint'):
            rospy.loginfo("hand_motor_joint: {}".format(joint_states[u'hand_motor_joint']))
            result = joint_states[u'hand_motor_joint'] > 0.0
            rospy.loginfo("object_in_gripper: {}".format(result))
            return result
        else:
            rospy.logerr("object_in_gripper: unable to hand_motor_joint state defaulting to false!")
        return False

    def close_gripper_force(self, force=0.8):
        """
        Closes the gripper with the given force.
        :param force: force to grasp with should be between 0.2 and 0.8 (N)
        :return: applied effort
        """
        rospy.loginfo("Closing gripper with force: {}".format(force))
        f = max(min(0.8, force), 0.2)
        goal = GripperApplyEffortGoal()
        goal.effort = f
        self._gripper_apply_force_client.send_goal(goal)
        if self._gripper_apply_force_client.wait_for_result(rospy.Duration(secs=5)):
            result = self._gripper_apply_force_client.get_result()
            rospy.loginfo("close_gripper: force {}".format(result.effort))
        else:
            rospy.logwarn("Close Gripper Server timed out.")

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
        self._gripper_controller.wait_for_result()
        result = self._gripper_controller.get_result()
        rospy.loginfo("set_gripper_postion: exited with error code: {}".format(result.error_code))
        return result.error_code
