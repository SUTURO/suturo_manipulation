#!/usr/bin/env python

import math
import sys

from geometry_msgs.msg import WrenchStamped

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
from listener import Listener
from sensor_msgs.msg import JointState
from force_checking import ForceSensorCapture

from giskardpy import tfwrapper, utils
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class GraspsObjectServer:
    # Add to YAML
    FRONT_ROTATION_QUATERNION = [0.7, 0.0, 0.7, 0.0]
    TOP_ROTATION_QUATERNION = [1, 0, 0, 0]

    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._giskard_wrapper = GiskardWrapper()
        self._gripper_apply_force_client = actionlib.SimpleActionClient(u'/hsrb/gripper_controller/apply_force',
                                                                        GripperApplyEffortAction)
        self._gripper_apply_force_client.wait_for_server()
        self._gripper_controller = actionlib.SimpleActionClient(u'/hsrb/gripper_controller/follow_joint_trajectory',
                                                                FollowJointTrajectoryAction)
        self._gripper_controller.wait_for_server()
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper)
        self._as.start()
        print("GraspsActionServer greets its masters and is waiting for orders")

    def execute_cb(self, goal):

        print("Recieve Order. grasp", goal)
        self._giskard_wrapper.interrupt()
        in_gripper = False
        # grasped_object = u'grasped_object'
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position

        # Set initial result value.
        self._result.error_code = self._result.FAILED

        self.set_gripper_joint_position(1.2)
        
        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        # grasp_mode
        root_tip_orientation = None
        step = None
        if goal.grasp_mode == goal.FRONT:
            pose.pose.position.z += goal.object_size.z / 2.0
            root_tip_orientation = self.FRONT_ROTATION_QUATERNION
            step = 0.1

        elif goal.grasp_mode == goal.TOP:
            pose.pose.position.z += goal.object_size.z
            rospy.loginfo("ACTUAL TOP GRASPING HEIGHT: " + str(pose))
            root_tip_orientation = self.TOP_ROTATION_QUATERNION

        pose.header.stamp = rospy.Time.now()
        # Move the robot in goal position.        
        # self._giskard_wrapper.avoid_all_collisions(distance=0.3)
        self._giskard_wrapper.allow_all_collisions()
        self._giskard_wrapper.allow_collision(body_b=goal.object_frame_id)


        self._giskard_wrapper.set_cart_goal_wstep(self._root, u'hand_palm_link', pose, root_tip_orientation, step=step, hsr_transform=hsr_transform)
		self._giskard_wrapper.plan_and_execute(wait=True)

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result and result.SUCCESS in result.error_codes:

            # Close the Gripper
            self.close_gripper_force(0.8)

            if self.object_in_gripper():
                # Attach object TODO: Add again once we disable collision for object to grasp | enable after grasp
                self._giskard_wrapper.attach_object(goal.object_frame_id, u'hand_palm_link')

                obj_in_gri = ObjectInGripper()
                obj_in_gri.object_frame_id = goal.object_frame_id
                obj_in_gri.goal_pose = goal.goal_pose
                obj_in_gri.mode = ObjectInGripper.GRASPED
                self._obj_in_gripper_pub.publish(obj_in_gri)

                in_gripper = True

            # Pose to move with an attached Object

            p_temp = PoseStamped()
            p_temp.header.frame_id = "map"
            p_temp.header.stamp = rospy.Time.now()
            p_temp.pose.position = Point(hsr_transform.transform.translation.x, hsr_transform.transform.translation.y, hsr_transform.transform.translation.z)
            p_temp.pose.orientation = hsr_transform.transform.rotation
            self._giskard_wrapper.set_cart_goal_wstep(self._root, u'base_footprint', p_temp)

            self._giskard_wrapper.plan_and_execute(wait=True)

            # self._giskard_wrapper.avoid_all_collisions(distance=0.02)
            self._giskard_wrapper.allow_all_collisions()
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/robot_poses/transport'))
            self._giskard_wrapper.plan_and_execute(wait=True)

            result_giskard = self._giskard_wrapper.get_result()

            if in_gripper and result_giskard and result_giskard.SUCCESS in result_giskard.error_codes:
                self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)

    def object_in_gripper(self):
        """
        This method checks if an object is in the gripper
            :return: false or true, boolean
        """
        joint_states = self._giskard_wrapper.get_joint_states(u'/hsrb/joint_states')
        if joint_states.has_key(u'hand_motor_joint'):
            result = joint_states[u'hand_motor_joint'] <= -0.8
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
        f = max(min(0.8, force), 0.2)
        goal = GripperApplyEffortGoal()
        goal.effort = f
        self._gripper_apply_force_client.send_goal(goal)
        self._gripper_apply_force_client.wait_for_result()
        result = self._gripper_apply_force_client.get_result()
        rospy.loginfo("close_gripper: force {}".format(result.effort))

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
        rospy.loginfo("set_gripper_postion: exited with error code: ".format(result.error_code))
        return result.error_code


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()
