#!/usr/bin/env python

import math
import sys

from geometry_msgs.msg import WrenchStamped

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point
from giskardpy import tfwrapper
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
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper, queue_size=10)
        self._as.start()
        print("GraspsActionServer greets its masters and is waiting for orders")

    def execute_cb(self, goal):

        print("Recieve Order. grasp", goal)
        self._giskard_wrapper.interrupt()

        # Set initial result value.
        self._result.error_code = self._result.FAILED
        goal_pose = goal.goal_pose

        #open gripper
        self.set_gripper_joint_position(1.2)
        in_gripper = False

        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        base_tip_rotation = None
        step = None
        if goal.grasp_mode == goal.FRONT:
            goal_pose.pose.position.z += goal.object_size.z / 2.0
            base_tip_rotation = self.FRONT_ROTATION_QUATERNION
            step = 0.1

        elif goal.grasp_mode == goal.TOP:
            goal_pose.pose.position.z += goal.object_size.z
            rospy.loginfo("ACTUAL TOP GRASPING HEIGHT: " + str(goal_pose))
            base_tip_rotation = self.TOP_ROTATION_QUATERNION

        # Move the robot in goal position.        
        self._giskard_wrapper.avoid_all_collisions(distance=0.3)
        self._giskard_wrapper.allow_collision(body_b=goal.object_frame_id)
        self._giskard_wrapper.set_cart_goal_wstep(self._root,
                                                  u'hand_palm_link',
                                                  goal_pose,
                                                  base_tip_rotation=base_tip_rotation,
                                                  step=step,
                                                  base_transform=hsr_transform)
        result = self._giskard_wrapper.get_result(rospy.Duration(60))

        if result and result.SUCCESS in result.error_codes:
            # Close the Gripper
            self.close_gripper_force(0.8)
            in_gripper = self.object_in_gripper()

            if in_gripper:
                # Attach object
                self._giskard_wrapper.attach_object(goal.object_frame_id, u'hand_palm_link')

                obj_in_gri = ObjectInGripper()
                obj_in_gri.object_frame_id = goal.object_frame_id
                obj_in_gri.goal_pose = goal.goal_pose
                obj_in_gri.mode = ObjectInGripper.GRASPED
                self._obj_in_gripper_pub.publish(obj_in_gri)

            # Pose to move with an attached Object
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.header.stamp = rospy.Time.now()
            start_pose.pose.position = Point(hsr_transform.transform.translation.x, hsr_transform.transform.translation.y, hsr_transform.transform.translation.z)
            start_pose.pose.orientation = hsr_transform.transform.rotation
            self._giskard_wrapper.set_cart_goal_wstep(self._root, u'base_footprint', start_pose)

            self._giskard_wrapper.avoid_all_collisions(distance=0.03)
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
            result = joint_states[u'hand_motor_joint'] > -0.8
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
