#!/usr/bin/env python

import math
import sys

from geometry_msgs.msg import WrenchStamped

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
import hsrb_interface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
from listener import Listener
from sensor_msgs.msg import JointState
from force_checking import ForceSensorCapture
from giskardpy import tfwrapper


class GraspsObjectServer:
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper)
        print("GraspsActionServer greets its masters and is waiting for orders")

    def get_current_joint_state(self, joint):
        """
        sets the value of every slider to its corresponding current joint state
        """
        msg = rospy.wait_for_message(u'joint_states', JointState)
        for i in range(len(msg.name)):
            if msg.name[i] == joint:
                return  msg.position[i]

    def calculateWayPoint2D(self, target, origin, distance):
        x = target.x - origin.x
        y = target.y - origin.y

        alpha = math.atan(y / x)
        dx = math.cos(alpha) * distance
        dy = math.sin(alpha) * distance

        return Point(target.x - dx, target.y - dy, target.z)

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

        self._gripper.command(1.2)

        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        q1 = [hsr_transform.transform.rotation.x, hsr_transform.transform.rotation.y, hsr_transform.transform.rotation.z,
              hsr_transform.transform.rotation.w]
        # grasp_mode
        if goal.grasp_mode == goal.FRONT:
            pose.pose.position.z += goal.object_size.z / 2.0
            rospy.loginfo("ACTUAL FRONT GRASPING HEIGHT: "+ str(pose))
            pose.pose.position = self.calculateWayPoint2D(pose.pose.position, hsr_transform.transform.translation, goal.object_size.x / 2)
            q2 = [0.7, 0.0, 0.7, 0.0]  # Quaternion for rotation to grasp from front relative to map for hand_palm_link
        elif goal.grasp_mode == goal.TOP:
            pose.pose.position.z += goal.object_size.z
            rospy.loginfo("ACTUAL TOP GRASPING HEIGHT: " + str(pose))
            q2 = [1, 0, 0, 0]  # Quaternion for rotation to grasp from above relative to map for hand_palm_link           

        if goal.grasp_mode != goal.FREE:  
            q3 = quaternion_multiply(q1, q2)
            pose.pose.orientation = Quaternion(q3[0], q3[1], q3[2], q3[3])

        if goal.grasp_mode == goal.FRONT:
            pose_step = PoseStamped()
            pose_step.header.frame_id = "map"
            pose_step.header.stamp = rospy.Time.now()
            pose_step.pose.position = self.calculateWayPoint2D(pose.pose.position, hsr_transform.transform.translation, 0.1)
            pose_step.pose.orientation = pose.pose.orientation
            self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose_step)
            self._giskard_wrapper.plan_and_execute(wait=True)

        pose.header.stamp = rospy.Time.now()
        # Move the robot in goal position.        
        self._giskard_wrapper.avoid_all_collisions(distance=0.3)
        self._giskard_wrapper.allow_collision(body_b=goal.object_frame_id)

        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)
        self._giskard_wrapper.plan_and_execute(wait=True)

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result and result.SUCCESS in result.error_codes:

            # Close the Gripper
            self._gripper.apply_force(1.0)

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
            self._giskard_wrapper.set_cart_goal(self._root, u'base_footprint', p_temp)
            self._giskard_wrapper.plan_and_execute(wait=True)

            self._giskard_wrapper.avoid_all_collisions(distance=0.02)
            arm_lift_joint = self.get_current_joint_state(u'arm_lift_joint')
            self._giskard_wrapper.set_joint_goal({
                u'head_pan_joint': 0,
                u'head_tilt_joint': 0,
                u'arm_lift_joint': arm_lift_joint + 0.1,
                u'arm_flex_joint': 0,
                u'arm_roll_joint': 1.4,
                u'wrist_flex_joint': -1.5,
                u'wrist_roll_joint': 0.14})
            self._giskard_wrapper.plan_and_execute(wait=True)
            
            self._giskard_wrapper.avoid_all_collisions(distance=0.02)
            self._giskard_wrapper.set_joint_goal({
                u'head_pan_joint': 0,
                u'head_tilt_joint': 0,
                u'arm_lift_joint': 0,
                u'arm_flex_joint': 0,
                u'arm_roll_joint': 1.4,
                u'wrist_flex_joint': -1.5,
                u'wrist_roll_joint': 0.14})
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
        listener = Listener()
        listener.set_topic_and_typMEssage("/hsrb/joint_states", JointState)
        listener.listen_topic_with_sensor_msg()
        current_hand_motor_value = listener.get_value_from_sensor_msg("hand_motor_joint")
        print("Current hand motor joint is:")
        print current_hand_motor_value
        print("Is object in gripper ?")
        print current_hand_motor_value >= -0.8
        return current_hand_motor_value >= -0.8


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()
