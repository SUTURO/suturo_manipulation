#!/usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
import hsrb_interface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply

import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
#from listener import Listener
from sensor_msgs.msg import JointState


class GraspsObjectServer:
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        print("GraspsActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):

        print("Recieve Order. grasp")

        self._giskard_wrapper.interrupt()

        grasped_object = u'grasped_object'
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position

        # Remove old object, with same name like the new object.
        self._giskard_wrapper.detach_object(grasped_object)
        self._giskard_wrapper.remove_object(grasped_object)

        # Set initial result value.
        self._result.error_code = self._result.FAILED

        '''
        # Open the gripper.
        self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.7,
                                              u'hand_r_spring_proximal_joint': 0.7})
        self._giskard_wrapper.plan_and_execute(wait=True)
        '''

        self._gripper.command(0.0)
        self._gripper.command(1.2)

        hsr_pose = tfwrapper.lookup_transform('map', 'base_footprint')
        q1 = [hsr_pose.transform.rotation.x, hsr_pose.transform.rotation.y, hsr_pose.transform.rotation.z, hsr_pose.transform.rotation.w]
        #q2 = [0.7, 0.0, 0.7, 0.0] # Quaternion for rotation to grap from front relative to map for hand_palm_link
        q2 = [1, 0, 0, 0] #Quaternion for rotation of grap from above relative to map for hand_palm_link

        q3 = quaternion_multiply(q1, q2)

        pose.pose.orientation = Quaternion(q3[0], q3[1], q3[2], q3[3])

        #quat1 = [goal.goal_pose.pose.orientation.x, goal.goal_pose.pose.orientation.y, goal.goal_pose.pose.orientation.z, goal.goal_pose.pose.orientation.w]

        #orientation = quaternion_multiply(quat1, quaternion_from_euler(0, 1.57, 0))
        #pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

        # Move the robot in goal position.
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)
        self._giskard_wrapper.plan_and_execute(wait=False)

        # TODO: send feedback periodically?

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result.error_code == result.SUCCESS:

            # Close the Gripper
            '''
            self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.2,
                                                  u'hand_r_spring_proximal_joint': 0.2})
            self._giskard_wrapper.plan_and_execute()
            '''
            self._gripper.set_distance(0.05)

            # Attach object
            self._giskard_wrapper.add_cylinder(name=grasped_object, size=(goal.object_size.x, goal.object_size.y), pose=goal.goal_pose)
            self._giskard_wrapper.attach_object(name=grasped_object, link_frame_id=u'hand_palm_link')

            # Pose to move with an attatched Object
            neutral_js = {
                u'head_pan_joint': 0,
                u'head_tilt_joint': 0,
                u'arm_lift_joint': 0,
                u'arm_flex_joint': 0,
                u'arm_roll_joint': 1.4,
                u'wrist_flex_joint': -1.5,
                u'wrist_roll_joint': 0.14}

            self._giskard_wrapper.set_joint_goal(neutral_js)
            self._giskard_wrapper.plan_and_execute()

            result = self._giskard_wrapper.get_result()

        # self._feedback.tf_gripper_to_object = tfwrapper.lookup_transform(goal.object_frame_id, u'hand_palm_link')
        # self._feedback.gripper_joint_state = u'hand_l_spring_proximal_joint' + u'hand_r_spring_proximal_joint'

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()
