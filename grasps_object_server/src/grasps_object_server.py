#!/usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply


class GraspsObjectServer:
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        print("GraspsActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):

        self._giskard_wrapper.interrupt()

        grasped_object = u'grasped_object'
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position

        self._giskard_wrapper.detach_object(grasped_object)
        self._giskard_wrapper.remove_object(grasped_object)

        self._result.error_code = self._result.FAILED

        self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.7,
                                              u'hand_r_spring_proximal_joint': 0.7})
        self._giskard_wrapper.plan_and_execute(wait=True)

        quat1 = [goal.goal_pose.pose.orientation.x, goal.goal_pose.pose.orientation.y, goal.goal_pose.pose.orientation.z, goal.goal_pose.pose.orientation.w]

        orientation = quaternion_multiply(quat1, quaternion_from_euler(0, 1.57, 0))
        pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)
        self._giskard_wrapper.plan_and_execute(wait=False)

        # TODO: send feedback periodically?

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result.error_code == result.SUCCESS:

            # Close the Gripper
            self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.2,
                                                  u'hand_r_spring_proximal_joint': 0.2})
            self._giskard_wrapper.plan_and_execute()

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

    def hardcode_grasp(self):
        giskard_wrapper = GiskardWrapper()
        print("start grasp")
        self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.2, u'hand_r_spring_proximal_joint': 0.2})
        self._giskard_wrapper.plan_and_execute()

        # Alternative without Giskard
        '''
            robot = hsrb_interface.Robot()
            gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
            gripper.command(1.2, 2.0)
        '''
        print("finish")

    def place_object(self):
        giskard_wrapper = GiskardWrapper()
        GiskardWrapper.add_box(giskard_wrapper, 'box', (1, 1, 0.5), u'map', (1.5, 0, 0.5), (0, 0, 0, 1))
        GiskardWrapper.add_box(giskard_wrapper, 'grasps_obj', (0.1, 0.1, 0.1), u'map', (1.1, 0, 0.8), (0, 0, 0, 1))

    def attach_test(self):
        giskard_wrapper = GiskardWrapper()

        pose = PoseStamped()
        pose.header.frame_id = u'hand_palm_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(0, 0, 0)
        pose.pose.orientation = Quaternion(0, 0.7, 0, 0.7)

        giskard_wrapper.add_cylinder(name=u'box', size=(0.25, 0.07), pose=pose)
        # TODO: Add object ro gripper
        giskard_wrapper.attach_object(name=u'box', link_frame_id=u'hand_palm_link')


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    #server.place_object()
    #server.hardcode_grasp()
    #server.attach_test()
    rospy.spin()
