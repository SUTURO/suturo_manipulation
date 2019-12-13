#!/usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspActionFeedback, GraspActionResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
#import hsrb_interface


class GraspsObjectServer:
    _feedback = GraspActionFeedback
    _result = GraspActionResult
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        print("GraspsActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):
        print("try to do something")
        self._result.error_code = self.result.FAILED

        self._giskard_wrapper.add_cylinder(u'cylinder', (0.25, 0.07), u'map', (0, 0, 0), (0, 0, 0, 1))

        # Close the Gripper
        self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint: 0.4, u', u'hand_r_spring_proximal_joint: 0.4'})
        self._giskard_wrapper.plan_and_execute()

        # Attach object
        self._giskard_wrapper.attach_box(u'cylinder')

        # Pose to move with an attatched Object
        self._giskard_wrapper.set_joint_goal({u'arm_roll_joint': 1.57, u'wrist_flex_joint': -1.37})

        result = self._giskard_wrapper.get_result()

        self._feedback.tf_gripper_to_object = tfwrapper.lookup_transform(goal.object_frame_id, u'hand_palm_link')
        self._feedback.gripper_joint_state = u'hand_l_spring_proximal_joint' + u'hand_r_spring_proximal_joint'

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


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    server.place_object()
    #server.hardcode_grasp()
    rospy.spin()
