#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
from manipulation_action_msgs.msg import TakePoseAction, TakePoseFeedback, TakePoseResult
from giskardpy.python_interface import GiskardWrapper

class TakePoseServer():
    _feedback = TakePoseFeedback()
    _result = TakePoseResult()
    _goal = TakePoseAction()


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, TakePoseAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        print("PerceiveActionServer greets its masters and is waiting for orders")

    def execute_cb(self, goal):
        print("Order recieved: perceive", goal)
        self._result.error_code = self._result.FAILED

        if goal.pose_mode == goal.NEUTRAL:
            self._giskard_wrapper.set_joint_goal({
             u'head_pan_joint': 0.0,
             u'head_tilt_joint': 0.0,
             u'arm_lift_joint': 0.0,
             u'arm_flex_joint': 0.0,
             u'arm_roll_joint': 1.4,
             u'wrist_flex_joint': -1.5,
             u'wrist_roll_joint': 0.14
        })
        elif goal.pose_mode == goal.LOOK_HIGH:
	    #TABLE_BIG
            self._giskard_wrapper.set_joint_goal({
             u'head_pan_joint': -1.54,
             u'head_tilt_joint': -0.22,
             u'arm_lift_joint': 0.43,
             u'arm_flex_joint': -0.5,
             u'arm_roll_joint': -1.8,
             u'wrist_flex_joint': -1.57,
             u'wrist_roll_joint': 0.0
        })
        elif goal.pose_mode == goal.LOOK_LOW:
	    #TABLE_SMALL
            self._giskard_wrapper.set_joint_goal({
             u'head_pan_joint': -1.54,
             u'head_tilt_joint': -0.46,
             u'arm_lift_joint': 0.0,
             u'arm_flex_joint': -0.5,
             u'arm_roll_joint': -1.8,
             u'wrist_flex_joint': -1.57,
             u'wrist_roll_joint': 0.0
        })
        else:
            self._giskard_wrapper.set_joint_goal({})
            print("Mode not implemented yet!")


        self._giskard_wrapper.plan_and_execute(wait=False)

        result = self._giskard_wrapper.get_result(rospy.Duration(60))

        self._feedback.torso_joint_state = 0.0

        self._as.publish_feedback(self._feedback)

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('perceive_server')
    server = TakePoseServer(rospy.get_name())
    rospy.spin()