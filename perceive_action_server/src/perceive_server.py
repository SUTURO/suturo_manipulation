#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
from manipulation_action_msgs.msg import PerceiveAction, PerceiveFeedback, PerceiveResult
from giskardpy.python_interface import GiskardWrapper

class PerceiveServer():
    _feedback = PerceiveFeedback()
    _result = PerceiveResult()
    _goal = PerceiveAction()


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PerceiveAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        print("PerceiveActionServer greets its masters and is waiting for orders")

    def execute_cb(self, goal):
        print("Order recieved: perceive")
        self._result.error_code = self._result.FAILED

	#raise/lower torso to the given joint state
        if goal.perceive_mode == goal.PERCEIVE_ARM_LOW:
	    #PERCEIVE_ARM_LOW
            self._whole_body.move_to_joint_positions({u'head_pan_joint': 0.0, u'arm_flex_joint': -1.5, u'arm_roll_joint': 0.0, u'wrist_flex_joint': -1.9})
            self._giskard_wrapper.set_joint_goal({u'arm_lift_joint': goal.torso_joint_state})
            self._giskard_wrapper.plan_and_execute(wait=False) 
        elif goal.perceive_mode == goal.PERCEIVE_SIDE:
	    #PERCEIVE_SIDE
            self._whole_body.move_to_joint_positions({u'arm_flex_joint': -0.5, u'arm_roll_joint': 0.0})
            self._giskard_wrapper.set_joint_goal({u'arm_lift_joint': goal.torso_joint_state}) 
            self._giskard_wrapper.plan_and_execute(wait=True) #avoid self-collision
            self._whole_body.move_to_joint_positions({u'head_pan_joint': -1.6})
        elif goal.perceive_mode == goal.PERCEIVE_ARM_HIGH:
	    #PERCEIVE_ARM_HIGH
            self._whole_body.move_to_joint_positions({u'head_pan_joint': 0.0, u'arm_flex_joint': 0.0, u'arm_roll_joint': 1.5, u'wrist_flex_joint': -1.9})
            self._giskard_wrapper.set_joint_goal({u'arm_lift_joint': goal.torso_joint_state}) 
            self._giskard_wrapper.plan_and_execute(wait=False)


        result = self._giskard_wrapper.get_result(rospy.Duration(60))

        self._feedback.torso_joint_state = goal.torso_joint_state

        self._as.publish_feedback(self._feedback)

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('perceive_server')
    server = PerceiveServer(rospy.get_name())
    rospy.spin()    
