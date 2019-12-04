#! /usr/bin/env python

import rospy
import actionlib
import move_gripper_action_server.msg
from giskardpy.python_interface import GiskardWrapper

class MoveGripperAction():
    _feedback = move_gripper_action_server.msg.MoveGripperFeedback()
    _result = move_gripper_action_server.msg.MoveGripperResult()
    _giskard_wrapper = GiskardWrapper()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_gripper_action_server.msg.MoveGripperAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("we recieved something")
        #self._giskard_wrapper.set_cart_goal('base_footprint', 'hand_palm_link', goal.goal_pose)
        #self._giskard_wrapper.plan_and_execute()
        self._giskard_wrapper.set_joint_goal({'arm_lift_joint':0.2})
        self._giskard_wrapper.plan_and_execute()

if __name__ == '__main__':
    rospy.init_node('move_gripper_server')
    server = MoveGripperAction(rospy.get_name())
    rospy.spin()    
