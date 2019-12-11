#! /usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import MoveGripperAction, MoveGripperFeedback, MoveGripperResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import TransformStamped

class MoveGripperServer():
    _feedback = MoveGripperFeedback()
    _result = MoveGripperResult()
    _root = u'base_link'


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveGripperAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        print("MoveGripperActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("Order recieved")
        self._result.error_code = self._result.FAILED

        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', goal.goal_pose)
        self._giskard_wrapper.plan_and_execute(wait=False)

        #TODO: send feedback periodically?

        result = self._giskard_wrapper.get_result(rospy.Duration(30))

        self._feedback.tf_gripper_to_goal = tfwrapper.lookup_transform(goal.goal_pose.header.frame_id, u'hand_palm_link')

        self._as.publish_feedback(self._feedback)

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move_gripper_server')
    server = MoveGripperServer(rospy.get_name())
    rospy.spin()    