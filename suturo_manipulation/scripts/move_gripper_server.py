#! /usr/bin/env python

import rospy
import actionlib
from manipulation_msgs.msg import MoveGripperAction, MoveGripperFeedback, MoveGripperResult
from giskardpy.python_interface import GiskardWrapper


class MoveGripperServer:
    _feedback = MoveGripperFeedback()
    _result = MoveGripperResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveGripperAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._giskard_wrapper = GiskardWrapper()
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Move gripper: {}".format(goal))
        self._result.error_code = self._result.FAILED
        self._giskard_wrapper.interrupt()
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', goal.goal_pose)
        self._giskard_wrapper.plan_and_execute(wait=False)
        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result and result.SUCCESS in result.error_code:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move_gripper_server')
    server = MoveGripperServer(rospy.get_name())
    rospy.spin()    
