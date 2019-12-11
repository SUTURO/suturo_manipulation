#!/usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspActionFeedback, GraspActionResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper


class GraspsObjectServer:
    _feedback = GraspActionFeedback
    _result = GraspActionResult
    _root = u'base_link'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, self.execute_cb, False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper

    def execute_cb(self, goal):
        print("try to do something")
        self._result.error_code = self.result.FAILED

        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', goal.object_frame_id)
        self._giskard_wrapper.plan_and_execute(wait=False)

        result = self._giskard_wrapper.get_result()

        #self._feedback.tf_gripper_to_object =
        #self._feedback.gripper_joint_state =

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)

        #TODO Attach object
        #self._giskard_wrapper.attach_box()


    def place_object(self):
        giskard_wrapper = GiskardWrapper()
        GiskardWrapper.add_box(giskard_wrapper, 'box', (1, 1, 0.5), u'map', (1.5, 0, 0.5), (0, 0, 0, 1))
        GiskardWrapper.add_box(giskard_wrapper, 'grasps_obj', (0.1, 0.1, 0.1), u'map', (1.1, 0, 0.8), (0, 0, 0, 1))


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    server.place_object()
    rospy.spin()
