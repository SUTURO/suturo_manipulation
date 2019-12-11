#! /usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import PlaceAction, PlaceFeedback, PlaceResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion

class PlaceServer():
    _feedback = PlaceFeedback()
    _result =PlaceResult()
    _root = u'base_footprint'


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        print("PlaceActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("Order recieved")
        self._result.error_code = self._result.FAILED


        pose = PoseStamped()
        pose.header.frame_id = u'hand_palm_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(0,0,0)
        pose.pose.orientation = Quaternion(0,1,0,1)

        #TODO: Get object dimension
#        self._giskard_wrapper.add_cylinder(name=u'box', size=(0.25,0.07,0.07), pose=pose)
        #TODO: Add object ro gripper
#        self._giskard_wrapper.attach_object(name=u'box', link_frame_id=u'hand_palm_link')
        #TODO: Move gripper with object to goal
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', goal.goal_pose)
        self._giskard_wrapper.plan_and_execute(wait=False)
        result = self._giskard_wrapper.get_result(rospy.Duration(60))

        #TODO: Release gripper
#        if result.error_code == result.SUCCESS:
#            self._giskard_wrapper.set_joint_goal({''})

#        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', goal.goal_pose)
#        self._giskard_wrapper.plan_and_execute(wait=False)

        #TODO: send feedback periodically?


        #TODO: Detach object from gripper
        self._giskard_wrapper.remove_object(u'box')

#        self._feedback.tf_gripper_to_goal = tfwrapper.lookup_transform(goal.goal_pose.header.frame_id, u'hand_palm_link')

#        self._as.publish_feedback(self._feedback)

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer(rospy.get_name())
    rospy.spin()