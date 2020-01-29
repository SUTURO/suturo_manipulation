#! /usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import PlaceAction, PlaceFeedback, PlaceResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import hsrb_interface

class PlaceServer():
    _feedback = PlaceFeedback()
    _result =PlaceResult()
    _root = u'odom'


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        print("PlaceActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("Order recieved. place")
        self._result.error_code = self._result.FAILED

        #
        object_frame_id = goal.object_frame_id

        ## Calculate offset for hand palm link because of grasped object
        ## TODO: Find out if giskard can handle this. if not make it more generic and put it in an extra library
#        goal.goal_pose.pose.position = Point(
#            goal.goal_pose.pose.position.x,
#            goal.goal_pose.pose.position.y,
#            goal.goal_pose.pose.position.z + 0.125
#        )

        #TODO: Move gripper with object to goal
        self._giskard_wrapper.set_cart_goal(self._root, object_frame_id, goal.goal_pose)
        self._giskard_wrapper.plan_and_execute(wait=False)
        result = self._giskard_wrapper.get_result(rospy.Duration(30))

        # Release gripper
        # Detach object from gripper
        '''
        goal_js ={
            u'hand_l_spring_proximal_joint': 0.7,
            u'hand_r_spring_proximal_joint': 0.7
        }
        if result.error_code == result.SUCCESS:
            self._giskard_wrapper.set_joint_goal(goal_js)
            result = self._giskard_wrapper.plan_and_execute()
            self._giskard_wrapper.detach_object(object_frame_id)
            self._giskard_wrapper.remove_object(object_frame_id)
            
        '''
        self._gripper.command(1.2)
        self._giskard_wrapper.detach_object(object_frame_id)
        #self._giskard_wrapper.remove_object(object_frame_id)

        ##TODO: load default pose from json file
        '''
        goal_js = {
            u'head_pan_joint': 0,
            u'head_tilt_joint': 0,
            u'arm_lift_joint': 0,
            u'arm_flex_joint': 0,
            u'arm_roll_joint': 1.4,
            u'wrist_flex_joint': -1.5,
            u'wrist_roll_joint': 0.14,

            u'hand_l_spring_proximal_joint': 0.1,
            u'hand_r_spring_proximal_joint': 0.1
        }
        

        if result.error_code == result.SUCCESS:
            self._giskard_wrapper.set_joint_goal(goal_js)
            result = self._giskard_wrapper.plan_and_execute()

        #TODO: send feedback periodically?
        '''
        self._whole_body.move_to_neutral()



#        self._feedback.tf_gripper_to_goal = tfwrapper.lookup_transform(goal.goal_pose.header.frame_id, u'hand_palm_link')

#        self._as.publish_feedback(self._feedback)

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer(rospy.get_name())
    rospy.spin()
