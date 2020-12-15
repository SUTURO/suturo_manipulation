#! /usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import PlaceAction, PlaceFeedback, PlaceResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import hsrb_interface
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math

class PlaceServer():
    # Add to YAML
    FRONT_ROTATION_QUATERNION = [0.7, 0.0, 0.7, 0.0]
    TOP_ROTATION_QUATERNION = [1, 0, 0, 0]

    _feedback = PlaceFeedback()
    _result = PlaceResult()
    _root = u'odom'


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper, queue_size=10)
        print("PlaceActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("Order recieved. place", goal)
        self._result.error_code = self._result.FAILED
        object_frame_id = goal.object_frame_id

        #TODO: Move gripper with object to goal
        #self._giskard_wrapper.set_cart_goal(self._root, object_frame_id, goal.goal_pose)
        #self._giskard_wrapper.plan_and_execute(wait=False)
        #result = self._giskard_wrapper.get_result(rospy.Duration(30))
        place_pose = goal.goal_pose
        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        # place_mode
        base_tip_rotation = None
        step = None
        if goal.place_mode == goal.TOP:
            base_tip_rotation = self.TOP_ROTATION_QUATERNION

        if goal.place_mode == goal.FRONT:
            base_tip_rotation = self.FRONT_ROTATION_QUATERNION

        # Move the robot in goal position.
        self._giskard_wrapper.avoid_all_collisions(distance=0.03)
        giskard_result = self._giskard_wrapper.set_cart_goal_wstep(self._root,
                                                                   u'hand_palm_link',
                                                                   place_pose,
                                                                   base_tip_rotation=base_tip_rotation,
                                                                   base_transform=hsr_transform)

        if giskard_result and giskard_result.SUCCESS in giskard_result.error_codes:
            # Release gripper
            self._gripper.command(1.2)
            self._giskard_wrapper.detach_object(object_frame_id)

            obj_in_gri = ObjectInGripper()
            obj_in_gri.object_frame_id = object_frame_id
            obj_in_gri.goal_pose = goal.goal_pose
            obj_in_gri.mode = ObjectInGripper.PLACED

            self._obj_in_gripper_pub.publish(obj_in_gri)

            p_temp = PoseStamped()
            p_temp.header.frame_id = "map"
            p_temp.header.stamp = rospy.Time.now()
            p_temp.pose.position = Point(hsr_transform.transform.translation.x, hsr_transform.transform.translation.y,
                                         hsr_transform.transform.translation.z)
            p_temp.pose.orientation = hsr_transform.transform.rotation
            self._giskard_wrapper.set_cart_goal_wstep(self._root, u'base_footprint', p_temp)

            ##TODO: load default pose from json file
            self._giskard_wrapper.avoid_all_collisions(distance=0.03)
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/robot_poses/neutral'))
            self._giskard_wrapper.plan_and_execute(True)
            self._result.error_code = self._result.SUCCESS

    #        self._feedback.tf_gripper_to_goal = tfwrapper.lookup_transform(goal.goal_pose.header.frame_id, u'hand_palm_link')
    #        self._as.publish_feedback(self._feedback)

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer(rospy.get_name())
    rospy.spin()
