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
    _feedback = PlaceFeedback()
    _result = PlaceResult()
    _root = u'odom'


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self._obj_in_gripper_pub = rospy.Publisher("object_in_gripper", ObjectInGripper)
        print("PlaceActionServer greats its masters and is waiting for orders")

    def calculateWayPoint2D(self, target, origin, distance):
        x = target.x - origin.x
        y = target.y - origin.y

        alpha = math.atan(y / x)
        dx = math.cos(alpha) * distance
        dy = math.sin(alpha) * distance

        return Point(target.x - dx, target.y - dy, target.z)

    def execute_cb(self, goal):
        ## Integrate giskard here
        print("Order recieved. place", goal)
        self._result.error_code = self._result.FAILED

        #
        object_frame_id = goal.object_frame_id


        #TODO: Move gripper with object to goal
        #self._giskard_wrapper.set_cart_goal(self._root, object_frame_id, goal.goal_pose)
        #self._giskard_wrapper.plan_and_execute(wait=False)
        #result = self._giskard_wrapper.get_result(rospy.Duration(30))
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position
        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        q1 = [hsr_transform.transform.rotation.x, hsr_transform.transform.rotation.y, hsr_transform.transform.rotation.z,
              hsr_transform.transform.rotation.w]
        # place_mode
        if goal.place_mode == goal.FRONT:
            q2 = [0.7, 0.0, 0.7, 0.0]  # Quaternion for rotation to grasp from front relative to map for hand_palm_link
        elif goal.place_mode == goal.TOP:
            q2 = [1, 0, 0, 0]  # Quaternion for rotation to grasp from above relative to map for hand_palm_link

        if goal.place_mode != goal.FREE:
            q3 = quaternion_multiply(q1, q2)
            pose.pose.orientation = Quaternion(q3[0], q3[1], q3[2], q3[3])

        if goal.place_mode == goal.FRONT:
            pose_step = PoseStamped()
            pose_step.header.frame_id = "map"
            pose_step.header.stamp = rospy.Time.now()
            pose_step.pose.position = self.calculateWayPoint2D(pose.pose.position, hsr_transform.transform.translation, 0.1)
            pose_step.pose.orientation = pose.pose.orientation
            self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose_step)
            self._giskard_wrapper.plan_and_execute(wait=True)

        pose.header.stamp = rospy.Time.now()

        # Move the robot in goal position.
        #self._giskard_wrapper.avoid_all_collisions(distance=0.1)
        self._giskard_wrapper.allow_all_collisions()
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)
        self._giskard_wrapper.plan_and_execute()
        giskard_result = self._giskard_wrapper.get_result()

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
            self._giskard_wrapper.set_cart_goal(self._root, u'base_footprint', p_temp)
            self._giskard_wrapper.plan_and_execute(wait=True)

            ##TODO: load default pose from json file
            #self._giskard_wrapper.avoid_all_collisions(distance=0.1)
            self._giskard_wrapper.allow_all_collisions()
            self._giskard_wrapper.set_joint_goal({
                u'head_pan_joint': 0.0,
                u'head_tilt_joint': 0.0,
                u'arm_lift_joint': 0.0,
                u'arm_flex_joint': 0.0,
                u'arm_roll_joint': 1.4,
                u'wrist_flex_joint': -1.5,
                u'wrist_roll_joint': 0.14
            })
            self._giskard_wrapper.plan_and_execute(True)

    #        self._feedback.tf_gripper_to_goal = tfwrapper.lookup_transform(goal.goal_pose.header.frame_id, u'hand_palm_link')
    #        self._as.publish_feedback(self._feedback)

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer(rospy.get_name())
    rospy.spin()
