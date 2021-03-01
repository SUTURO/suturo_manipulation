#! /usr/bin/env python

import rospy
import actionlib
from manipulation_msgs.msg import PlaceAction, PlaceGoal, PlaceFeedback, PlaceResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper
from suturo_manipulation.manipulator import Manipulator


class PlaceServer:
    _feedback = PlaceFeedback()
    _result = PlaceResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._giskard_wrapper = GiskardWrapper()
        self._gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                                follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
        self._manipulator = Manipulator(mode_rotation=self.get_mode_rotation())
        self._as.start()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def get_mode_rotation(self):
        mode_rotation = {}
        front_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/front', default=None)
        top_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/top', default=None)
        if front_rotation:
            mode_rotation[PlaceGoal.FRONT] = front_rotation
        if top_rotation:
            mode_rotation[PlaceGoal.TOP] = top_rotation
        return mode_rotation

    def execute_cb(self, goal):
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Placing: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED

        if goal.object_frame_id not in self._giskard_wrapper.get_attached_objects().object_names:
            rospy.logwarn("object not attached to gripper: {}".format(goal.object_frame_id))

        # get current robot_pose
        robot_pose = tfwrapper.lookup_pose('map', 'base_footprint')

        success &= self._manipulator.move_to_goal(root_link=self._root,
                                                  tip_link=u'hand_palm_link',
                                                  goal_pose=goal.goal_pose,
                                                  robot_pose=robot_pose,
                                                  mode=goal.place_mode,
                                                  step=0.1)
        if success:
            self._gripper.set_gripper_joint_position(1.2)
            #if goal.object_frame_id in self._giskard_wrapper.get_attached_objects().object_names:
            #    self._giskard_wrapper.detach_object(goal.object_frame_id)
            self._gripper.publish_object_in_gripper(goal.object_frame_id, goal.goal_pose, ObjectInGripper.PLACED)
        robot_pose.header.stamp = rospy.Time.now()  # Might not be needed but is cleaner this way
        success &= self._manipulator.move_to_goal(root_link=self._root,
                                                  tip_link=u'base_footprint',
                                                  goal_pose=robot_pose)
        success &= self._manipulator.take_robot_pose(rospy.get_param(u'/manipulation/robot_poses/neutral'))
        if success:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer(rospy.get_name())
    rospy.spin()
