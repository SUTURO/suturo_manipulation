#! /usr/bin/env python
import actionlib
import rospy
from geometry_msgs.msg import Quaternion
from manipulation_msgs.msg import MakePlanAction, MakePlanGoal, MakePlanResult
from tf.transformations import quaternion_multiply

from giskardpy import tfwrapper
from giskardpy.python_interface import GiskardWrapper
from giskardpy.utils import to_tf_quaternion


class MakePlanServer:

    def __init__(self, name):
        self._action_name = name
        self.dummy_object_ = u'dummy_plan_object'
        self.gripper_frame_ = u'hand_palm_link'
        self.root_frame_ = u'odom'
        self._as = actionlib.SimpleActionServer(self._action_name, MakePlanAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._giskard_wrapper = GiskardWrapper()
        self.mode_rotation_ = self.get_mode_rotation()
        self._as.start()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def get_mode_rotation(self):
        mode_rotation = {}
        front_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/front', default=None)
        top_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/top', default=None)
        if front_rotation:
            mode_rotation[MakePlanGoal.FRONT] = front_rotation
        if top_rotation:
            mode_rotation[MakePlanGoal.TOP] = top_rotation
        return mode_rotation

    def execute_cb(self, goal):
        rospy.loginfo("Making plan: {}".format(goal))
        self._result = MakePlanResult()
        self._result.error_code = self._result.FAILED

        # TODO move robot in start pose currently replaced with actual robot pose
        robot_pose = tfwrapper.lookup_pose('map', 'base_footprint')

        # Prepare object
        if goal.action_mode == goal.PLACE:
            if goal.object_frame_id not in self._giskard_wrapper.get_attached_objects().object_names:
                rospy.loginfo("object not attached to gripper: {}. Creating dummy object".format(goal.object_frame_id))
                if self.dummy_object_ in self._giskard_wrapper.get_object_names().object_names:
                    self._giskard_wrapper.remove_object(self.dummy_object_)
                self._giskard_wrapper.add_box(self.dummy_object_,
                                              [goal.object_size.x, goal.object_size.y, goal.object_size.z],
                                              self.gripper_frame_)
                self._giskard_wrapper.attach_object(self.dummy_object_, self.gripper_frame_)
        elif goal.action_mode == goal.GRASP:
            if goal.object_frame_id in self._giskard_wrapper.get_object_names().object_names:
                self._giskard_wrapper.allow_collision(body_b=goal.object_frame_id)
        else:
            rospy.logerr("Unknown action_mode: {}".format(goal.action_mode))
            self._as.set_succeeded(self._result)
            return
        if goal.gripper_mode in [goal.FRONT, goal.TOP]:
            goal.goal_pose.pose.orientation = Quaternion(
                *quaternion_multiply(to_tf_quaternion(robot_pose.pose.orientation),
                                     self.mode_rotation_[goal.gripper_mode]))

        self._giskard_wrapper.set_cart_goal(self.root_frame_, self.gripper_frame_, goal.goal_pose)
        self._giskard_wrapper.plan()
        result = self._giskard_wrapper.get_result()

        # Clean up dummy object
        if self.dummy_object_ in self._giskard_wrapper.get_attached_objects().object_names:
            self._giskard_wrapper.detach_object(self.dummy_object_)
            self._giskard_wrapper.remove_object(self.dummy_object_)
        # Clean up collission excemption for goal_object
        self._giskard_wrapper.avoid_all_collisions()

        if result.SUCCESS in result.error_codes:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('make_plan_server')
    server = MakePlanServer(rospy.get_name())
    rospy.spin()
