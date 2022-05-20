#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
from manipulation_msgs.msg import TakePoseAction, TakePoseFeedback, TakePoseResult
from giskardpy.python_interface import GiskardWrapper
from suturo_manipulation.manipulator import Manipulator

class TakePoseServer:
    """
    Action Server, which handles the pose taking.
    """
    _feedback = TakePoseFeedback()
    _result = TakePoseResult()
    _goal = TakePoseAction()

    def __init__(self, name):
        """
        Initializes the Server.
        :param name The server name
        :type name string
        """
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, TakePoseAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self._manipulator = Manipulator(collision_distance=0.01)
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def execute_cb(self, goal):
        """
        Executes the pose taking
        :param goal The goal of this action
        :type goal TakePoseGoal
        """
        rospy.loginfo("Take pose: {}".format(goal))
        self._result.error_code = self._result.FAILED
        #self._manipulator.set_collision(-1)
        self._giskard_wrapper.allow_self_collision()
        if goal.pose_mode == goal.FREE:
            self._giskard_wrapper.set_joint_goal({
                u'head_pan_joint': goal.head_pan_joint,
                u'head_tilt_joint': goal.head_tilt_joint,
                u'arm_lift_joint': goal.arm_lift_joint,
                u'arm_flex_joint': goal.arm_flex_joint,
                u'arm_roll_joint': goal.arm_roll_joint,
                u'wrist_flex_joint': goal.wrist_flex_joint,
                u'wrist_roll_joint': goal.wrist_roll_joint
            })
            self._giskard_wrapper.plan_and_execute(wait=True)
        elif goal.pose_mode == goal.NEUTRAL:
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/neutral'))
            self._giskard_wrapper.plan_and_execute(wait=True)
        elif goal.pose_mode == goal.LOOK_HIGH:
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/look_high'))
            self._giskard_wrapper.plan_and_execute(wait=True)
        elif goal.pose_mode == goal.LOOK_LOW:
            self._manipulator.set_collision(-1)
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/look_low'))
            self._giskard_wrapper.plan_and_execute(wait=True)
        elif goal.pose_mode == goal.LOOK_FLOOR:
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/look_floor'))
            self._giskard_wrapper.plan_and_execute(wait=True)
        elif goal.pose_mode == goal.LOOK_DRAWER:
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/look_drawer'))
            self._giskard_wrapper.plan_and_execute(wait=True)

        elif goal.pose_mode == goal.GAZE:
            camera_height = goal.gaze_point.z - 0.4
            if camera_height < 0.0:
                camera_height = 0.0
            elif camera_height > 0.69:
                camera_height = 0.69
            self._giskard_wrapper.set_joint_goal({
                u'head_pan_joint': -1.54,
                u'head_tilt_joint': 0.0,
                u'arm_lift_joint': camera_height,  # must be between 0 and 0.69
                u'arm_flex_joint': -0.5,
                u'arm_roll_joint': -1.8,
                u'wrist_flex_joint': -1.57,
                u'wrist_roll_joint': 0.0
            })
            self._giskard_wrapper.plan_and_execute(wait=True)
            v3 = hsrb_interface.geometry.Vector3(x=goal.gaze_point.x, y=goal.gaze_point.y, z=goal.gaze_point.z)
            self._whole_body.gaze_point(point=v3, ref_frame_id='map')
        elif goal.pose_mode == goal.GIVE_TAKE:
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/manipulation/robot_poses/give_take'))
            self._giskard_wrapper.plan_and_execute(wait=True)
        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result and result.SUCCESS in result.error_codes:
            self._result.error_code = self._result.SUCCESS

        self._giskard_wrapper.avoid_self_collision()
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('take_pose_server')
    server = TakePoseServer(rospy.get_name())
    rospy.spin()
