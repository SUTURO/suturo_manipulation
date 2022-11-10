#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped
from giskardpy.python_interface import GiskardWrapper
import actionlib
import moveit_msgs.msg as move

import tf

import hsrb_interface
from manipulation_msgs.msg import TakePoseAction, TakePoseFeedback, TakePoseResult


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

        self.listener = tf.TransformListener()

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
        new_goal = self.listener.transformPose('map', goal)
        self._result.error_code = self._result.FAILED

        self._giskard_wrapper.set_cart_goal(root_link='map', tip_link='hand_palm_link', goal_pose=new_goal)

        self._giskard_wrapper.allow_collision()
        self._giskard_wrapper.plan_and_execute()


        '''
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
        '''

        result = self._giskard_wrapper.get_result(rospy.Duration(120))
        if result.SUCCESS in result.error_codes:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    server = TakePoseServer(rospy.get_name())
    rospy.spin()

'''
# Start
rospy.init_node('test')
# rospy.loginfo('Start')

base_goal = PoseStamped()
base_goal.header.frame_id = 'map'
base_goal.pose.position = Point(0, 0, 0)
base_goal.pose.orientation = Quaternion(0, 0, 0, 1)

#giskard_wrapper.set_cart_goal(root_link = 'map', tip_link = 'base_footprint', goal_pose = base_goal)

# giskard_wrapper.allow_all_collisions()
#giskard_wrapper.plan_and_execute()

if result.SUCCESS in result.error_codes:
    self._result.error_code = self._result.SUCCESS
self._as.set_succeeded(self._result)
'''
