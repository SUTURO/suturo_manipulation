#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
"""Speak Object Weight Sample"""

import math
import os
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)

_CONNECTION_TIMEOUT = 10.0

# Create speak sentences
# Index 0: in Japanese, 1: in English
_EXPLAIN1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',
             u'Please set the object between my gripper']
_EXPLAIN2 = [u'グリッパを閉じます', u'I close my hand now']
_ANSWER = [u'これは{0}グラムです', u'This is {0} gram']


def compute_difference(pre_data_list, post_data_list):
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
    # Calcurate square sum of difference
    square_sums = sum([math.pow(b - a, 2)
                       for (a, b) in zip(pre_data_list, post_data_list)])
    return math.sqrt(square_sums)


class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z


class JointController(object):
    """Control arm and gripper"""

    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(
            joint_control_service, SafeJointChange)

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(
                timeout=_CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(
                    _CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def move_to_joint_positions(self, goal_joint_states):
        """Joint position control"""
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success

    def grasp(self, effort):
        """Gripper torque control"""
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


class Speaker(object):
    """Speak sentence in robot's language"""

    def __init__(self):
        talk_action = '/talk_request_action'
        self._talk_request_client = actionlib.SimpleActionClient(
            talk_action, TalkRequestAction)

        # Wait for connection
        try:
            if not self._talk_request_client.wait_for_server(
                    rospy.Duration(_CONNECTION_TIMEOUT)):
                raise Exception(talk_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # Detect robot's language
        if os.environ['LANG'] == 'ja_JP.UTF-8':
            self._lang = Voice.kJapanese
        else:
            self._lang = Voice.kEnglish

    def get_language(self):
        return self._lang

    def speak_sentence(self, sentence):
        goal = TalkRequestGoal()
        goal.data.language = self._lang
        goal.data.sentence = sentence

        if (self._talk_request_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


def main():
    # Start force sensor capture
    force_sensor_capture = ForceSensorCapture()

    # Set initial pose
    joint_controller = JointController()

    initial_position = JointState()
    initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'head_pan_joint',
                                  'head_tilt_joint', 'hand_motor_joint'])
    initial_position.position.extend([0.0, 0.0, 0.0, -1.57,
                                      0.0, 0.0, 0.0, 1.2])
    joint_controller.move_to_joint_positions(initial_position)

    # Get initial data of force sensor
    pre_force_list = force_sensor_capture.get_current_force()

    # Ask user to set object
    speaker = Speaker()
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    rospy.sleep(2.0)

    # Inform user of next gripper action
    speaker.speak_sentence(_EXPLAIN2[speaker.get_language()])
    rospy.sleep(1.0)

    # Grasp the object
    joint_controller.grasp(-0.1)

    # Wait until force sensor data become stable
    rospy.sleep(1.0)
    post_force_list = force_sensor_capture.get_current_force()

    force_difference = compute_difference(pre_force_list, post_force_list)

    # Convert newton to gram
    weight = round(force_difference / 9.81 * 1000, 1)

    # Speak object weight in first decimal place
    speaker.speak_sentence(_ANSWER[speaker.get_language()].format(weight))

if __name__ == '__main__':
    rospy.init_node('hsrb_speak_object_weight')
    main()