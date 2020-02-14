#!/usr/bin/env python

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


class ForceSensorCapture(object):

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self.pre_force_list = []
        self.force_list = []


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

    def compute_difference(self, pre_data_list, post_data_list):
        if (len(pre_data_list) != len(post_data_list)):
            raise ValueError('Argument lists differ in length')
        # Calcurate square sum of difference
        square_sums = sum([math.pow(b - a, 2)
                           for (a, b) in zip(pre_data_list, post_data_list)])
        return math.sqrt(square_sums)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

    def round_grasp(self):
        force_difference = self.compute_difference(self.pre_force_list, self.post_force_list)
        print force_difference
        return round(force_difference / 9.81 , 2)

"""
    def main(self):
        # Start force sensor capture
        force_sensor_capture = ForceSensorCapture()
        pre_force_list = force_sensor_capture.get_current_force()
        # Wait until force sensor data become stable
        rospy.sleep(1.0)
        post_force_list = force_sensor_capture.get_current_force()
        force_difference = self.compute_difference(pre_force_list, post_force_list)
        # Convert newton to gram
        weight = round(force_difference / 9.81 * 1000, 1)
        print weight
"""

#if __name__ == '__main__':
#    rospy.init_node('force_measure')
#    force_sensor_capture = ForceSensorCapture()
#    force_sensor_capture.main()