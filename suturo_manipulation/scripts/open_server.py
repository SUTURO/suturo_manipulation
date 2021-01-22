#!/usr/bin/env python
import rospy
import actionlib
from manipulation_msgs.msg import OpenAction, OpenGoal, OpenFeedback, OpenResult
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper
from suturo_manipulation.manipulator import Manipulator
from giskardpy.python_interface import GiskardWrapper


class OpenServer:
    _feedback = OpenFeedback()
    _result = OpenResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, OpenAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._manipulator = Manipulator()
        self._giskard_wrapper = GiskardWrapper()
        self._as.start()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def execute_cb(self, goal):
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Opening: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED

        collision_whitelist = []
#        if (goal.object_name and goal.object_link_name) not in self._giskard_wrapper.get_object_names().object_names:
#            rospy.logerr("unknown object: {} or unknown object_link: {}".format(goal.object_name, goal.object_link_name))
#            self._as.set_succeeded(self._result)
#            return

        # get current robot_pose
        robot_pose = tfwrapper.lookup_pose('map', 'base_footprint')

        # open
        success &= self._manipulator.open(goal.object_name, goal.object_link_name)
        # return to initial pose
        if success:
            success &= self._manipulator.move_to_goal(root_link=self._root,
                                                      tip_link=u'base_footprint',
                                                      goal_pose=robot_pose)
        success &= self._manipulator.take_robot_pose(rospy.get_param(u'/manipulation/robot_poses/transport'))
        if success:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('open_server')
    server = OpenServer(rospy.get_name())
    rospy.spin()
