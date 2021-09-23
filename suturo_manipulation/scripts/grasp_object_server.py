#!/usr/bin/env python
import rospy
import actionlib
from manipulation_msgs.msg import GraspAction, GraspGoal, GraspFeedback, GraspResult, ObjectInGripper
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper
from suturo_manipulation.manipulator import Manipulator
from giskardpy.python_interface import GiskardWrapper


class GraspsObjectServer:
    """
    Action Server, which handles the Grasping of objects
    """
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        """
        Initializes the Server.
        :param name The server name
        :type name string
        """
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                                follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
        self._manipulator = Manipulator(mode_rotation=self.get_mode_rotation())
        self._giskard_wrapper = GiskardWrapper()
        self._as.start()
        if tfwrapper.tfBuffer is None:
            tfwrapper.init()
        rospy.loginfo("{} is ready and waiting for orders.".format(self._action_name))

    def get_mode_rotation(self):
        """
        Loads the gripper rotation modes from the ROS parameter server.
        """
        mode_rotation = {}
        front_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/front', default=None)
        top_rotation = rospy.get_param(u'/manipulation/base_gripper_rotation/top', default=None)
        if front_rotation:
            mode_rotation[GraspGoal.FRONT] = front_rotation
        if top_rotation:
            mode_rotation[GraspGoal.TOP] = top_rotation
        return mode_rotation

    def execute_cb(self, goal):
        """
        Executes the grasping of objects.
        :param goal The grasp goal
        :type goal GraspGoal
        """
        # uncomment to disable collision avoidance
        # self._manipulator.set_collision(None)
        rospy.loginfo("Grasping: {}".format(goal))
        # Set initial result value.
        success = True
        self._result.error_code = self._result.FAILED
        collision_whitelist = []
        if goal.object_frame_id in self._giskard_wrapper.get_object_names().object_names:
            collision_whitelist.append(goal.object_frame_id)
        else:
            rospy.logwarn("unknown object: {}".format(goal.object_frame_id))

        # get current robot_pose
        robot_pose = tfwrapper.lookup_pose('map', 'base_footprint')
        # open gripper
        self._gripper.set_gripper_joint_position(1.2)

        success &= self._manipulator.move_to_goal(root_link=self._root,
                                                  tip_link=u'hand_gripper_tool_frame',
                                                  goal_pose=goal.goal_pose,
                                                  robot_pose=robot_pose,
                                                  mode=goal.grasp_mode,
                                                  step=0.1,
                                                  collision_whitelist=collision_whitelist)
        if success:
            self._gripper.close_gripper_force(0.8)
            success &= self._gripper.object_in_gripper()
            if success:
                # Attach object
                if goal.object_frame_id in self._giskard_wrapper.get_object_names().object_names:
                    self._giskard_wrapper.attach_object(goal.object_frame_id, u'hand_palm_link')
                self._gripper.publish_object_in_gripper(goal.object_frame_id, goal.goal_pose, ObjectInGripper.GRASPED)
        robot_pose.header.stamp = rospy.Time.now()  # Might not be needed but is cleaner this way
        success &= self._manipulator.move_to_goal(root_link=self._root,
                                                  tip_link=u'base_footprint',
                                                  goal_pose=robot_pose)
        success &= self._manipulator.take_robot_pose(rospy.get_param(u'/manipulation/robot_poses/transport'))
        if success:
            self._result.error_code = self._result.SUCCESS
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('grasp_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()
