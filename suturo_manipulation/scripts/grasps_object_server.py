#!/usr/bin/env python
import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult, ObjectInGripper
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point
from giskardpy import tfwrapper
from suturo_manipulation.gripper import Gripper

class GraspsObjectServer:
    # Add to YAML
    FRONT_ROTATION_QUATERNION = [0.7, 0.0, 0.7, 0.0]
    TOP_ROTATION_QUATERNION = [1, 0, 0, 0]

    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._giskard_wrapper = GiskardWrapper()
        self._gripper = Gripper(apply_force_action_server=u'/hsrb/gripper_controller/apply_force',
                 follow_joint_trajectory_server=u'/hsrb/gripper_controller/follow_joint_trajectory')
        self._as.start()
        print("GraspsActionServer greets its masters and is waiting for orders")

    def execute_cb(self, goal):

        print("Recieve Order. grasp", goal)
        self._giskard_wrapper.interrupt()

        # Set initial result value.
        self._result.error_code = self._result.FAILED
        goal_pose = goal.goal_pose

        #open gripper
        self._gripper.set_gripper_joint_position(1.2)
        in_gripper = False

        hsr_transform = tfwrapper.lookup_transform('map', 'base_footprint')
        base_tip_rotation = None
        step = None
        if goal.grasp_mode == goal.FRONT:
            #TODO: Hard coded offset
            goal_pose.pose.position.z += goal.object_size.z / 2.0
            base_tip_rotation = self.FRONT_ROTATION_QUATERNION
            step = 0.1

        elif goal.grasp_mode == goal.TOP:
            goal_pose.pose.position.z += goal.object_size.z
            rospy.loginfo("ACTUAL TOP GRASPING HEIGHT: " + str(goal_pose))
            base_tip_rotation = self.TOP_ROTATION_QUATERNION

        # Move the robot in goal position.        
        self._giskard_wrapper.avoid_all_collisions(distance=0.3)
        self._giskard_wrapper.allow_collision(body_b=goal.object_frame_id)
        self._giskard_wrapper.set_cart_goal_wstep(self._root,
                                                  u'hand_palm_link',
                                                  goal_pose,
                                                  base_tip_rotation=base_tip_rotation,
                                                  step=step,
                                                  base_transform=hsr_transform)
        result = self._giskard_wrapper.get_result(rospy.Duration(60))

        if result and result.SUCCESS in result.error_codes:
            # Close the Gripper
            self._gripper.close_gripper_force(0.8)

            if self._gripper.object_in_gripper():
                # Attach object
                self._giskard_wrapper.attach_object(goal.object_frame_id, u'hand_palm_link')
                self._gripper.publish_object_in_gripper(goal.object_frame_id, goal.goal_pose, ObjectInGripper.GRASPED)

            # Pose to move with an attached Object
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.header.stamp = rospy.Time.now()
            start_pose.pose.position = Point(hsr_transform.transform.translation.x,
                                             hsr_transform.transform.translation.y,
                                             hsr_transform.transform.translation.z)
            start_pose.pose.orientation = hsr_transform.transform.rotation
            self._giskard_wrapper.set_cart_goal_wstep(self._root, u'base_footprint', start_pose)

            self._giskard_wrapper.avoid_all_collisions(distance=0.03)
            self._giskard_wrapper.set_joint_goal(rospy.get_param(u'/robot_poses/transport'))
            self._giskard_wrapper.plan_and_execute(wait=True)
            result_giskard = self._giskard_wrapper.get_result()

            if in_gripper and result_giskard and result_giskard.SUCCESS in result_giskard.error_codes:
                self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()
