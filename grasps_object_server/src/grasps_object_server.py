! / usr / bin / env
python

import math
import sys

from geometry_msgs.msg import WrenchStamped

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult
from giskardpy.python_interface import GiskardWrapper
import hsrb_interface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
from listener import Listener
from sensor_msgs.msg import JointState
import force_checking
from giskardpy import tfwrapper


class GraspsObjectServer:
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self._force_checker = force_checking.ForceSensorCapture()
        print("GraspsActionServer greats its masters and is waiting for orders")

    def execute_cb(self, goal):

        print("Recieve Order. grasp")

        self._giskard_wrapper.interrupt()

        # grasped_object = u'grasped_object'
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position

        # Set initial result value.
        self._result.error_code = self._result.FAILED

        self._gripper.command(1.2)

        hsr_pose = tfwrapper.lookup_transform('map', 'base_footprint')
        q1 = [hsr_pose.transform.rotation.x, hsr_pose.transform.rotation.y, hsr_pose.transform.rotation.z,
              hsr_pose.transform.rotation.w]
        # grasp_mode
        if goal.grasp_mode == goal.FRONT:
            q2 = [0.7, 0.0, 0.7, 0.0]  # Quaternion for rotation to grasp from front relative to map for hand_palm_link
        elif goal.grasp_mode == goal.TOP:
            q2 = [1, 0, 0, 0]  # Quaternion for rotation to grasp from above relative to map for hand_palm_link

        q3 = quaternion_multiply(q1, q2)
        pose.pose.orientation = Quaternion(q3[0], q3[1], q3[2], q3[3])

        # Move the robot in goal position.
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)
        self._giskard_wrapper.plan_and_execute(wait=False)

        # TODO: send feedback periodically?

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result.error_code == result.SUCCESS:
            # Save the force before grasp
            self._force_checker._pre_force_list = self._force_checker.get_current_force()

            # Close the Gripper
            self._gripper.apply_force(1.0)

            # Wait for force sensor data to become stable and save the force after grasp
            rospy.sleep(1)
            self._force_checker._post_force_list = self._force_checker.get_current_force()

            # Calculate the current weight from object in gripper
            weight = self._force_checker.round_grasp()

            print weight

            # Attach object
            self._giskard_wrapper.attach_object(goal.object_frame_id)

            # Pose to move with an attatched Object
            neutral_js = {
                u'head_pan_joint': 0,
                u'head_tilt_joint': 0,
                u'arm_lift_joint': 0,
                u'arm_flex_joint': 0,
                u'arm_roll_joint': 1.4,
                u'wrist_flex_joint': -1.5,
                u'wrist_roll_joint': 0.14}

            self._giskard_wrapper.set_joint_goal(neutral_js)
            self._giskard_wrapper.plan_and_execute()

            result = self._giskard_wrapper.get_result()

        # self._feedback.tf_gripper_to_object = tfwrapper.lookup_transform(goal.object_frame_id, u'hand_palm_link')
        # self._feedback.gripper_joint_state = u'hand_l_spring_proximal_joint' + u'hand_r_spring_proximal_joint'

        if result and result.error_code == result.SUCCESS:
            self._result.error_code = self._result.SUCCESS

        self._as.set_succeeded(self._result)

    """
    Force checking method from Michelle
    def object_in_gripper(self, width_object):

        This method checks if the object is in the gripper, then position_value of gripper should be greater as -0.5
        :param width_object: float
            :return: false or true, boolean

        self.move_gripper(-2, 1, 0.8)
        l= Listener()
        l.set_topic_and_typMEssage("/hsrb/joint_states", JointState)
        l.listen_topic_with_sensor_msg()
        current_hand_motor_value= l.get_value_from_sensor_msg("hand_motor_joint")
        print("Current hand motor joint is:")
        print current_hand_motor_value
        print("Is object in gripper ?")
        print current_hand_motor_value >= -0.5
        return current_hand_motor_value >= -0.5
        """


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    rospy.spin()