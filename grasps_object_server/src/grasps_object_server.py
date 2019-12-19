#!/usr/bin/env python

import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspFeedback, GraspResult
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
import hsrb_interface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply

import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
#from listener import Listener
from sensor_msgs.msg import JointState


class GraspsObjectServer:
    _feedback = GraspFeedback()
    _result = GraspResult()
    _root = u'odom'

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GraspAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._giskard_wrapper = GiskardWrapper()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot('whole_body')
        self._gripper = self._robot('gripper')
        print("GraspsActionServer greats its masters and is waiting for orders")

        # initialize action client
        self.cli = actionlib.SimpleActionClient(
            '/hsrb/gripper_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        self.cli.wait_for_server()

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        self._running = False
        while self._running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'gripper_controller' and c.state == 'running':
                    self._running = True

    def execute_cb(self, goal):

        print("Recieve Order")

        self._giskard_wrapper.interrupt()

        grasped_object = u'grasped_object'
        pose = PoseStamped()
        pose.header = goal.goal_pose.header
        pose.pose.position = goal.goal_pose.pose.position

        # Remove old object, with same name like the new object.
        self._giskard_wrapper.detach_object(grasped_object)
        self._giskard_wrapper.remove_object(grasped_object)

        # Set initial result value.
        self._result.error_code = self._result.FAILED

        '''
        # Open the gripper.
        self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.7,
                                              u'hand_r_spring_proximal_joint': 0.7})
        self._giskard_wrapper.plan_and_execute(wait=True)
        '''

        self._gripper.command(0.0)
        self._gripper.command(1.2)

        quat1 = [goal.goal_pose.pose.orientation.x, goal.goal_pose.pose.orientation.y, goal.goal_pose.pose.orientation.z, goal.goal_pose.pose.orientation.w]


        # gripper.command(1.2)

        orientation = quaternion_multiply(quat1, quaternion_from_euler(0, 1.57, 0))
        pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

        # Move the robot in goal position.
        self._giskard_wrapper.set_cart_goal(self._root, u'hand_palm_link', pose)

        # TODO: send feedback periodically?

        result = self._giskard_wrapper.get_result(rospy.Duration(60))
        if result.error_code == result.SUCCESS:

            # Close the Gripper
            '''
            self._giskard_wrapper.set_joint_goal({u'hand_l_spring_proximal_joint': 0.2,
                                                  u'hand_r_spring_proximal_joint': 0.2})
            self._giskard_wrapper.plan_and_execute()
            '''
            self._gripper.set_distance(0.05)

            # Attach object
            self._giskard_wrapper.add_cylinder(name=grasped_object, size=(goal.object_size.x, goal.object_size.y), pose=goal.goal_pose)
            self._giskard_wrapper.attach_object(name=grasped_object, link_frame_id=u'hand_palm_link')

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

    def move_gripper(self, position_value, velocity, effort):
        """
        this method is used to catch objects with the grippers
        :param position_value: float
        :param velocity: float
        :param effort: float
        :return:
        """
        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [position_value - 0.42]
        p.velocities = [velocity]
        p.effort = [effort]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        self.cli.send_goal(goal)

        # wait for the action server to complete the order
        self.cli.wait_for_result()
        return self._running

    def close_gripper(self):
        """
        this method close the gripper
        :return:
        """
        return self.move_gripper(0.05, 1, 0.0)

    def open_gripper(self):
        """
        this method open the gripper.
        :return:
        """
        return self.move_gripper(1.2, 1, 0.0)


if __name__ == '__main__':
    rospy.init_node('grasps_object_server')
    server = GraspsObjectServer(rospy.get_name())
    #server.open_gripper()
    rospy.spin()
