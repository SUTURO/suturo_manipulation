#!/usr/bin/env python
import math

import rospy
from std_srvs.srv import SetBool
from giskardpy.python_interface import GiskardWrapper
from giskardpy.utils import to_tf_quaternion
from geometry_msgs.msg import Quaternion, PoseStamped, PointStamped, Vector3Stamped, Point
from tf.transformations import quaternion_multiply, quaternion_from_matrix
from sensor_msgs.msg import JointState


class Manipulator:
    """
    Handles the movement and communication with Giskard.
    """
    def __init__(self, collision_distance=0.05, mode_rotation=None):
        """
        Proxy class working with giskard for manipulating object with a gripper
        :param collision_distance: distance to keep from objects None or < 0 allow all collisions
        :type collision_distance:float
        :param mode_rotation: Rotation of tip depending on robot_pose
        :type mode_rotation: dict(mode: [x, y, z, w])
        """

        self.mode_rotation_ = mode_rotation
        self.collision_distance_ = collision_distance
        self.giskard_wrapper_ = GiskardWrapper()
        self.giskard_wrapper_._object_js_topics['iai_kitchen'] = rospy.Publisher('kitchen/cram_joint_states', JointState, queue_size=10)


    def set_collision(self, distance, collision_whitelist=None):
        """
        Sets the collision avoidance for the Manipulator and next giskard_goal.
        To disable collision avoidance set to None or <= 0.
        :param distance: min distance to objects
        :type distance: float
        :param collision_whitelist: list of body names allowed for collision
        :type collision_whitelist: list[string]
        :return:
        """
        self.collision_distance_ = distance
        if 0 >= self.collision_distance_:
            self.giskard_wrapper_.allow_all_collisions()
        else:
            self.giskard_wrapper_.avoid_all_collisions(distance=self.collision_distance_)
            if collision_whitelist:
                for body in collision_whitelist:
                    self.giskard_wrapper_.allow_collision(body_b=body)

    def move_to_goal(self, root_link, tip_link, goal_pose, robot_pose=None, mode=None, step=None, collision_whitelist=None):
        """
        Moves the tip to the given goal. Look at parameters for additional information
        :param root_link: name of the root link of the kinematic chain
        :type root_link: string
        :param tip_link: name of the tip link of the kinematic chain
        :type tip_link: string
        :param robot_pose: current pose of the robot (optional required for step and mode to work)
        :type robot_pose: PoseStamped
        :param goal_pose: goal pose for the tip
        :type goal_pose: PoseStamped
        :param mode: mode for rotation of tip (optional if None orientation of goal pose is used)
        :type
        :param step: distance for step from goal (m) (optional)
        :type step: float
        :param collision_whitelist: list of body names allowed for collision (optional if None avoid all collisions)
        :type collision_whitelist: list[string]
        :return: True on success
        """
        self.giskard_wrapper_.interrupt()
        if robot_pose and mode and self.mode_rotation_ and mode in self.mode_rotation_:
            goal_pose.pose.orientation = Quaternion(*quaternion_multiply(to_tf_quaternion(robot_pose.pose.orientation),
                                                                         self.mode_rotation_[mode]))
        if robot_pose and step:
            step_pose = PoseStamped()
            step_pose.header.frame_id = goal_pose.header.frame_id
            step_pose.header.stamp = rospy.Time.now()
            step_pose.pose.position = self.calculate_waypoint2D(goal_pose.pose.position, robot_pose.pose.position, step)
            step_pose.pose.orientation = goal_pose.pose.orientation
            rospy.loginfo("step_pose: {}".format(step_pose))
            # Move to the defined step
            self.set_collision(self.collision_distance_, collision_whitelist)
            self.giskard_wrapper_.set_cart_goal(goal_pose=step_pose, tip_link=tip_link, root_link=root_link)
            self.giskard_wrapper_.plan_and_execute(wait=True)
        rospy.loginfo("goal_pose: {}".format(goal_pose))
        goal_pose.header.stamp = rospy.Time.now()
        self.set_collision(self.collision_distance_, collision_whitelist)
        self.giskard_wrapper_.set_cart_goal(goal_pose=goal_pose, tip_link=tip_link, root_link=root_link)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        rospy.loginfo("Giskard result: {}".format(result.error_codes))
        return result and result.SUCCESS in result.error_codes

    def take_robot_pose(self, robot_pose):
        """
        Lets the robot take the given pose
        :param robot_pose: Dictionary (joint_name: position)
        :type robot_pose: dict
        :return: True if success
        """
        self.giskard_wrapper_.interrupt()
        self.set_collision(self.collision_distance_)
        self.giskard_wrapper_.set_joint_goal(robot_pose)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        return result and result.SUCCESS in result.error_codes

    def grasp_bar(self, root_link, tip_link, goal_pose):
        """
        Lets the robot grasp the bar of an object (in this case the handles)
        :type root_link str
        :param root_link The base/root link of the robot
        :type tip_link str
        :param tip_link The tip link of the robot(gripper)
        :type goal_pose PoseStamped
        :param goal_pose The goal of the grasp bar action
        """
        self.set_collision(-1)
        #self.set_collision(self.collision_distance_)
        self.giskard_wrapper_.set_cart_goal(goal_pose=goal_pose, tip_link=tip_link, root_link=root_link)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        return result and result.SUCCESS in result.error_codes

    def open(self, tip_link, object_name_prefix, object_link_name, angle_goal, use_limitation):
        """
        Lets the robot open the given object
        :type tip_link str
        :param tip_link the name of the gripper
        :type object_name_prefix str
        :param object_name_prefix the object link name prefix
        :type object_link_name str
        :param object_link_name handle to grasp
        :type angle_goal float
        :param angle_goal the angle goal in relation to the current status
        :type use_limitation bool
        :param use_limitation indicator, if the limitation should be used
        """
        self.change_base_scan_limitation(use_limitation)
        self.set_collision(-1)
        updates = {
            u'rosparam': {
                u'general_options': {
                    u'joint_weights': {
                        u'arm_lift_joint': 1000,
                        u'arm_roll_joint': 1000
                    }
                }
            }
        }
        rospy.logerr(angle_goal)
        self.giskard_wrapper_.update_god_map(updates)
        self.giskard_wrapper_.set_pull_door_goal(tip_link, object_name_prefix, object_link_name.split('/')[1], angle_goal)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        if use_limitation:
            self.change_base_scan_limitation(False)
        return result and result.SUCCESS in result.error_codes

    def opendrawer(self, tip_link, object_name_prefix, object_link_name, distance_goal, use_limitation):
        """
        Lets the robot open the drawer
        :type tip_link str
        :param tip_link the name of the gripper
        :type object_name_prefix str
        :param object_name_prefix the object link name prefix
        :type object_link_name str
        :param object_link_name handle to grasp
        :type distance_goal float
        :param distance_goal the distance_goal in relation to the current status
        :type use_limitation bool
        :param use_limitation indicator, if the limitation should be used
        """
        self.change_base_scan_limitation(use_limitation)
        self.set_collision(-1)
        rospy.logerr(distance_goal)
        self.giskard_wrapper_.set_open_drawer_goal(tip_link, object_name_prefix, object_link_name.split('/')[1], distance_goal)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        if use_limitation:
            self.change_base_scan_limitation(False)
        self.giskard_wrapper_.get_object_info('iai_kitchen')
        self.giskard_wrapper_.set_object_joint_state('iai_kitchen', {'drawer_move': distance_goal})
        return result and result.SUCCESS in result.error_codes

    def closedrawer(self, tip_link, object_name_prefix, object_link_name, distance_goal, use_limitation):
        """
        Lets the robot close the drawer
        :type tip_link str
        :param tip_link the name of the gripper
        :type object_name_prefix str
        :param object_name_prefix the object link name prefix
        :type object_link_name str
        :param object_link_name knob to grasp
        :type distance_goal float
        :param distance_goal the distance_goal in relation to the current status
        :type use_limitation bool
        :param use_limitation indicator, if the limitation should be used
        """
        self.change_base_scan_limitation(use_limitation)
        self.set_collision(-1)
        rospy.logerr(distance_goal)
        self.giskard_wrapper_.set_close_drawer_goal(tip_link, object_name_prefix, object_link_name.split('/')[1], distance_goal)
        self.giskard_wrapper_.plan_and_execute(wait=True)
        result = self.giskard_wrapper_.get_result()
        if use_limitation:
            self.change_base_scan_limitation(False)
        self.giskard_wrapper_.get_object_info('iai_kitchen')
        self.giskard_wrapper_.set_object_joint_state('iai_kitchen', {'drawer_move': distance_goal})
        return result and result.SUCCESS in result.error_codes

    def change_base_scan_limitation(self, indicator):
        """
        Switches the base scan limitation on/off.
        :param indicator Switch off left (false) or right (true) side
        :type indicator bool
        """
        rospy.wait_for_service('/base_scan_limitation')
        try:
            base_scan_limitation = rospy.ServiceProxy('/base_scan_limitation', SetBool)
            response = base_scan_limitation(indicator)
            return response.success
        except rospy.ServiceProxy as e:
            print("base scan limitation failed: %e"%e)

    def calculate_waypoint2D(self, target, origin, distance):
        """
        Calculates a waypoint in front of the target.
        :param target: The target position
        :type target: Point
        :param origin: The origin position
        :type origin: Point
        :param distance: The distance of the new waypoint to the target
        :type distance: float
        :return:
        """
        distance = abs(distance)
        x = target.x - origin.x
        y = target.y - origin.y

        if x == 0.0:
            if y < 0.0:
                return Point(target.x, target.y + distance, target.z)
            else:
                return Point(target.x, target.y - distance, target.z)

        alpha = math.atan(y / x)
        dx = math.cos(alpha) * distance
        dy = math.sin(alpha) * distance

        if x > 0.0:
            return Point(target.x - dx, target.y - dy, target.z)
        else:
            return Point(target.x + dx, target.y + dy, target.z)

