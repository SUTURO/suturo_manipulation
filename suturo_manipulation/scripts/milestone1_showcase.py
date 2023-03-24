#!/usr/bin/env python
import time

import rospy
import math  # for sin and cos
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3

from giskardpy.python_interface import GiskardWrapper


def prepare_variables():
    _giskard_wrapper = GiskardWrapper()

    # mueslibox
    mueslibox_center = PoseStamped()
    mueslibox_center.header.frame_id = 'map'
    mueslibox_center.pose.position.x = - 0.1
    mueslibox_center.pose.position.y = 1.68
    mueslibox_center.pose.position.z = 0.7

    # drawer
    drawer_point = PoseStamped()
    drawer_point.header.frame_id = 'map'
    drawer_point.pose.position.x = 0.18
    drawer_point.pose.position.y = -0.225
    drawer_point.pose.position.z = 0.282


    # Test Pose
    test_orientation = Quaternion(x=0.0, y=0.0, z=-1.0, w=1.0)
    test_pose = mueslibox_center
    test_pose.pose.orientation = test_orientation

    return _giskard_wrapper, mueslibox_center, drawer_point, test_pose


def pick_object(name: str,
                pose: PoseStamped,
                size: Vector3,
                root_link='map',
                tip_link='hand_palm_link',
                object_type='box'):
    # Open Gripper
    open_gripper()

    # Pick object
    print('Getting in position')
    # object_size = [0.04, 0.1, 0.2]  # FIXME make box size dynamic
    height = 0.259
    radius = 0.0395

    ### Will be removed with knowledge synchronization ###
    if object_name not in _giskard_wrapper.get_group_names():
        gisk_size = (size.x, size.y, size.z)
        gisk_pose = pose

        if object_type == 'box':
            _giskard_wrapper.add_box(name=name,
                                     size=gisk_size,
                                     pose=gisk_pose)
        elif object_type == 'cylinder':

            _giskard_wrapper.add_cylinder(name=name,
                                          height=height,
                                          radius=radius,
                                          pose=gisk_pose)

    #######################################################

    _giskard_wrapper.grasp_object(object_name=name,
                                  object_pose=pose,
                                  object_size=size,
                                  root_link=root_link,
                                  tip_link=tip_link)

    _giskard_wrapper.plan_and_execute(wait=True)
    # _giskard_wrapper.plan()

    # Attach Object
    _giskard_wrapper.update_parent_link_of_group(object_name, tip_link)

    print('Grabbing Object')
    _giskard_wrapper.move_gripper(False)
    _giskard_wrapper.plan_and_execute(wait=True)

    # Lift Object
    print('Lifting Object')
    _giskard_wrapper.lift_object(object_name=object_name)
    _giskard_wrapper.plan_and_execute(wait=True)

    # Retract
    print('Retracting')
    _giskard_wrapper.retract(object_name=object_name)
    _giskard_wrapper.plan_and_execute(wait=True)

    set_base_position()
    _giskard_wrapper.plan_and_execute(wait=True)


def open_gripper():
    print('Open Gripper')
    _giskard_wrapper.move_gripper(True)
    _giskard_wrapper.plan_and_execute(wait=True)


def place_object(name: str,
                 pose: PoseStamped,
                 height: float,
                 root_link='map',
                 tip_link='hand_palm_link'):
    # Align height
    _giskard_wrapper.prepare_placing(object_pose=pose)
    _giskard_wrapper.plan_and_execute(wait=True)

    # Place Object
    _giskard_wrapper.place_object(object_name=name,
                                  goal_pose=pose,
                                  object_height=height,
                                  root_link=root_link,
                                  tip_link=tip_link)
    _giskard_wrapper.plan_and_execute(wait=True)

    # Open Gripper

    open_gripper()

    # Detach Object
    print('Move Back')
    _giskard_wrapper.update_parent_link_of_group(object_name, root_link)
    _giskard_wrapper.avoid_collision(min_distance=0.01, group1=_giskard_wrapper.robot_name, group2=object_name)
    _giskard_wrapper.retract(object_name=object_name)

    _giskard_wrapper.plan_and_execute(wait=True)

    set_base_position()

    _giskard_wrapper.plan_and_execute(wait=True)


def set_base_position():
    joints = {'head_pan_joint': 0.0,
              'head_tilt_joint': 0.0,
              'arm_lift_joint': 0.0,
              'arm_flex_joint': 0.0,
              'arm_roll_joint': 1.4,
              'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0.14
              }
    _giskard_wrapper.set_joint_goal(joints)


def add_object(name: str,
               pose: PoseStamped,
               size: Vector3,
               object_type='box'):
    height = 0.259
    radius = 0.0395

    ### Will be removed with knowledge synchronization ###
    if object_name in _giskard_wrapper.get_group_names():
        _giskard_wrapper.remove_group(name)

    gisk_size = (size.x, size.y, size.z)
    gisk_pose = pose

    if object_type == 'box':
        _giskard_wrapper.add_box(name=name,
                                 size=gisk_size,
                                 pose=gisk_pose)
    elif object_type == 'cylinder':

        _giskard_wrapper.add_cylinder(name=name,
                                      height=height,
                                      radius=radius,
                                      pose=gisk_pose)

    _giskard_wrapper.plan_and_execute()


if __name__ == '__main__':
    rospy.init_node('milestone0_server')

    _giskard_wrapper, mueslibox_center, drawer_point, t_pose = prepare_variables()

    object_name = 'boxy'

    # LabEnv coordinates
    table_position = [1.6, -1.1, 0.9]
    shelf_position = [-0.092, 1.65, 0.75]

    # Grab mueslibox

    muesli_size = Vector3(x=0.04, y=0.1, z=0.2)

    obj_height = 0.2

    obj = table_position

    obj_pose = PoseStamped()
    obj_pose.header.frame_id = 'map'
    obj_pose.pose.position.x = obj[0]
    obj_pose.pose.position.y = obj[1]
    obj_pose.pose.position.z = obj[2] + 0.15

    # obj_pose = mueslibox_center
    obj_pose = t_pose

    #pick_object(name='', pose=obj_pose, size=muesli_size)

    # create object
    add_object(name=object_name, pose=obj_pose, size=muesli_size)

    # place object
    #place_object(name=object_name, pose=obj_pose, height=obj_height)

    # Drawer
    knob_size = Vector3(x=0.04, y=0.1, z=0.2)
    #pick_object(name='', pose=drawer_point, size=knob_size)
