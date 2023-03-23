#!/usr/bin/env python
import time

import rospy
import math  # for sin and cos
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3

from giskardpy.python_interface import GiskardWrapper


def prepare_variables():
    _giskard_wrapper = GiskardWrapper()

    # mueslibox variables
    mueslibox_center = PoseStamped()
    mueslibox_center.header.frame_id = 'map'
    mueslibox_center.pose.position.x = - 0.1
    mueslibox_center.pose.position.y = 1.68
    mueslibox_center.pose.position.z = 0.7

    mueslibox_tip_link = "hand_palm_link"

    box_z_length = 0.2

    floor_pose = PoseStamped()
    floor_pose.header.frame_id = 'map'
    floor_pose.pose.position.x = - 0.1
    floor_pose.pose.position.y = 1.68
    floor_pose.pose.position.z = 0.6

    pick_direction = Vector3()
    pick_direction.z = 1

    pick_distance = 0.1

    mueslibox_vars = (mueslibox_center, mueslibox_tip_link, box_z_length, floor_pose, pick_direction, pick_distance)

    # drawer variables

    # y = -0.02 for test without gripper
    # y = -0.225 for real test.
    drawer_point = PoseStamped()
    drawer_point.header.frame_id = 'map'
    drawer_point.pose.position.x = 0.18
    drawer_point.pose.position.y = -0.225
    drawer_point.pose.position.z = 0.282

    drawer_open_direction = Vector3()
    drawer_open_direction.y = 1

    drawer_close_direction = Vector3()
    drawer_close_direction.y = -1

    drawer_distance = 0.1

    drawer_vars = (drawer_point, drawer_open_direction, drawer_distance, drawer_close_direction)

    return _giskard_wrapper, mueslibox_vars, drawer_vars


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

    # _giskard_wrapper.plan_and_execute(wait=True)
    _giskard_wrapper.plan()

    # Attach Object
    _giskard_wrapper.update_parent_link_of_group(object_name, tip_link)
    '''
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
    '''

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

if __name__ == '__main__':
    rospy.init_node('milestone0_server')

    _giskard_wrapper, mueslibox_variables, drawer_variables = prepare_variables()

    object_name = 'boxy'

    # Testing: Open drawer
    # test_open_drawer(drawer_variables[0])

    # Testing: Mueslibox
    # test_muesli(mueslibox_point)

    # test_close_gripper()
    # test_open_gripper()

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

    obj_pose = mueslibox_variables[0]
    # _giskard_wrapper.grasp_object(object_name=box_name, object_pose=mueslibox_variables[0])
    # _giskard_wrapper.grasp_object(object_name=box_name, object_pose=obj_pose, object_size=muesli_size)

    #pick_object(name=object_name, pose=obj_pose, size=muesli_size)

    #_giskard_wrapper.plan_and_execute(wait=True)

    #place_object(name=object_name, pose=obj_pose, height=obj_height)

    # _giskard_wrapper.add_object_to_world(object_name=box_name, object_pose=mueslibox_variables[0])

    # Place mueslibox
    #_giskard_wrapper.place_object(object_name=box_name, goal_pose=mueslibox_variables[3], object_height=height, testing=True)

    # _giskard_wrapper.drive_back(distance=0.5)

    # Drawer

    knob_size = [0.04, 0.01, 0.02]
    Vector3(x=0.04, y=0.1, z=0.2)
    # Move to drawer
    # _giskard_wrapper.grasp_box(object_pose=drawer_variables[0], object_size=knob_size)
    # _giskard_wrapper.grasp_box(object_pose=drawer_variables[0], box_z=0.001, mueslibox=False, grasp_vertical=False)

    pick_object(name='', pose=drawer_variables[0], size=knob_size)

    # Open drawer
    # _giskard_wrapper.move_drawer(knob_pose=drawer_variables[0], direction=drawer_variables[1], distance=drawer_variables[2])
    # Close drawer
    # _giskard_wrapper.move_drawer(knob_pose=drawer_variables[0], direction=drawer_variables[3], distance=drawer_variables[2])

    # Run
    # _giskard_wrapper.plan_and_execute()

    # Drawer example
    # show_open_close_drawer()

    # Mueslibox example
    # show_pick_place_mueslibox()
