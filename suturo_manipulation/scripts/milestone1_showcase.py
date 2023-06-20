#!/usr/bin/env python
import time
from typing import List

import hsrb_interface
import rospy
import math  # for sin and cos
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3

from giskardpy.python_interface import GiskardWrapper


class TestEntity:
    def __init__(self):
        self.name = None
        self.size = None

    def get_name(self):
        return self.name


class Box(TestEntity):
    def __init__(self,
                 name: str,
                 size: Vector3):
        super().__init__()
        self.name = name
        self.size = size
        self.height = size.z


class Position:
    def __init__(self,
                 name: str,
                 pose: PoseStamped):
        super().__init__()
        self.name = name
        self.pose = pose

    def get_name(self):
        return self.name


def get_entity(ent_name: str,
               ent_list: [TestEntity],
               ):
    for ent in ent_list:
        if ent.get_name() == ent_name:
            return ent


def prepare_variables():
    # Objects
    # mueslibox position
    mueslibox_center = PoseStamped()
    mueslibox_center.header.frame_id = 'map'
    mueslibox_center.pose.position.x = - 0.1
    mueslibox_center.pose.position.y = 1.68
    mueslibox_center.pose.position.z = 0.7

    # medium muesli size
    medium_muesli_size = Vector3()
    medium_muesli_size.x = 0.14
    medium_muesli_size.y = 0.062
    medium_muesli_size.z = 0.22

    # big muesli size
    big_muesli_size = Vector3()
    big_muesli_size.x = 0.095
    big_muesli_size.y = 0.19
    big_muesli_size.z = 0.26

    # drawer position
    knob_pose = PoseStamped()
    knob_pose.header.frame_id = 'map'
    knob_pose.pose.position.x = 0.18
    knob_pose.pose.position.y = -0.225
    knob_pose.pose.position.z = 0.282

    # drawer size
    knob_size = Vector3()
    knob_size.x = 0.04
    knob_size.y = 0.1
    knob_size.z = 0.2

    medium_muesli = Box(name='muesli_medium', size=medium_muesli_size)
    big_muesli = Box(name='muesli_big', size=big_muesli_size)
    drawer_knob = Box(name='drawer_knob', size=knob_size)

    objects = []
    objects.append(medium_muesli)
    objects.append(big_muesli)
    objects.append(drawer_knob)

    # Single Positions
    # Simulation test pose
    Sim_test_pose_quaternion = Quaternion(x=0.0, y=0.0, z=0.087, w=0.996)
    Sim_test_pose = mueslibox_center
    Sim_test_pose.pose.orientation = Sim_test_pose_quaternion

    # LabEnv table 1
    LabEnv_table_pose = PoseStamped()
    LabEnv_table_pose.header.frame_id = 'map'
    LabEnv_table_pose.pose.position.x = 1.6
    LabEnv_table_pose.pose.position.y = -0.9
    LabEnv_table_pose.pose.position.z = 0.81
    LabEnv_table_pose.pose.orientation = Sim_test_pose_quaternion

    # LabEnv top shelf
    LabEnv_shelf_pose = PoseStamped()
    LabEnv_shelf_pose.header.frame_id = 'map'
    LabEnv_shelf_pose.pose.position.x = -0.092
    LabEnv_shelf_pose.pose.position.y = 1.65
    LabEnv_shelf_pose.pose.position.z = 0.75

    simulation_testing = Position('simulation_test_pose', Sim_test_pose)
    LabEnv_table = Position('LabEnv_table', LabEnv_table_pose)
    LabEnv_shelf = Position('LabEnv_shelf', LabEnv_shelf_pose)

    positions = []
    positions.append(simulation_testing)
    positions.append(LabEnv_table)
    positions.append(LabEnv_shelf)

    return objects, positions


def pick_object(name: str,
                pose: PoseStamped,
                size: Vector3,
                root_link='map',
                tip_link='hand_palm_link',
                object_type='box'):
    '''
    # Open Gripper
    open_gripper()

    # Add object to giskard
    print('Add Object')
    add_object(name=name, pose=pose, size=size)

    # print('Point Object')
    # _giskard_wrapper.set_pointing(goal_pose=pose, root_link=root_link, tip_link=tip_link)
    # _giskard_wrapper.plan_and_execute(wait=True)
    '''
    # Pick object
    print('Getting in position')
    _giskard_wrapper.grasp_object(object_name=name,
                                  object_pose=pose,
                                  object_size=size,
                                  root_link=root_link,
                                  tip_link=tip_link)

    # _giskard_wrapper.plan_and_execute(wait=True)
    # _giskard_wrapper.plan()
    '''
    # Attach Object
    #_giskard_wrapper.update_parent_link_of_group(name, tip_link)

    print('Grabbing Object')
    _giskard_wrapper.move_gripper(False)
    _giskard_wrapper.plan_and_execute(wait=True)
    
    # Lift Object
    print('Lifting Object')
    _giskard_wrapper.lift_object(object_name=name)
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
    _giskard_wrapper.align_height(object_name='', object_pose=pose, height=0.0)
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
    _giskard_wrapper.update_parent_link_of_group(name, root_link)
    _giskard_wrapper.avoid_collision(min_distance=0.01, group1=_giskard_wrapper.robot_name, group2=name)
    _giskard_wrapper.retract(object_name=name)

    _giskard_wrapper.plan_and_execute(wait=True)

    set_base_position()


def add_object(name: str,
               pose: PoseStamped,
               size: Vector3,
               object_type='box'):
    height = 0.259
    radius = 0.0395

    ### Will be removed with knowledge synchronization ###
    if name in _giskard_wrapper.get_group_names():
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

    _giskard_wrapper.plan_and_execute(wait=True)


def test_new_feature(name: str,
                     pose: PoseStamped,
                     size, grasp, lift_first):
    sequence = 'SequenceGoal'

    test_goal = sequence

    pose_1 = PoseStamped()
    pose_1.pose.position.x = 0.0
    pose_1.pose.position.y = 0.5
    pose_1.pose.position.z = 0.7

    pose_2 = PoseStamped()
    pose_2.pose.position.x = -1.0
    pose_2.pose.position.y = 1.0
    pose_2.pose.position.z = 0.7

    _giskard_wrapper.plan_and_execute()


def set_base_position(plan=True, execute=True):
    joints = {'head_pan_joint': 0.0,
              'head_tilt_joint': 0.0,
              'arm_lift_joint': 0.0,
              'arm_flex_joint': 0.0,
              'arm_roll_joint': 1.4,
              'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0.14
              }
    _giskard_wrapper.set_joint_goal(joints)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def align_height(name, pose, height, from_above=False, plan=True, execute=True):
    _giskard_wrapper.align_height(object_name=name, object_pose=pose, height=height, from_above=from_above)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def reach(name, pose, size, root='map', tip='hand_palm_link', from_above=False, plan=True, execute=True):
    _giskard_wrapper.reach(object_name=name,
                           object_pose=pose,
                           object_size=size,
                           root_link=root,
                           tip_link=tip,
                           from_above=from_above)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def retracting(name, distance, root='map', tip='base_link', plan=True, execute=True):
    _giskard_wrapper.retract(object_name=name,
                             distance=distance,
                             root_link=root,
                             tip_link=tip)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def sequence_goal(goals, plan=True, execute=True):
    goal_names = []
    goal_args = []

    for goal_name, kwargs in goals:
        goal_names.append(goal_name)
        goal_args.append(kwargs)

    _giskard_wrapper.sequence_goal(goal_names, goal_args)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


this_test_pose = PoseStamped()
this_test_pose.pose.position = Vector3(x=2.64, y=0.317, z=0.75)

this_test_size = Vector3(x=0, y=0, z=0.05)


def prepare_sequences():
    lift = 'LiftObject'
    retract = 'Retracting'
    align_height = 'AlignHeight'
    grasp_frontal = 'GraspObject'

    lift_kwargs_1 = {'object_name': '',
                     'lifting': 0.02,
                     'root_link': 'base_link',
                     'tip_link': 'hand_gripper_tool_frame'}
    lift_kwargs_2 = {'object_name': '',
                     'lifting': 0.04,
                     'root_link': 'base_link',
                     'tip_link': 'hand_gripper_tool_frame'}

    retracting_kwargs_1 = {'object_name': '',
                           'distance': 0.05,
                           'root_link': 'map',
                           'tip_link': 'base_link'}

    align_height_kwargs_1 = {'object_name': '',
                             'goal_pose': this_test_pose,
                             'object_height': 0.0,
                             'height_only': True,
                             'from_above': False,
                             'root_link': 'map',
                             'tip_link': 'hand_gripper_tool_frame'}

    grasp_frontal_kwargs_1 = {'object_name': '',
                              'object_pose': this_test_pose,
                              'object_size': this_test_size,
                              'frontal_grasping': True,
                              'root_link': 'odom',
                              'tip_link': 'hand_gripper_tool_frame'}
    lift_retract = [(lift, lift_kwargs_1),
                    (retract, retracting_kwargs_1)]

    lift_lift = [(lift, lift_kwargs_1),
                 (lift, lift_kwargs_2)]

    align_and_grasp = [(align_height, align_height_kwargs_1),
                       (grasp_frontal, grasp_frontal_kwargs_1)]

    everything = [(lift, lift_kwargs_1),
                  (retract, retracting_kwargs_1),
                  (lift, lift_kwargs_1),
                  (lift, lift_kwargs_2),
                  (align_height, align_height_kwargs_1),
                  (grasp_frontal, grasp_frontal_kwargs_1)]

    all_sequences = {'lift_retract': lift_retract,
                     'lift_lift': lift_lift,
                     'align_grasp': align_and_grasp,
                     'everything': everything}

    return all_sequences


def run_test():
    objects, positions = prepare_variables()

    all_sequences = prepare_sequences()

    object_name = 'muesli_medium'
    position_name = 'LabEnv_table'
    tf_name = 'Shelf_OGTVKLRY'
    test_object = get_entity(object_name, objects)
    test_position: Position = get_entity(position_name, positions)

    sequence = all_sequences['lift_retract']

    # pick_object(name=tf_name, pose=test_position.pose, size=test_object.size)

    # place object
    # place_object(name=object_name, pose=test_position.pose, height=test_object.height)

    # Sequencegoals

    # sequence_goal(sequence)

    # align_height(name='', pose=this_test_pose, height=0.0, from_above=True, execute=True)
    # reach(name='', pose=this_test_pose, size=this_test_size, from_above=True, execute=False)

    # Test new feature
    # test_new_feature(tf_name, test_position.pose, test_object.size, grasp=True, lift_first=False)


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    _giskard_wrapper = GiskardWrapper()

    set_base_position()

    # run_test()

    _giskard_wrapper.plan_and_execute()
    # _giskard_wrapper.plan(wait=True)
