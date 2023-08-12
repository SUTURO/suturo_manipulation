#!/usr/bin/env python
import csv
import os
import time
from copy import deepcopy
from math import sin
from pprint import pprint
import scipy

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3, Pose
from matplotlib import pyplot as plt
from numpy import savetxt, asarray

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


def open_gripper():
    print('Open Gripper')
    _giskard_wrapper.move_gripper(gripper_state='open')
    _giskard_wrapper.plan_and_execute(wait=True)


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


def test_new_feature(rad=0.0,
                     direction='',
                     plan=True,
                     execute=True,
                     **kwargs):
    sequence = 'SequenceGoal'
    mixing = 'Mixing1'
    open = 'OpenHandlelessCart'

    test_goal = open

    pose_1 = PoseStamped()
    pose_1.pose.position.x = 0.0
    pose_1.pose.position.y = 0.5
    pose_1.pose.position.z = 0.7

    pose_2 = PoseStamped()
    pose_2.pose.position.x = -1.0
    pose_2.pose.position.y = 1.0
    pose_2.pose.position.z = 0.7

    circle_point = Point(x=1.3, y=0.7, z=0.7)
    circle_point_stamped = PointStamped(point=circle_point)

    obj_size = Vector3(0.1, 0.2, 0.1)

    _giskard_wrapper.test_goal(goal_name=test_goal,
                               temp_position=circle_point_stamped,
                               temp_size=obj_size,
                               opening_radius=rad,
                               door_opening_direction=direction,
                               offset=0.0,
                               tip_link_name='hand_palm_link')

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def set_base_position(plan=True, execute=True):
    joints = {'head_pan_joint': 0.0,
              'head_tilt_joint': 0.0,
              'arm_lift_joint': 0.0,
              'arm_flex_joint': 0.0,
              'arm_roll_joint': 1.4,
              'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0.14,
              'hand_motor_joint': 0.8}

    _giskard_wrapper.set_joint_goal(joints)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def align_height(context, name, pose, height, tip_link='hand_palm_link', plan=True, execute=True):
    _giskard_wrapper.align_height(context=context, object_name=name, goal_pose=pose, height=height, tip_link=tip_link)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def lifting(context, distance=0.02, root='base_link', tip_link='hand_gripper_tool_frame', plan=True, execute=True):
    _giskard_wrapper.lift_object(context=context, distance=distance, root_link=root, tip_link=tip_link)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def placing(context, pose=None, tip_link='hand_palm_link', execute=True):
    _giskard_wrapper.placing(context=context, goal_pose=pose, tip_link=tip_link)

    if execute:
        _giskard_wrapper.plan_and_execute(wait=True)
    else:
        _giskard_wrapper.plan(wait=True)


def reaching(context, name, shape, pose=None, size=None, root='map', tip='hand_gripper_tool_frame', plan=True,
             execute=True):
    _giskard_wrapper.reaching(context=context,
                              object_name=name,
                              object_shape=shape,
                              goal_pose=pose,
                              object_size=size,
                              root_link=root,
                              tip_link=tip)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def retracting(name='', distance=0.2, reference_frame='base_link', root='map', tip='hand_palm_link', velocity=0.2,
               plan=True,
               execute=True):
    _giskard_wrapper.retract(object_name=name,
                             distance=distance,
                             reference_frame=reference_frame,
                             root_link=root,
                             tip_link=tip,
                             velocity=velocity)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def sequence_goal(goals, plan=True, execute=True):
    goal_names = []
    goal_args = []

    '''for goal_name, kwargs in goals:
        goal_names.append(goal_name)
        goal_args.append(kwargs)'''

    _giskard_wrapper.sequence_goal(motion_sequence=goals)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


this_test_pose = PoseStamped()
this_test_pose.pose.position = Vector3(x=2.64, y=0.317, z=0.75)

this_test_size = Vector3(x=0, y=0, z=0.05)


def prepare_sequences():
    lift = 'VerticalMotion'
    retract = 'Retracting'
    align_height = 'AlignHeight'
    grasp_frontal = 'GraspObject'

    context_grasp = {'action': 'grasping'}

    lift_kwargs_1 = {'context': context_grasp,
                     'distance': 0.02,
                     'root_link': 'base_link',
                     'tip_link': 'hand_gripper_tool_frame'}
    lift_kwargs_2 = {'context': context_grasp,
                     'lifting': 0.04,
                     'root_link': 'base_link',
                     'tip_link': 'hand_gripper_tool_frame'}

    retracting_kwargs_1 = {'object_name': '',
                           'distance': 0.05,
                           'root_link': 'map',
                           'tip_link': 'base_link'}

    retracting_kwargs_2 = {'object_name': '',
                           'distance': 0.05,
                           'root_link': 'map',
                           'tip_link': 'hand_gripper_tool_frame'}

    align_height_kwargs_1 = {'object_name': '',
                             'goal_pose': this_test_pose,
                             'object_height': 0.0,
                             'height_only': True,
                             'from_above': False,
                             'root_link': 'map',
                             'tip_link': 'hand_gripper_tool_frame'}

    grasp_frontal_kwargs_1 = {'object_name': '',
                              'goal_pose': this_test_pose,
                              'object_size': this_test_size,
                              'frontal_grasping': True,
                              'root_link': 'odom',
                              'tip_link': 'hand_gripper_tool_frame'}
    lift_retract = {lift: lift_kwargs_1,
                    retract: retracting_kwargs_2}

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


def move_gripper(gripper_state: str, plan=True, execute=True):
    _giskard_wrapper.move_gripper(gripper_state=gripper_state)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def mixing(center, radius=0.1, scale=1.0, tip_link='hand_palm_link', mixing_time=40, plan=True, execute=True):
    _giskard_wrapper.mixing(center=center,
                            radius=radius,
                            scale=scale,
                            mixing_time=mixing_time,
                            tip_link=tip_link)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def take_pose(pose_keyword='park', plan=True, execute=True):
    _giskard_wrapper.take_pose(pose_keyword=pose_keyword)
    _giskard_wrapper.allow_all_collisions()
    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def test_goal(goal_name, plan=True, execute=True, **kwargs):
    _giskard_wrapper.test_goal(goal_name=goal_name, **kwargs)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def tilting(tilt_direction='right', tilt_angle=None, tip_link='wrist_roll_joint', plan=True, execute=True):
    _giskard_wrapper.tilting(tilt_direction=tilt_direction,
                             tilt_angle=tilt_angle,
                             tip_link=tip_link)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def open_environment(tip_link: str,
                     environment_link: str,
                     tip_group=None, environment_group=None, goal_joint_state=None, plan=True, execute=True):
    _giskard_wrapper.open_environment(tip_link=tip_link,
                                      environment_link=environment_link,
                                      tip_group=tip_group,
                                      environment_group=environment_group,
                                      goal_joint_state=goal_joint_state)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def run_test():
    objects, positions = prepare_variables()

    all_sequences = prepare_sequences()

    object_name = 'muesli_medium'
    position_name = 'LabEnv_table'
    tf_name = 'Shelf_OGTVKLRY'
    test_object = get_entity(object_name, objects)
    test_position: Position = get_entity(position_name, positions)

    local_point = Point(x=2.35, y=0.5, z=0.8)
    local_pose = Pose(position=local_point)

    local_pose_stamped = PoseStamped(pose=local_pose)
    object_size = Vector3(x=0.1, y=0.05, z=0.2)

    root_point_floor = Point(x=0.94, y=0.0, z=0.1)
    root_pose_floor = Pose(position=root_point_floor)
    root_pose_stamped_floor = PoseStamped(pose=root_pose_floor)

    sequence = all_sequences['lift_retract']

    context_grasping = {'action': 'grasping',
                        'from_above': False,
                        'vertical_align': False}

    context_placing = {'action': 'placing',
                       'from_above': True}

    context_door = {'action': 'door-opening'}

    center_point = PointStamped()
    center_point.header.frame_id = 'hand_gripper_tool_frame'
    center_point.point.x += 0.04

    # retracting()

    name = ''
    shape = ''

    # align_height(context, name=name, pose=local_pose_stamped, height=object_size.z, execute=False)
    # reaching(context=context, name=name, shape=shape, pose=local_pose_stamped, size=object_size, execute=False)

    # Sequencegoals

    # sequence_goal(sequence, execute=False)

    # time.sleep(3)
    # align_height(name='', pose=this_test_pose, height=0.0, from_above=True, execute=True)
    # _giskard_wrapper.allow_all_collisions()
    # reaching(context=context_grasping, name='iai_kitchen/drawer:drawer:drawer_knob', shape='', execute=False)# pose=root_pose_stamped_floor, size=object_size, execute=True)

    # circle_point_vec = Vector3(x=1.3, y=0.7, z=0.7)
    circle_point = Point(x=1.5, y=-1.5, z=0.8)
    circle_pose = Pose(position=circle_point)
    circle_pose_stamped = PoseStamped(pose=circle_pose)
    obj_size = Vector3(0.1, 0.2, 0.1)

    table_pose_stamped = deepcopy(circle_pose_stamped)
    table_pose_stamped.pose.position.z = 0.7

    # align_height(context_grasping, name=name, pose=circle_pose_stamped, height=obj_size.z, execute=True)
    # reaching(context=context_grasping, name='', shape='', pose=circle_pose_stamped, size=obj_size, tip='hand_palm_link', execute=True)
    placing(context=context_grasping, pose=table_pose_stamped)
    move_gripper('open')

    # reaching(context_door, name='iai_kitchen/shelf:shelf:shelf_door_left:handle', shape='')
    # open_environment(tip_link='hand_gripper_tool_frame', environment_link='iai_kitchen/shelf:shelf:shelf_door_left:handle', goal_joint_state=-0.3)

    # Test new feature
    # test_new_feature(rad=0.3, direction='right', execute=False)
    # test_new_feature(rad=0.5, direction='left', execute=False)

    # move_gripper(gripper_state='neutral')

    # mixing(execute=False, center=center_point, radius=0.1, scale=1.0, tip_link='hand_palm_link', mixing_time=10)

    # lifting(context=context_placing, distance=0.1, execute=True)

    # retracting(distance=0.3, velocity=0.2, execute=True)

    # take_pose(pose_keyword='park', execute=True)

    # move_gripper(gripper_state='neutral', execute=True)
    # move_gripper(gripper_state='close', execute=True)

    # tilting(tilt_direction='right')

    # open_environment(tip_link='hand_gripper_tool_frame', environment_link='shelf:shelf:shelf_door_left:handle', goal_joint_state=-0.2, execute=True)


def read_force_torque_data(filename):
    data_range_path = os.path.expanduser('~/ForceTorqueData/' + filename)
    seq = 'seq'
    seq_path = '/hsrb/wrist_wrench/compensated/header/seq'
    force_path = '/hsrb/wrist_wrench/compensated/wrench/force/'
    torque_path = '/hsrb/wrist_wrench/compensated/wrench/torque/'
    force = '_force'
    torque = '_torque'

    with open(data_range_path) as data_range:
        reader = csv.DictReader(data_range)
        wrist_seq, wrist_stamp, x_force, y_force, z_force, x_torque, y_torque, z_torque = [], [], [], [], [], [], [], []
        for row in reader:
            if row[seq] != '':
                wrist_seq.append(row[seq])
                #wrist_stamp.append(row['/hsrb/wrist_wrench/compensated/header/stamp'])
                x_force.append(round(eval(row['x' + force]), 5))
                y_force.append(round(eval(row['y' + force]), 5))
                z_force.append(round(eval(row['z' + force]), 5))
                x_torque.append(round(eval(row['x' + torque]), 5))
                y_torque.append(round(eval(row['y' + torque]), 5))
                z_torque.append(round(eval(row['z' + torque]), 5))

        x_force_trimmed = np.trim_zeros(x_force)
        y_force_trimmed = np.trim_zeros(y_force)
        z_force_trimmed = np.trim_zeros(z_force)
        x_torque_trimmed = np.trim_zeros(x_torque)
        y_torque_trimmed = np.trim_zeros(y_torque)
        z_torque_trimmed = np.trim_zeros(z_torque)

        if not x_force.index(x_force_trimmed[0]) == y_force.index(y_force_trimmed[0]) == z_force.index(z_force_trimmed[0]):
            print('Starting index not identical!!!')



        fig, ax = plt.subplots(1, 2)

        ax[0].set_title('Force')
        # ax[0].plot(x_force_filtered, 'r', label='x_filtered')
        ax[0].plot(x_force_trimmed, 'r', label='x_trimmed')
        ax[0].plot(y_force_trimmed, 'g', label='y')
        ax[0].plot(z_force_trimmed, 'b', label='z')
        ax[0].legend()
        ax[1].set_title('Torque')
        ax[1].plot(x_torque_trimmed, 'r', label='x')
        ax[1].plot(y_torque_trimmed, 'g', label='y')
        ax[1].plot(z_torque_trimmed, 'b', label='z')
        ax[1].legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    _giskard_wrapper = GiskardWrapper()

    # set_base_position()

    # run_test()


    last_filtered = 'filtered.csv'
    last_unfiltered = 'unfiltered.csv'
    hsr_place_1 = 'place_object_hsr.csv'
    hsr_door_slipped_1 = 'slipped_door_hsr.csv'
    read_force_torque_data(last_filtered)

    '''import time
    while True:
        print(f"{'hand_gripper_tool_frame' in _giskard_wrapper.get_group_info(group_name='hsrb').links}  {rospy.get_rostime()}")
        # print(_giskard_wrapper.get_group_names())
        time.sleep(1.0)'''
