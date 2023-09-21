#!/usr/bin/env python
import csv
import math
import os
import time
from copy import deepcopy
from enum import Enum
from math import sin
from pprint import pprint
from typing import Dict

import scipy

import numpy as np
import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3, Pose
from matplotlib import pyplot as plt
from numpy import savetxt, asarray

from giskardpy.python_interface import GiskardWrapper


class Robots(Enum):
    hsrb = 'hsrb',
    donbot = 'iai_donbot'


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


class FunctionWrapper:
    def __init__(self, robot, plan, execute):

        self.robot = robot
        self.plan = plan
        self.execute = execute

        self.hand_gripper_tool_frame = None
        if self.robot == 'hsrb':
            self.hand_gripper_tool_frame = 'hand_gripper_tool_frame'

        elif self.robot == 'iai_donbot':
            self.hand_gripper_tool_frame = 'gripper_tool_frame'

    def execute_goal(self):
        if self.plan:
            _giskard_wrapper.allow_all_collisions()
            if self.execute:
                _giskard_wrapper.plan_and_execute(wait=True)
            else:
                _giskard_wrapper.plan(wait=True)

    def set_base_position(self):
        joints = {'head_pan_joint': 0.0,
                  'head_tilt_joint': 0.0,
                  'arm_lift_joint': 0.0,
                  'arm_flex_joint': 0.0,
                  'arm_roll_joint': 0.0,
                  'wrist_flex_joint': -1.57,
                  'wrist_roll_joint': 0.0,
                  'hand_motor_joint': 0.8}

        _giskard_wrapper.set_joint_goal(joints)

        self.execute_goal()

    def take_pose(self, pose_keyword='park'):
        _giskard_wrapper.take_pose(pose_keyword=pose_keyword)

        _giskard_wrapper.allow_all_collisions()
        self.execute_goal()

    def move_gripper(self, gripper_state: str):
        if self.execute:
            _giskard_wrapper.move_gripper(gripper_state=gripper_state)

            self.execute_goal()

    def reaching(self, context, name, shape, pose=None, size=None, root='map', tip='hand_gripper_tool_frame'):
        _giskard_wrapper.reaching(context=context,
                                  object_name=name,
                                  object_shape=shape,
                                  goal_pose=pose,
                                  object_size=size,
                                  root_link=root,
                                  tip_link=tip)

        self.execute_goal()

    def align_height(self, context, name, pose, object_height=0.0, root_link='map', tip_link='hand_palm_link'):
        _giskard_wrapper.align_height(context=context, object_name=name, goal_pose=pose, object_height=object_height,
                                      tip_link=tip_link)

        self.execute_goal()

    def placing(self, context, pose=None, tip_link='hand_gripper_tool_frame'):
        _giskard_wrapper.placing(context=context, goal_pose=pose, tip_link=tip_link)

        self.execute_goal()

    def vertical_motion(self, context, distance=0.02, root='base_link', tip_link='hand_gripper_tool_frame'):
        _giskard_wrapper.vertical_motion(context=context, distance=distance, root_link=root, tip_link=tip_link)

        self.execute_goal()

    def retracting(self, name='', distance=0.2, reference_frame='base_link', root='map', tip='hand_palm_link',
                   velocity=0.2):
        _giskard_wrapper.retract(object_name=name,
                                 distance=distance,
                                 reference_frame=reference_frame,
                                 root_link=root,
                                 tip_link=tip,
                                 velocity=velocity)

        self.execute_goal()

    def sequence_goal(self, goals):
        _giskard_wrapper.sequence_goal(motion_sequence=goals)

        self.execute_goal()

    def mixing(self, mixing_time=40):
        _giskard_wrapper.mixing(mixing_time=mixing_time)

        self.execute_goal()

    def joint_rotation_continuous(self,
                                  joint_name: str,
                                  joint_center: float,
                                  joint_range: float,
                                  trajectory_length: float = 20,
                                  target_speed: float = 1,
                                  period_length: float = 1.0):
        _giskard_wrapper.joint_rotation_continuous(joint_name=joint_name,
                                                   joint_center=joint_center,
                                                   joint_range=joint_range,
                                                   trajectory_length=trajectory_length,
                                                   target_speed=target_speed,
                                                   period_length=period_length)

        self.execute_goal()

    def tilting(self, tilt_direction='right', tilt_angle=None, tip_link='wrist_roll_joint'):
        _giskard_wrapper.tilting(tilt_direction=tilt_direction,
                                 tilt_angle=tilt_angle,
                                 tip_link=tip_link)

        self.execute_goal()

    def open_environment(self, tip_link: str,
                         environment_link: str,
                         tip_group=None, environment_group=None, goal_joint_state=None, velocity=0.2):
        _giskard_wrapper.open_environment(tip_link=tip_link,
                                          environment_link=environment_link,
                                          tip_group=tip_group,
                                          environment_group=environment_group,
                                          goal_joint_state=goal_joint_state,
                                          velocity=velocity)

        self.execute_goal()

    def test_goal(self, goal_name, **kwargs):
        _giskard_wrapper.test_goal(goal_name=goal_name, **kwargs)

        self.execute_goal()

    def cart_goal(self, pose, tip, root):
        _giskard_wrapper.set_cart_goal(goal_pose=pose, tip_link=tip, root_link=root)

        self.execute_goal()


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


class ObjectWrapper:

    def __init__(self):
        self.poses: Dict[str, PoseStamped] = self.initiate_poses()
        self.sizes: Dict[str, Vector3] = self.initiate_sizes()

        self.contexts = self.initiate_contexts()
        self.sequences = self.initiate_sequences()

    def initiate_poses(self):
        poses = {}

        test_point_name = 'test1'
        test_point_1 = Point(x=0, y=0, z=0)
        test_pose_1 = Pose(position=test_point_1)
        test_pose_stamped_1 = PoseStamped(pose=test_pose_1)

        poses[test_point_name] = test_pose_stamped_1

        return poses

    def initiate_sizes(self):
        sizes = {}

        test_size_1_name = 'test1'
        test_size_1 = Vector3(x=0, y=0, z=0)

        sizes[test_size_1_name] = test_size_1

        return sizes

    # FIXME Write function to convert PoseStamped to anything
    def convert_to_point_stamped(self, value):
        # if isinstance(value, PoseStamped()):
        #    return PointStamped()
        pass

    def initiate_contexts(self):
        contexts = {}

        # Declare keys
        grasp_default_name = 'grasp_default'
        grasp_above_name = 'grasp_above'
        place_default_name = 'place_default'
        place_above_name = 'place_above'
        open_default_name = 'open_door'
        slip_door_name = 'slip_door'

        # Declare values
        grasp_default = {'action': 'grasping'}

        grasp_above = {'action': 'grasping',
                       'from_above': True,
                       'vertical_align': False}

        place_default = {'action': 'placing',
                         'from_above': False}

        place_above = {'action': 'placing',
                       'from_above': True}

        context_door = {'action': 'door-opening'}

        slip_door = {'action': 'door-opening',
                     'slip_door': -0.0065}

        # Add to dict
        contexts[grasp_default_name] = grasp_default
        contexts[grasp_above_name] = grasp_above
        contexts[place_default_name] = place_default
        contexts[place_above_name] = place_above
        contexts[open_default_name] = context_door
        contexts[slip_door_name] = slip_door

        return contexts

    def initiate_sequences(self):
        sequences = {}

        # Supported goal names
        align_height = 'AlignHeight'
        grasp_frontal = 'Reaching'
        lift = 'VerticalMotion'
        retract = 'Retracting'
        take_pose = 'TakePose'

        # Seuence args
        align_height_kwargs_1 = {'context': self.contexts['grasp_default'],
                                 'object_name': '',
                                 'goal_pose': self.poses['test1'],
                                 'root_link': 'map',
                                 'tip_link': 'hand_palm_link'}

        grasp_frontal_kwargs_1 = {'context': self.contexts['grasp_default'],
                                  'object_name': '',
                                  'goal_pose': self.poses['test1'],
                                  'object_size': self.sizes['test1'],
                                  'root_link': 'map',
                                  'tip_link': 'hand_palm_link'}

        lift_kwargs_1 = {'context': self.contexts['grasp_default'],
                         'distance': 0.02,
                         'root_link': 'base_link',
                         'tip_link': 'hand_palm_link'}
        lift_kwargs_2 = {'context': self.contexts['grasp_default'],
                         'distance': 0.04,
                         'root_link': 'base_link',
                         'tip_link': 'hand_palm_link'}

        retracting_kwargs_1 = {'object_name': '',
                               'distance': 0.05,
                               'root_link': 'map',
                               'tip_link': 'base_link'}

        retracting_kwargs_2 = {'object_name': '',
                               'distance': 0.05,
                               'root_link': 'map',
                               'tip_link': 'hand_palm_link'}

        take_pose_kwargs_1 = {'pose_keyword': 'park'}

        # Create sequence
        align_and_grasp = [{align_height: align_height_kwargs_1},
                           {grasp_frontal: grasp_frontal_kwargs_1}]

        lift_retract = [{lift: lift_kwargs_1},
                        {retract: retracting_kwargs_2}]

        lift_lift = [{lift: lift_kwargs_1},
                     {lift: lift_kwargs_2}]

        grasp_and_lift = [{grasp_frontal: grasp_frontal_kwargs_1},
                          {lift: lift_kwargs_1}]

        align_grasp_lift_retract_park = [{align_height: align_height_kwargs_1},
                                         {grasp_frontal: grasp_frontal_kwargs_1},
                                         {lift: lift_kwargs_1},
                                         {retract: retracting_kwargs_2},
                                         {take_pose: take_pose_kwargs_1}]

        everything = [{lift: lift_kwargs_1},
                      {retract: retracting_kwargs_1},
                      {lift: lift_kwargs_1},
                      {lift: lift_kwargs_2},
                      {align_height: align_height_kwargs_1},
                      {grasp_frontal: grasp_frontal_kwargs_1}]

        sequences['align_grasp'] = align_and_grasp,
        sequences['lift_retract'] = lift_retract,
        sequences['lift_lift'] = lift_lift,
        sequences['grasp_lift'] = grasp_and_lift,
        sequences['reaching_sequence'] = align_grasp_lift_retract_park,
        sequences['everything'] = everything,

        return sequences


def run_test():
    test_wrapper = FunctionWrapper(robot=Robots.hsrb, plan=True, execute=True)
    objects = ObjectWrapper()

    def grasp_object(context, name, pose, size, root, tip):
        test_wrapper.align_height(context=context, name=name, pose=pose, tip_link=tip)

        test_wrapper.reaching(context=context, name=name, shape='', pose=pose, size=size, root=root, tip=tip)

        test_wrapper.move_gripper('close')

        test_wrapper.vertical_motion(context=context, distance=0.03, tip_link=tip)

        test_wrapper.retracting(tip=tip)

        test_wrapper.take_pose(pose_keyword='park')

    def serve_breakfast(sequencegoal=False):
        root_map = 'map'
        starting_pose = PoseStamped()
        # TODO: Set starting pose

        # Go to starting pose
        test_wrapper.cart_goal(pose=starting_pose, root=root_map, tip=test_wrapper.hand_gripper_tool_frame)

        if sequencegoal:
            pass
        else:

            target_pose = PoseStamped
            target_size = Vector3()
            grasping_context = objects.contexts['grasp_default']

            grasp_object(context=grasping_context, name='', pose=target_pose, size=target_size,
                         tip=test_wrapper.hand_gripper_tool_frame, root='map')

    def storing_groceries(sequencegoal=False):
        root_map = 'map'
        starting_pose = PoseStamped()
        # TODO: Set starting pose

        # Go to starting pose
        test_wrapper.cart_goal(pose=starting_pose, root=root_map, tip=test_wrapper.hand_gripper_tool_frame)

        if sequencegoal:
            pass
        else:

            target_pose = PoseStamped
            grasping_context = objects.contexts['grasp_default']
            test_wrapper.align_height(context=grasping_context, name='', pose=target_pose,
                                      tip_link=test_wrapper.hand_gripper_tool_frame)

    def reset_base(base_pose, gripper_state='neutral'):
        test_wrapper.set_base_position()
        test_wrapper.cart_goal(base_pose, tip='base_link', root='map')
        test_wrapper.move_gripper(gripper_state)

    def open_door_plan():
        test_wrapper.reaching(context=ctx, name=shelf_handle_name, shape='')
        time.sleep(1)
        test_wrapper.move_gripper('close')
        time.sleep(2)
        test_wrapper.open_environment(tip_link=test_wrapper.hand_gripper_tool_frame, environment_link=shelf_handle_name,
                                      goal_joint_state=-1.1)
        time.sleep(1)
        test_wrapper.move_gripper('open')

    def prepare_place_object_plan():
        placing_pose = PoseStamped()
        placing_pose.header.frame_id = 'map'
        placing_pose.pose.position = Point(x=1.7, y=1.3, z=0.78)

        places_object_size = Vector3(x=0, y=0, z=0.0)

        # Grasp Object
        # test_wrapper.align_height(ctx, name='', pose=placing_pose, object_height=0.1)
        test_wrapper.reaching(context=ctx, name='', shape='', pose=placing_pose, size=places_object_size)

    def place_object_plan():

        placing_floor_pose = PoseStamped()
        placing_floor_pose.header.frame_id = 'map'
        placing_floor_pose.pose.position = Point(x=1.7, y=1.3, z=0.72)

        time.sleep(3)
        test_wrapper.move_gripper('close')
        time.sleep(2)
        test_wrapper.vertical_motion(context={'action': 'grasping'}, distance=0.03)
        time.sleep(2)
        test_wrapper.placing(context=objects.contexts['place_default'], pose=placing_floor_pose)
        time.sleep(2)
        test_wrapper.move_gripper('open')

    def giskard_ghost_loop():
        test_wrapper.take_pose('perceive')
        time.sleep(1)
        test_wrapper.take_pose('park')
        hsr_test_sim = PoseStamped()
        hsr_test_sim.header.frame_id = 'map'
        hsr_test_sim.pose.position = Point(x=0.7, y=0.2, z=0.7)
        test_wrapper.reaching(ctx, '', '', pose=hsr_test_sim, tip='hand_palm_link', root='base_link',
                              size=Vector3(0, 0, 0))
        time.sleep(1)
        test_wrapper.take_pose('park')

    def align_two_planes():
        table_sim_pose = PoseStamped()
        table_sim_pose.pose.position = Point(1.0, 1.55, 0.5)

        test_wrapper.reaching(ctx, name='', shape='', pose=table_sim_pose, size=Vector3(), tip='hand_palm_link',
                              root='map')

        goal_normx = Vector3Stamped()
        goal_normx.header.frame_id = 'map'
        goal_normx.vector.y = 1

        tip_normx = Vector3Stamped()
        tip_normx.header.frame_id = 'hand_palm_link'
        tip_normx.vector.z = 1

        _giskard_wrapper.allow_all_collisions()
        _giskard_wrapper.set_align_planes_goal(goal_normal=goal_normx, tip_link='hand_palm_link', tip_normal=tip_normx,
                                               root_link='arm_flex_link')

        goal_normz = Vector3Stamped()
        goal_normz.header.frame_id = 'map'
        goal_normz.vector.x = -1

        tip_normz = Vector3Stamped()
        tip_normz.header.frame_id = 'hand_palm_link'
        tip_normz.vector.z = 1

        _giskard_wrapper.set_align_planes_goal(goal_normal=goal_normz, tip_link='hand_gripper_tool_frame',
                                               tip_normal=tip_normz,
                                               root_link='arm_flex_link')

        goal_norm_up = Vector3Stamped()
        goal_norm_up.header.frame_id = 'map'
        goal_norm_up.vector.z = 1

        tip_norm_up = Vector3Stamped()
        tip_norm_up.header.frame_id = 'hand_palm_link'
        tip_norm_up.vector.x = 1
        _giskard_wrapper.set_align_planes_goal(goal_normal=goal_norm_up, tip_link='hand_palm_link',
                                               tip_normal=tip_norm_up,
                                               root_link='arm_flex_link')
        _giskard_wrapper.plan_and_execute()

    this_test_pose = PoseStamped()
    this_test_pose.header.frame_id = 'map'
    this_test_pose.pose.position = Vector3(x=0.7, y=1.0, z=0.75)

    # sequence = all_sequences['reaching_sequence']
    # sequence = all_sequences['lift_retract']
    sequence = objects.sequences['align_grasp']

    start = Point(x=2.7, y=0.17, z=0.0)
    start_pose = PoseStamped()
    start_pose.pose.position = start
    # start_pose = objects.poses['test1']
    test_context = objects.contexts['grasp_default']
    # test_wrapper.sequence_goal(sequence)

    start_hsr_door = PoseStamped()
    start_hsr_door.header.frame_id = 'map'
    start_hsr_door.pose.position = Point(x=2.7, y=0.17, z=0.0)

    start_hsr_table = PoseStamped()
    start_hsr_table.header.frame_id = 'map'
    start_hsr_table.pose.position = Point(x=1.995, y=0.7, z=0.0)
    start_hsr_table.pose.orientation = Quaternion(x=0, y=0, z=0.703, w=0.71)

    # ctx = objects.contexts['slip_door']
    ctx = objects.contexts['grasp_default']
    shelf_handle_name = 'iai_kitchen/shelf:shelf:shelf_door_left:handle'
    ex = False
    # ex = True

    # plan = 'door'
    plan = 'place'

    # RESET
    # reset_base(start_hsr_door)

    # test_wrapper.set_base_position()

    # EXECUTION
    if ex:
        time.sleep(3)

        if plan == 'door':

            test_wrapper.move_gripper('close')
            time.sleep(2)
            test_wrapper.open_environment(tip_link=test_wrapper.hand_gripper_tool_frame,
                                          environment_link=shelf_handle_name,
                                          goal_joint_state=-1.1, velocity=0.1)

            time.sleep(2)
            test_wrapper.move_gripper('open')

            # open_door_plan()
            # reset_pose = start_hsr_door
            # reset_base(base_pose=reset_pose)
        elif plan == 'place':
            prepare_place_object_plan()
            # place_object_plan()
        else:
            return

        # time.sleep(5)

    test_wrapper.mixing(mixing_time=30)


def read_force_torque_data(filename, topic_names=False, trim_data=True):
    data_range_path = os.path.expanduser('~/ForceTorqueData/' + filename)

    if topic_names:
        seq_path = '/hsrb/wrist_wrench/compensated/header/seq'
        force_path = '/hsrb/wrist_wrench/compensated/wrench/force/'
        torque_path = '/hsrb/wrist_wrench/compensated/wrench/torque/'
    else:
        seq_path = 'seq'
        force_path = '_force'
        torque_path = '_torque'

    with open(data_range_path) as data_range:
        reader = csv.DictReader(data_range)
        wrist_seq, wrist_stamp, x_force, y_force, z_force, x_torque, y_torque, z_torque = [], [], [], [], [], [], [], []
        for row in reader:
            if row[seq_path] != '':
                wrist_seq.append(row[seq_path])
                x_force.append(round(eval(row['x' + force_path]), 5))
                y_force.append(round(eval(row['y' + force_path]), 5))
                z_force.append(round(eval(row['z' + force_path]), 5))
                x_torque.append(round(eval(row['x' + torque_path]), 5))
                y_torque.append(round(eval(row['y' + torque_path]), 5))
                z_torque.append(round(eval(row['z' + torque_path]), 5))

        if trim_data:
            x_force = np.trim_zeros(x_force)
            y_force = np.trim_zeros(y_force)
            z_force = np.trim_zeros(z_force)
            x_torque = np.trim_zeros(x_torque)
            y_torque = np.trim_zeros(y_torque)
            z_torque = np.trim_zeros(z_torque)

        if not x_force.index(x_force[0]) == y_force.index(y_force[0]) == z_force.index(z_force[0]):
            print('Starting index not identical!!!')

        fig, ax = plt.subplots(1, 2)

        ax[0].set_title('Force')
        # ax[0].plot(x_force_filtered, 'r', label='x_filtered')
        ax[0].plot(x_force, 'r', label='x_trimmed')
        ax[0].plot(y_force, 'g', label='y')
        ax[0].plot(z_force, 'b', label='z')
        ax[0].legend()
        ax[1].set_title('Torque')
        ax[1].plot(x_torque, 'r', label='x')
        ax[1].plot(y_torque, 'g', label='y')
        ax[1].plot(z_torque, 'b', label='z')
        ax[1].legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    _giskard_wrapper = GiskardWrapper()

    # set_base_position()

    run_test()

    last_filtered = 'filtered.csv'
    last_unfiltered = 'unfiltered.csv'
    hsr_place_1 = 'place_object_hsr.csv'
    hsr_door_slipped_1 = 'slipped_door_hsr.csv'
    # read_force_torque_data(last_unfiltered)
