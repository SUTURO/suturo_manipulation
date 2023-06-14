#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Vector3

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
    mueslibox_center.pose.position.z = 0.9

    # medium muesli size
    medium_muesli_size = Vector3()
    medium_muesli_size.x = 0.14
    medium_muesli_size.y = 0.062
    medium_muesli_size.z = 0.22

    # Spoon size
    spoon_size = Vector3()
    spoon_size.x = 0.06
    spoon_size.y = 0.02
    spoon_size.z = 0.04

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
    spoon = Box(name='spoon', size=spoon_size)

    objects = []
    objects.append(medium_muesli)
    objects.append(big_muesli)
    objects.append(drawer_knob)
    objects.append(spoon)

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

    _giskard_wrapper.plan_and_execute(wait=True)
    # _giskard_wrapper.plan()
    '''
    # Attach Object
    #_giskard_wrapper.update_parent_link_of_group(name, tip_link)

    print('Grabbing Object')
    _giskard_wrapper.move_gripper(False)
    _giskard_wrapper.plan_and_execute(wait=True)
    
    # Lift Object
    print('Lifting Object')
    new_start = PointStamped()
    new_start.point.z = 0.3
    
    _giskard_wrapper.lift_object(object_name=name)
    _giskard_wrapper.plan_and_execute(wait=True)
    
    # Retract
    print('Retracting')
    _giskard_wrapper.retract(object_name=object_name, distance=0.1)
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
                 radius=0.0,
                 root_link='map',
                 tip_link='hand_palm_link',
                 height_only=True,
                 frontal=True):
    # Align height
    '''_giskard_wrapper.align_height(object_name=name,
                                  object_pose=pose,
                                  height=height,
                                  tip_link=tip_link,
                                  height_only=height_only)
    _giskard_wrapper.plan_and_execute(wait=True)'''

    # Place Object
    _giskard_wrapper.place_object(object_name=name,
                                  goal_pose=pose,
                                  object_height=height,
                                  radius=radius,
                                  root_link=root_link,
                                  tip_link=tip_link,
                                  frontal=frontal)
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
    _giskard_wrapper.plan_and_execute()


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
    # add_object(name=name, pose=pose, size=size)

    sequence = 'SequenceGoal'
    force_sensor = 'TestForceSensorGoal'

    test_goal = sequence

    # pose.pose.position.x = 1.48
    # pose.pose.position.y = -0.979
    # pose.pose.position.z = 0.7

    pose_1 = PoseStamped()
    pose_1.pose.position.x = 0.5
    pose_1.pose.position.y = 0.0
    pose_1.pose.position.z = 0.7

    pose_2 = PoseStamped()
    pose_2.pose.position.x = -1.0
    pose_2.pose.position.y = 1.0
    pose_2.pose.position.z = 0.7

    place_object_args = {'object_name': '',
                         'target_pose': pose_1,
                         'object_height': 0.0}

    lifting1_point = PointStamped()
    lifting1_point.header.frame_id = 'hand_palm_link'
    lifting1_point.point.x += 0.05

    retracting1_point = PointStamped()
    retracting1_point.header.frame_id = 'hand_palm_link'
    retracting1_point.point.z -= 0.1

    object_name_args = {'object_name': '', 'lifting': 0.05}

    retract_args = {'object_name': '',
                    'distance': 0.1,
                    'tip_link': 'base_link'}

    object_name_args_2 = {'object_name': 'second',
                          'lifting': 0.06}

    align_height_args = {'object_name': name,
                         'goal_pose': pose_1,
                         'object_height': 0.0,
                         'height_only': True}

    grasp_object_args = {'object_name': name,
                         'object_pose': pose,
                         'object_size': size}

    goal_type_seq = ['AlignHeight', 'GraspObject']
    kwargs_seq = [align_height_args, grasp_object_args]

    _giskard_wrapper.test_goal(test_goal,
                               object_name=name,
                               object_pose_1=pose_1,
                               object_pose_2=pose_2,
                               grasp_object=grasp,
                               lift_first=lift_first,
                               goal_type_seq=goal_type_seq,
                               kwargs_seq=kwargs_seq)

    # _giskard_wrapper.retract(object_name='')

    # _giskard_wrapper.add_cmd()

    # _giskard_wrapper.lift_object(object_name='')

    # _giskard_wrapper.move_gripper(open_gripper=False)

    # joints = {'arm_lift_joint': 0.0}

    # _giskard_wrapper.set_joint_goal(goal_state=joints)

    # _giskard_wrapper.align_height(object_pose=test_position.pose, height=size.z)

    # Move gripper, theoretically
    # joints = {'hand_motor_joint': 1.0}
    # _giskard_wrapper.set_joint_goal(goal_state=joints)

    _giskard_wrapper.plan_and_execute()
    # _giskard_wrapper.plan()


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    _giskard_wrapper = GiskardWrapper()
    objects, positions = prepare_variables()

    object_name = 'spoon'
    position_name = 'simulation_test_pose'
    tf_name = 'Shelf_OGTVKLRY'
    test_object = get_entity(object_name, objects)
    test_position: Position = get_entity(position_name, positions)

    print(test_object.size)
    print(test_position.pose)

    #pick_object(name=tf_name, pose=test_position.pose, size=test_object.size)

    # create object
    # add_object(name=tf_name, pose=test_position.pose, size=test_object.size)

    # place object
    rad = 0.0
    #place_object(name=object_name, pose=test_position.pose, height=test_object.height, radius=rad, height_only=True)

    #set_base_position()

    # Drawer
    #pick_object(name='', pose=test_position.pose, size=test_object.size)

    # Gripper
    test_new_feature(tf_name, test_position.pose, test_object.size, grasp=True, lift_first=False)

    # giskard_wrapper.lift_object(object_name='')

    # goal_p = PoseStamped()
    # goal_p.pose.position.x = 0.08
    # goal_p.pose.position.y = 1.75
    # goal_p.pose.position.z = 0.725

    # height = 0.22
    # _giskard_wrapper.place_object(object_name='', goal_pose=goal_p, object_height=height)
    # _giskard_wrapper.tilt()
    # _giskard_wrapper.move_gripper(open_gripper=False)

    # _giskard_wrapper.align_height(object_name='', object_pose=test_position.pose, height=test_object.height)

    # _giskard_wrapper.plan_and_execute()
