#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Vector3, Pose

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
    _giskard_wrapper.move_gripper(True)
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
              'wrist_roll_joint': 0.14}

    _giskard_wrapper.set_joint_goal(joints)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def align_height(context, name, pose, height, plan=True, execute=True):
    _giskard_wrapper.align_height(context=context, object_name=name, goal_pose=pose, height=height)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def lifting(name='', distance=0.02, root='base_link', tip='hand_palm_link', plan=True, execute=True):
    _giskard_wrapper.lift_object(object_name=name, lifting=distance, root_link=root, tip_link=tip)

    if plan:
        if execute:
            _giskard_wrapper.plan_and_execute(wait=True)
        else:
            _giskard_wrapper.plan(wait=True)


def reaching(context, name, shape, pose, size, root='map', tip='hand_gripper_tool_frame', plan=True, execute=True):
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


def retracting(name='', distance=0.2, reference_frame='base_link', root='map', tip='hand_palm_link', plan=True,
               execute=True):
    _giskard_wrapper.retract(object_name=name,
                             distance=distance,
                             reference_frame=reference_frame,
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


def move_gripper(open_gripper, plan=True, execute=True):

    _giskard_wrapper.move_gripper(open_gripper=open_gripper)

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


def take_pose(plan=True, execute=True):
    _giskard_wrapper.take_pose()

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


def run_test():
    objects, positions = prepare_variables()

    all_sequences = prepare_sequences()

    object_name = 'muesli_medium'
    position_name = 'LabEnv_table'
    tf_name = 'Shelf_OGTVKLRY'
    test_object = get_entity(object_name, objects)
    test_position: Position = get_entity(position_name, positions)

    local_point = Point(x=2.5, y=0.33, z=0.8)
    local_pose = Pose(position=local_point)

    local_pose_stamped = PoseStamped(pose=local_pose)
    object_size = Vector3(x=0.1, y=0.05, z=0.2)

    sequence = all_sequences['lift_retract']

    context = {'action': 'placing',
               'from_above': False}

    center_point = PointStamped()
    center_point.header.frame_id = 'hand_gripper_tool_frame'
    center_point.point.x += 0.04

    # retracting()

    name = ''
    shape = ''

    # align_height(context, name=name, pose=local_pose_stamped, height=object_size.z, execute=True)

    # reaching(context=context, name=name, shape=shape, pose=local_pose_stamped, size=object_size, execute=False)

    # Sequencegoals

    # sequence_goal(sequence)

    # align_height(name='', pose=this_test_pose, height=0.0, from_above=True, execute=True)
    # reaching(name='', pose=this_test_pose, size=this_test_size, from_above=True, execute=False)

    # Test new feature
    # test_new_feature(tf_name, test_position.pose, test_object.size, grasp=True, lift_first=False)

    # move_gripper(False)

    # mixing(execute=True, center=center_point, radius=0.1, scale=4.0, tip_link='hand_palm_link', mixing_time=5)

    # lifting()

    # retracting(reference_frame='hand_palm_link')

    #take_pose()

    move_gripper(open_gripper=False, execute=True)


if __name__ == '__main__':
    rospy.init_node('milestone0_server')
    _giskard_wrapper = GiskardWrapper()

    # set_base_position()

    run_test()

    '''import time
    while True:
        print(f"{'hand_gripper_tool_frame' in _giskard_wrapper.get_group_info(group_name='hsrb').links}  {rospy.get_rostime()}")
        # print(_giskard_wrapper.get_group_names())
        time.sleep(1.0)'''

