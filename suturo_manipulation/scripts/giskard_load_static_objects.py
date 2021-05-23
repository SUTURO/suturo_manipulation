#! /usr/bin/env python
import rospy
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper

rospy.init_node('add_static_giskard')
giskard_wrapper = GiskardWrapper()
_urdf = rospy.get_param('hsrb_lab', False)
kitchen_frame = rospy.get_param('~environment_frame', 'iai_kitchen/world')
while not _urdf:
    rospy.sleep(5)
    _urdf = rospy.get_param('hsrb_lab', False)
giskard_wrapper.detach_object('gripper_dummy')
giskard_wrapper.remove_object('gripper_dummy')
giskard_wrapper.remove_object('lab')
p = tfwrapper.lookup_pose('map', kitchen_frame)

#giskard_wrapper.attach_box(name='gripper_dummy', size=[0.05, 0.1, 0.1], frame_id="hand_palm_link",
#                           position=[0.05, 0, 0.05], orientation=[0, 0, 0, 1])
giskard_wrapper.add_urdf(name='lab', urdf=_urdf, js_topic='/kitchen/joint_states', pose=p)
