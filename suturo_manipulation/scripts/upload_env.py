#!/usr/bin/env python
import rospy

from giskardpy.python_interface import GiskardWrapper
import giskardpy.utils.tfwrapper as tf


rospy.init_node('evn_uploader')

giskard = GiskardWrapper()
env_urdf = rospy.get_param('kitchen_description')
kitchen_pose = tf.lookup_pose('map', 'iai_kitchen/urdf_main')
giskard.add_urdf(name='iai_kitchen',
                 urdf=env_urdf,
                 pose=kitchen_pose,
                 js_topic='/iai_kitchen/joint_states',
                 set_js_topic='/kitchen/cram_joint_states')