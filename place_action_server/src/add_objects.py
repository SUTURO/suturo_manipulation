#! /usr/bin/env python

import rospy
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion

def stupid_stuff():
    giskard_wrapper = GiskardWrapper()

    giskard_wrapper.clear_world()

    pose = PoseStamped()
    pose.header.frame_id = u'hand_palm_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = Point(0, 0, 0)
    pose.pose.orientation = Quaternion(0, 0.7, 0, 0.7)

    giskard_wrapper.add_cylinder(name=u'grasped_object', size=(0.25, 0.07), pose=pose)
    # TODO: Add object ro gripper
    giskard_wrapper.attach_object(name=u'grasped_object', link_frame_id=u'hand_palm_link')

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('stupid_stuff_py')
        stupid_stuff()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")