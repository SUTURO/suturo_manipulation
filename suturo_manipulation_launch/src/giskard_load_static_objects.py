#! /usr/bin/env python
import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion

rospy.init_node('add_static_giskard')

_urdf = rospy.get_param('kitchen_description', False)
while not _urdf:
    rospy.sleep(5)
    _urdf = rospy.get_param('kitchen_description', False)
print(_urdf)
giskard_wrapper = GiskardWrapper()
giskard_wrapper.remove_object('lab')
p = PoseStamped()
p.header.stamp = rospy.Time.now()
p.header.frame_id = u'map'
p.pose.position = Point(0, 0, 0)
p.pose.orientation = Quaternion(0, 0, 0, 1)
giskard_wrapper.add_urdf(name='lab', urdf=_urdf, js_topic='/kitchen/joint_states', pose=p)
