#! /usr/bin/env python
import rospy
from giskardpy.python_interface import GiskardWrapper
from giskardpy import tfwrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

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
q = quaternion_from_euler(0, 0, 4.715)
p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
giskard_wrapper.add_urdf(name='lab', urdf=_urdf, js_topic='/kitchen/joint_states', pose=p)
