#! /usr/bin/env python



# uint8 ARROW=0
# uint8 CUBE=1
# uint8 SPHERE=2
# uint8 CYLINDER=3
#
# string object_id                # ID of the object
# string frame_name               # Frame name in TF tree
# string object_type              # Object type IRI
# int32 shape                     # Approximate shape
# string mesh_path                # Path to mesh or empty string
# std_msgs/ColorRGBA color        # Material color [0.0-1.0]
# geometry_msgs/Vector3 size      # Bounding box
# geometry_msgs/PoseStamped pose  # Pose of the object
# # static transforms of affordances, etc.
# geometry_msgs/TransformStamped[] static_transforms

import rospy
from knowrob_objects.msg import ObjectStateArray, ObjectState
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('object_state_test_publisher')
    pub = rospy.Publisher('/knowledgebase/object_state', ObjectStateArray, queue_size=10)

    rospy.sleep(5)

    object_state = ObjectState()
    object_state.object_id = 'lel'
    object_state.shape = ObjectState.SPHERE
    object_state.size = Vector3(0.5, 0.5, 0.5)

    quat = quaternion_from_euler(0,0,0)

    pose = PoseStamped()
    pose.header.frame_id = u'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = Point(1, 1, 2.5)
    pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

    object_state.pose = pose

    object_state2 = ObjectState()
    object_state2.object_id = 'wow'
    object_state2.shape = ObjectState.CYLINDER
    object_state2.size = Vector3(0.5, 0.1, 1)

    quat2 = quaternion_from_euler(0, 0, 0)

    pose2 = PoseStamped()
    pose2.header.frame_id = u'map'
    pose2.header.stamp = rospy.Time.now()
    pose2.pose.position = Point(-2, 1, 0)
    pose2.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

    object_state2.pose = pose2

    object_state_array = ObjectStateArray()
    object_state_array.action = ObjectStateArray.DELETE
    object_state_array.object_states = [object_state, object_state2]

    pub.publish(object_state_array)

    print("published:")#, object_state_array)

