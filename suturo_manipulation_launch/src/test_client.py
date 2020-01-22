#! /usr/bin/env python
import rospy
import actionlib
from manipulation_action_msgs.msg import MoveGripperGoal, MoveGripperAction
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from giskardpy import tfwrapper
from tf.transformations import *



def test_client():
    client = actionlib.SimpleActionClient('move_gripper_server', MoveGripperAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = u'map'
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position = Point(10, 0, 0)
    pose.pose.orientation = Quaternion(0, 0, 0, 0)



    goal = MoveGripperGoal(goal_pose= pose)

# Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_client_py')
#        print(tfwrapper.lookup_transform(u'map',u'hand_palm_link'))
        #result = test_client()
        #print("Result:", result)
        t1 = tfwrapper.lookup_transform('map', 'base_footprint')
        t2 = tfwrapper.lookup_transform('map', 'hand_palm_link')
        print("map", "base_footprint", t1)
        print("map", "hand_palm_link", t2)
        t1.transform.rotation.x
        q1 = [t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w]
        q2 = [t2.transform.rotation.x, t2.transform.rotation.y, -t2.transform.rotation.z, t2.transform.rotation.w]

        q3 = quaternion_multiply(q1, q2)

        print(q3)

        q_new = [0, -1, 0, 0]
        q_rot = [0, 0, 1, 0]

        q_old = quaternion_multiply(q_rot, q_new)
        print(q_old)


    except rospy.ROSInterruptException:
        print("program interrupted before completion")