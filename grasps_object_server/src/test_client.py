#! /usr/bin/env python
import rospy
import actionlib
from manipulation_action_msgs.msg import GraspAction, GraspGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point

from giskardpy import world

def test_client():

    client = actionlib.SimpleActionClient('grasps_server', GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    goal = GraspGoal()

    goal.object_size.x = 0.25
    goal.object_size.y = 0.07
    goal.object_size.z = 0.07
    goal.grasp_mode = goal.FRONT

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = u'map'
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position = Point(0.5, 1,0.3)
    pose.pose.orientation = Quaternion(0, 0, 0, 1)

    goal.object_frame_id = 'test'

    goal.goal_pose = pose

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
        result = test_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

'''
if __name__ == '__main__':
        rospy.init_node('fixed_tf_Broadcaster')
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            br.sendTransform((0.0, 2.0, 0.0),
                             (0.0, 0.0, 0.0, 1.0),
                                rospy.Time.now(),
                                "test_tf",
                                "hand_palm_link")
            rate.sleep()
'''
