#! /usr/bin/env python
import rospy
import actionlib
from manipulation_action_msgs.msg import PlaceGoal, PlaceAction
from geometry_msgs.msg import PoseStamped, Point, Quaternion


def test_client():
    client = actionlib.SimpleActionClient('place_server', PlaceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = u'map'
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position = Point(0, 0, 0)
    pose.pose.orientation = Quaternion(0, 0, 0, 1)

    goal = PlaceGoal(goal_pose=pose)

    # Sends the goal to the action server.
    client.send_goal(goal)

    #client.cancel_goal()

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