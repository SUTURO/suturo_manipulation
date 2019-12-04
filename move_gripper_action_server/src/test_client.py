#! /usr/bin/env python
import rospy
import actionlib
import move_gripper_action_server.msg
import geometry_msgs

def test_client():
    client = actionlib.SimpleActionClient('move_gripper_server', move_gripper_action_server.msg.MoveGripperAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 10.0
    pose.pose.position.y = 5.0
    pose.pose.position.z = 0.25

    goal = move_gripper_action_server.msg.MoveGripperGoal(goal_pose= pose)
    

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_client_py')
        result = test_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")