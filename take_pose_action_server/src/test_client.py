#! /usr/bin/env python
import rospy
import actionlib
from manipulation_action_msgs.msg import TakePoseGoal, TakePoseAction
from geometry_msgs.msg import Point

def test_client():
    client = actionlib.SimpleActionClient('take_pose_server', TakePoseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = TakePoseGoal( pose_mode= TakePoseGoal.LOOK_HIGH) #change values here to test different states/modes

    goal.gaze_point = Point(0.5, 0.0, 0.8)

    goal.head_pan_joint = 0.0
    goal.head_tilt_joint = 0.0
    goal.arm_lift_joint = 0.0
    goal.arm_flex_joint = 0.0
    goal.arm_roll_joint = 0.0
    goal.wrist_flex_joint = 0.0
    goal.wrist_roll_joint = 0.0

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
