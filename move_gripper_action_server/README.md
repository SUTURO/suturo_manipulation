# Move Gripper Action Server

This package provides an actionserver which goal is to move the gripper o the robot to a certain position.

The actual calculation of the joint states and the collision avoidance is handled by giskard.

The Action message is defined as follows:

PlaceAction
Platziert das Objekt im Gripper an der gegebenen pose

    goal:
        geometry_msgs/PoseStamped goal_pose // pose in map
    result
        uint8 success = 0; failure = 1
        uint8 error_code
    feedback
        geometry_msgs/TransformStamped tf_object_to_goal //object-pose in goal_frame
        float64 gripper_joint_state
