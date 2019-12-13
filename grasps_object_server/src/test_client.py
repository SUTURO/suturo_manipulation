#! /usr/bin/env python
import rospy
import actionlib
import tf
from manipulation_action_msgs.msg import GraspAction, GraspActionGoal


def test_client():

    '''
    client = actionlib.SimpleActionClient('grasps_object_server', GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    #goal = GraspActionGoal(object_frame_id="")

    # Sends the goal to the action server.
    #client.send_goal()

    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    # Prints out the result of executing the action
    #return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_client_py')
        result = test_client()
        #print("Result:", result)
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
