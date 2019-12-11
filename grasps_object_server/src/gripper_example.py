#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2
# TF name of the bottle
_BOTTLE_TF='ar_marker/4000'
# TF name of the gripper
_HAND_TF='hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# Posture that 0.02[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# Posture to move the hand 0.1[m] up
hand_up = geometry.pose(x=0.1)

# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)

# Location of the sofa
sofa_pos = (1.2, 0.4, 1.57)

if __name__=='__main__':

    # Greet
    rospy.sleep(5.0)
    tts.say('Hi. My name is HSR. I will pick the PET bottle at side of the sofa.')
    rospy.sleep(5.0)

    try:
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        tts.say('Fail to initialize.')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # Move to the location where the bottle is viewable
        omni_base.go_abs(sofa_pos[0], sofa_pos[1], sofa_pos[2], _MOVE_TIMEOUT)
    except:
        tts.say('Fail to move.')
        rospy.logerr('fail to move')
        sys.exit()

    try:
        # Transit to initial grasping posture
        whole_body.move_to_neutral()
        # Look at the hand after the transition
        whole_body.looking_hand_constraint = True
        # Move the hand to front of the bottle
        whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        # Specify the force to grasp
        gripper.apply_force(_GRASP_FORCE)
        # Wait time for simulator's grasp hack. Not needed on actual robot
        rospy.sleep(2.0)
        # Move the hand up on end effector coordinate
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # Move the hand back on end effector coordinate
        whole_body.move_end_effector_pose(hand_back, _HAND_TF)
        # Transit to initial posture
        whole_body.move_to_neutral()
    except:
        tts.say('Fail to grasp.')
        rospy.logerr('fail to grasp')
        sys.exit()
