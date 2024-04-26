#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniJointController
from sobit_mini_module import Joint
import sys
import math

def test():
    rospy.init_node('sobit_mini_test_control_arm')
    args = sys.argv
    mini_ctr = SobitMiniJointController(args[0]) # args[0] : Arguments for ros::init() on C++

    #Move the right arm joints
    #mini_ctr.moveRightArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool)
    #sec determines the speed at which the joint is moved. Maximum 1.0 sec. Any more than that is difficult to control
    #bool is basically true
    mini_ctr.moveRightArm(0.0, -1.25, 1.0, 1.0, 0.5, 2.0, True)

    #Move the left arm joints
    # mini_ctr.moveLeftArm(0.0, -1.25, 1.0, 1.0, 0.5, 2.0, True)

    #Move the body roll joints
    # mini_ctr.moveJoint( Joint.BODY_ROLL_JOINT, 0.5, 3.0, False)
    
    rospy.sleep(2.0)

    # Strike a certain pose.
    mini_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass