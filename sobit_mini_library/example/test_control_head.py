#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniJointController
from sobit_mini_module import Joint
import sys
import math

def test():
    rospy.init_node('sobit_mini_test_control_head')
    args = sys.argv
    mini_pantilt_ctr = SobitMiniJointController(args[0]) # args[0] : Arguments for ros::init() on C++

    ang = math.radians(-10)
    # Move camera pan tilt
    mini_pantilt_ctr.moveJoint( Joint.HEAD_PAN_JOINT, ang, 2.0, True )
    # mini_pantilt_ctr.moveJoint( Joint.HEAD_TILT_JOINT, ang, 2.0, True )
    rospy.sleep(2.0)

    #Strike a certain pose
    mini_pantilt_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
