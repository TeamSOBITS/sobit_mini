#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniController
from sobit_mini_module import Joint
import sys
import math

def test():
    rospy.init_node('test')
    # r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    mini_pantilt_ctr = SobitMiniController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # while not rospy.is_shutdown():
    # ang = -1.0 * ang

    ang = -0.45
    # カメラパンチルトを動かす
    mini_pantilt_ctr.moveJoint( Joint.HEAD_TILT_JOINT, ang, 2.0, True )
    # mini_pantilt_ctr.moveJoint( Joint.BODY_ROLL_JOINT, math.radians(-17), 2.0, True )
    # r.sleep()

    # mini_pantilt_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
