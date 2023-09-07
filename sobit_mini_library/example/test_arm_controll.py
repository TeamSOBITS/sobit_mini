#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniController
from sobit_mini_module import Joint
from geometry_msgs.msg import Point
import sys
import math

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    mini_ctr = SobitMiniController(args[0]) # args[0] : C++上でros::init()を行うための引数

    ###     arm controll      ###
    ### shoulder_roll =  0.00 ###
    ### shoulder_flex =  0.00 ###
    ### elbow_roll    =  1.57 ###
    ### hand_motor    =  0.00 ###

    #右腕を動かす
    #mini_ctr.moveRightArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool)
    #secはジョイントを動かす速度を決定する。最大1.0sec。それ以上は制御が難しい。
    #boolは基本True
    mini_ctr.moveRightArm( 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, True)

    #左腕を動かす
    # mini_ctr.moveLeftArm( 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, True)

    #腰を動かす
    # mini_ctr.moveJoint( Joint.BODY_ROLL_JOINT, 1.0, 3.0, False)
    
    rospy.sleep(2.0)

    # 決められたポーズをする
    mini_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass