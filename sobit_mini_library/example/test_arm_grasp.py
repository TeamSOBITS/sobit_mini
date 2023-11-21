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

    ang = math.radians(-10)
    ###     arm controll      ###
    ### shoulder_roll =  0.00 ###
    ### shoulder_flex =  0.00 ###
    ### elbow_roll    =  1.57 ###
    ### hand_motor    =  0.00 ###

    #逆運動学による物体の把持。
    #mini_ctr.moveGripperToTargetTF(arm_mode(0:left_arm,1:right_arm), 把持したい物体のtf名, 追加パラメータx, 追加パラメータy, 追加パラメータz)
    #追加パラメータは基本的に0.0。細かい調整で使用
    mini_ctr.moveJoint( Joint.HEAD_TILT_JOINT, ang, 2.0, True )
    rospy.sleep(2.0)
    mini_ctr.moveGripperToTargetTF(0, "object_0", 0.0, 0.0, 0.0)
    rospy.sleep(2.0)

    #物体を少し上に持ち上げる
    #mini_ctr.moveLeftArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool)
    #secはジョイントを動かす速度を決定する。最大1.0sec。それ以上は制御が難しい
    #boolは基本True
    mini_ctr.moveLeftArm( math.radians(90), math.radians(-90), math.radians(30), math.radians(60), 0.0, 2.0, True)

    #arm_modeが1のとき
    #mini_ctr.moveRightArm( math.radians(90), -1.57, math.radians(30), math.radians(60), 0.0, 2.0, True)

    #20cm後進
    mini_ctr.controlWheelLinear(-0.2)
    # rospy.sleep(1.0)
    mini_ctr.moveLeftArm( math.radians(0), math.radians(-90), math.radians(90), math.radians(0), 0.0, 2.0, True)
    
    rospy.sleep(2.0)

    # 決められたポーズをする
    # mini_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
