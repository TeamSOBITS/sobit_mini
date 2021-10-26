#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniController
from sobit_mini_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    mini_ctr = SobitMiniController(args[0]) # args[0] : C++上でros::init()を行うための引数

    ###  arm controll  ###
    ### arm1    =  1.0 ###
    ### arm2    =  1.0 ###
    ### arm3    =  1.0 ###
    ### arm4    =  1.0 ###
    ### gripper =  1.0 ###
    mini_ctr.moveRightArm( 0.0, 0.0, 1.57, 0.0)

    rospy.sleep(2.0)

    # 決められたポーズをする
    mini_ctr.movePose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
