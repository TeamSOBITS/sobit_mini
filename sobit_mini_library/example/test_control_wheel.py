#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniWheelController
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    mini_wheel_ctr = SobitMiniWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数
    
    # タイヤ車輪をを動かす
    mini_wheel_ctr.controlWheelLinear(1.0)
    mini_wheel_ctr.controlWheelRotateRad(1.57)
    mini_wheel_ctr.controlWheelRotateDeg(-90)

    mini_wheel_ctr.controlWheelLinear(-1.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
