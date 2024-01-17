#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniJointController,SobitMiniWheelController
from sobit_mini_module import Joint
import sys
import math

def test():
    rospy.init_node('sobit_mini_test_grasp')
    args = sys.argv
    mini_joint_ctrl = SobitMiniJointController(args[0]) # args[0] : Arguments for ros::init() on C++
    mini_wheel_ctrl = SobitMiniWheelController(args[0])

    ang = math.radians(-10)

    #Grasping an object by inverse kinematics
    #mini_joint_ctrl.moveGripperToTargetTF(arm_mode(0:left_arm, 1:right_arm), tf name of the object to be grasped, Additional parameter x, Additional parameter y, Additional parameter z)
    #Additional parameter is basically 0.0, used for fine tuning
    mini_joint_ctrl.moveJoint( Joint.HEAD_TILT_JOINT, ang, 2.0, True )
    rospy.sleep(2.0)
    mini_joint_ctrl.moveGripperToTargetTF(0, "object_0", 0.0, 0.0, 0.0)
    rospy.sleep(2.0)

    #Lift the object slightly upward
    #mini_joint_ctrl.moveLeftArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool)
    #Sec determines the speed at which the joint is moved. Max. 1.0 sec. any more than that is difficult to control
    #Bool is basically True
    mini_joint_ctrl.moveLeftArm( math.radians(90), math.radians(-90), math.radians(30), math.radians(60), 0.0, 2.0, True)

    #When arm_mode is 1
    #mini_joint_ctrl.moveRightArm( math.radians(90), -1.57, math.radians(30), math.radians(60), 0.0, 2.0, True)

    #20cm backward
    mini_wheel_ctrl.controlWheelLinear(-0.2)
    rospy.sleep(1.0)
    mini_joint_ctrl.moveLeftArm( math.radians(0), math.radians(-90), math.radians(90), math.radians(0), 0.0, 2.0, True)
    
    rospy.sleep(2.0)

    #Strike a certain pose
    # mini_joint_ctrl.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
