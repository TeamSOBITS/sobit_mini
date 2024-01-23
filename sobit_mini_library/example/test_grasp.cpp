#include <ros/ros.h>
#include <math.h>
#include "sobit_mini_library/sobit_mini_joint_controller.hpp"
#include "sobit_mini_library/sobit_mini_wheel_controller.hpp"

double radians(double deg){
    double result = deg * M_PI / 180.0;

    return result;
}

int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_mini_test_grasp");

    sobit_mini::SobitMiniJointController mini_joint_ctrl;
    sobit_mini::SobitMiniWheelController mini_wheel_ctrl;

    double ang = radians(-10.0);

    // Grasping an object by inverse kinematics
    // mini_joint_ctrl.moveGripperToTargetTF(arm_mode(0:left_arm, 1:right_arm), tf name of the object to be grasped, hand motor radian, Additional parameter x, Additional parameter y, Additional parameter z);
    mini_joint_ctrl.moveJoint( sobit_mini::HEAD_TILT_JOINT, ang, 2.0, true );
    ros::Duration(2.0).sleep();
    mini_joint_ctrl.moveGripperToTargetTF(0, "object_0", 1.0, 0.0, 0.0, 0.0);
    ros::Duration(2.0).sleep();

    // Lift the object slightly upward
    // mini_joint_ctrl.moveLeftArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool);
    // Sec determines the speed at which the joint is moved. Max. 1.0 sec. any more than that is difficult to control
    // Bool is basically True
    mini_joint_ctrl.moveLeftArm( radians(90.0), radians(-90.0), radians(30.0), radians(60.0), 0.0, 2.0, true );

    // When arm_mode is 1
    // mini_joint_ctrl.moveRightArm( radians(90.0), radians(-90.0), radians(30.0), radians(60.0), 0.0, 2.0, true );

    // 20cm backward
    mini_wheel_ctrl.controlWheelLinear(-0.2);
    ros::Duration(1.0).sleep();
    mini_joint_ctrl.moveLeftArm( radians(0.0), radians(-90), radians(90), radians(0.0), 0.0, 2.0, true );
    
    // When arm_mode is 1
    // mini_joint_ctrl.moveRightArm( radians(0.0), radians(-90), radians(90), radians(0.0), 0.0, 2.0, true );

    ros::Duration(2.0).sleep();

    // Strike a certain pose
    // mini_joint_ctrl.moveToPose( "initial_pose" );

    return 0;
}