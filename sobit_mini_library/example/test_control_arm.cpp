#include <ros/ros.h>
#include <math.h>
#include "sobit_mini_library/sobit_mini_joint_controller.hpp"

int main( int argc, char *argv[]){
    ros::init(argc, argv, "sobit_mini_test_control_arm");

    sobit_mini::SobitMiniJointController mini_joint_ctrl;

    //Move the right arm joints
    //mini_joint_ctrl.moveRightArm( shoulder_roll, shoulder_flex, elbow_roll, wrist_tilt, hand_motor, sec, bool);
    //sec determines the speed at which the joint is moved. Maximum 1.0 sec. Any more than that is difficult to control
    //bool is basically true
    mini_joint_ctrl.moveRightArm(0.0, -1.25, 1.0, 1.0, 0.5, 2.0, true);

    //Move the left arm joints
    mini_joint_ctrl.moveLeftArm(0.0, -1.25, 1.0, 1.0, 0.5, 2.0, true);

    //Move the body roll joints
    mini_joint_ctrl.moveJoint(sobit_mini::BODY_ROLL_JOINT, 0.5, 3.0, false);

    ros::Duration(2.0).sleep();

    //Strike a certain pose
    mini_joint_ctrl.moveToPose("initial_pose");

    return 0;
}