#include <ros/ros.h>
#include <math.h>
#include "sobit_mini_library/sobit_mini_joint_controller.hpp"

double radians(double deg){
    double result = deg * M_PI / 180.0;

    return result;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sobit_mini_test_control_head");

    sobit_mini::SobitMiniJointController mini_joint_ctrl;

    double ang = radians(-10.0);

    // Move camera pan tilt
    mini_joint_ctrl.moveJoint( sobit_mini::HEAD_PAN_JOINT, ang, 2.0, true);
    // mini_joint_ctrl.moveJoint( sobit_mini::HEAD_TILT_JOINT, ang, 2.0, true);

    ros::Duration(2.0).sleep();

    //Strike a certain pose
    mini_joint_ctrl.moveToPose( "initial_pose" );

    return 0;
}