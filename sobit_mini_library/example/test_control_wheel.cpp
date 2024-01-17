#include <ros/ros.h>
#include <math.h>
#include "sobit_mini_library/sobit_mini_wheel_controller.hpp"

int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_edu_test_control_wheel" );

    sobit_mini::SobitMiniWheelController mini_wheel_ctrl;

    //Move the tire wheel
    mini_wheel_ctrl.controlWheelLinear(1.0);
    mini_wheel_ctrl.controlWheelRotateRad(1.57);
    ros::Duration(2.0).sleep();
    mini_wheel_ctrl.controlWheelRotateDeg(-90);

    mini_wheel_ctrl.controlWheelLinear(-1.0);

    return 0;
}