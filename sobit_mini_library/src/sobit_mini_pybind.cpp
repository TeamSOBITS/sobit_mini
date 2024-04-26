#include "sobit_mini_library/sobit_mini_library.h"
#include "sobit_mini_library/sobit_mini_joint_controller.hpp"
#include "sobit_mini_library/sobit_mini_wheel_controller.hpp"

using namespace sobit_mini;
namespace py = pybind11;

PYBIND11_MODULE(sobit_mini_module, m) {
    py::enum_<Joint>( m, "Joint" )
        .value( "L_ARM_SHOULDER_ROLL_JOINT", Joint::L_ARM_SHOULDER_ROLL_JOINT )
        .value( "L_ARM_SHOULDER_PAN_JOINT", Joint::L_ARM_SHOULDER_PAN_JOINT )
        .value( "L_ARM_ELBOW_TILT_JOINT", Joint::L_ARM_ELBOW_TILT_JOINT )
        .value( "L_ARM_WRIST_TILT_JOINT", Joint::L_ARM_WRIST_TILT_JOINT )
        .value( "L_HAND_JOINT", Joint::L_HAND_JOINT )
        .value( "R_ARM_SHOULDER_ROLL_JOINT", Joint::R_ARM_SHOULDER_ROLL_JOINT )
        .value( "R_ARM_SHOULDER_PAN_JOINT", Joint::R_ARM_SHOULDER_PAN_JOINT )
        .value( "R_ARM_ELBOW_ROLL_JOINT", Joint::R_ARM_ELBOW_ROLL_JOINT )
        .value( "R_ARM_WRIST_TILT_JOINT", Joint::R_ARM_WRIST_TILT_JOINT )
        .value( "R_HAND_JOINT", Joint::R_HAND_JOINT )
        .value( "BODY_ROLL_JOINT", Joint::BODY_ROLL_JOINT )
        .value( "HEAD_PAN_JOINT", Joint::HEAD_PAN_JOINT )
        .value( "HEAD_TILT_JOINT", Joint::HEAD_TILT_JOINT )

        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values();
    
    py::class_<SobitMiniWheelController>(m, "SobitMiniWheelController")
        .def( py::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitMiniWheelController::controlWheelLinear, "control Wheel Linear" )
        .def( "controlWheelRotateRad", &SobitMiniWheelController::controlWheelRotateRad, "control Wheel Rotate Rad" )
        .def( "controlWheelRotateDeg", &SobitMiniWheelController::controlWheelRotateDeg, "control Wheel Rotate Deg" );

    py::class_<SobitMiniJointController>(m, "SobitMiniJointController")
        .def( py::init< const std::string& >() )
        .def( "moveJoint", &SobitMiniJointController::moveJoint, "move Joint", 
            py::arg("joint_num"), 
            py::arg("rad"), 
            py::arg("sec") = 5.0, 
            py::arg("is_sleep") = true )
        .def( "moveHeadPanTilt", &SobitMiniJointController::moveHeadPanTilt, "move Head PanTilt", 
            py::arg("pan_rad"), 
            py::arg("tilt_rad"), 
            py::arg("sec") = 5.0, 
            py::arg("is_sleep") = true )
        .def( "moveToPose", &SobitMiniJointController::moveToPose, "move Pose",
            py::arg("pose_name"), 
            py::arg("sec") = 5.0 )
        .def( "moveRightArm", &SobitMiniJointController::moveRightArm, "move Right Arm",
            py::arg( "shoulder_roll" ),
            py::arg( "shoulder_pan" ),
            py::arg( "elbow_tilt" ),
            py::arg( "wrist_tilt" ),
            py::arg( "hand_motor"),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveLeftArm", &SobitMiniJointController::moveLeftArm, "move Left Arm",
            py::arg( "shoulder_roll" ),
            py::arg( "shoulder_pan" ),
            py::arg( "elbow_tilt" ),
            py::arg( "wrist_tilt" ),
            py::arg( "hand_motor"),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveGripperToTargetCoord", &SobitMiniJointController::moveGripperToTargetCoord, "moveGripperToTargetCoord",
            py::arg( "arm_mode" ),
            py::arg( "hand_rad" ),
            py::arg( "goal_position_x" )     , py::arg( "goal_position_y" )     , py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ), py::arg( "diff_goal_position_y" ), py::arg( "diff_goal_position_z" ),
            py::arg( "sec" ) = 5.0, py::arg( "is_sleep" ) = true )
        .def( "moveGripperToTargetTF", &SobitMiniJointController::moveGripperToTargetTF, "moveGripperToTargetTF",
            py::arg( "arm_mode"),
            py::arg( "target_name" ),
            py::arg( "hand_rad" ),
            py::arg( "diff_goal_position_x" ), py::arg( "diff_goal_position_y" ), py::arg( "diff_goal_position_z" ),
            py::arg( "sec" ) = 5.0, py::arg( "is_sleep" ) = true );
}