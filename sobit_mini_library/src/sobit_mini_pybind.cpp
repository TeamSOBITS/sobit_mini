#include <sobit_mini_library/sobit_mini_controller.hpp>

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
        // .value( "HEAD_PAN_JOINT", Joint::HEAD_PAN_JOINT )
        // .value( "HEAD_TILT_JOINT", Joint::HEAD_TILT_JOINT )
        // .value( "BODY_ROLL_JOINT", Joint::BODY_ROLL_JOINT )
        // .value( "RIGHT_SHOULDER_ROLL_JOINT", Joint::RIGHT_SHOULDER_ROLL_JOINT )
        // .value( "RIGHT_SHOULDER_FLEX_JOINT", Joint::RIGHT_SHOULDER_FLEX_JOINT )
        // .value( "RIGHT_ELBOW_ROLL_JOINT", Joint::RIGHT_ELBOW_ROLL_JOINT )
        // .value( "RIGHT_HAND_MOTOR_JOINT", Joint::RIGHT_HAND_MOTOR_JOINT )
        // .value( "LEFT_SHOULDER_ROLL_JOINT", Joint::LEFT_SHOULDER_ROLL_JOINT )
        // .value( "LEFT_SHOULDER_FLEX_JOINT", Joint::LEFT_SHOULDER_FLEX_JOINT )
        // .value( "LEFT_ELBOW_ROLL_JOINT", Joint::LEFT_ELBOW_ROLL_JOINT )
        // .value( "LEFT_HAND_MOTOR_JOINT", Joint::LEFT_HAND_MOTOR_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values();
    
    py::class_<SobitTurtlebotController>(m, "SobitTurtlebotController")
        .def( py::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitTurtlebotController::controlWheelLinear, "control Wheel Linear" )
        .def( "controlWheelRotateRad", &SobitTurtlebotController::controlWheelRotateRad, "control Wheel Rotate Rad" )
        .def( "controlWheelRotateDeg", &SobitTurtlebotController::controlWheelRotateDeg, "control Wheel Rotate Deg" );

    py::class_<SobitMiniController, SobitTurtlebotController>(m, "SobitMiniController")
        .def( py::init< const std::string& >() )
        .def( "moveJoint", &SobitMiniController::moveJoint, "move Joint", 
            py::arg("joint_num"), py::arg("rad"), py::arg("sec"), py::arg("is_sleep") = true )
        .def( "moveHeadPanTilt", &SobitMiniController::moveHeadPanTilt, "move Head PanTilt", 
            py::arg("pan_rad"), py::arg("tilt_rad"), py::arg("sec"), py::arg("is_sleep") = true )
        // .def( "movePose", &SobitMiniController::movePose, "move Pose" )
        .def( "moveToPose", &SobitMiniController::moveToPose, "move Pose" )
        .def( "moveRightArm", &SobitMiniController::moveRightArm, "move Right Arm" )
        .def( "moveLeftArm", &SobitMiniController::moveLeftArm, "move Left Arm" );
}