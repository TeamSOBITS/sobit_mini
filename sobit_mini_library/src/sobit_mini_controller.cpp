#include <sobit_mini_library/sobit_mini_controller.hpp>

using namespace sobit;
namespace py = pybind11;

SobitMiniController::SobitMiniController( const std::string &name ) : SobitTurtlebotController( name ) {
    pub_body_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    pub_left_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/left_arm_trajectory_controller/command", 1); 
    pub_right_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/right_arm_trajectory_controller/command", 1);
    loadPose();
}

SobitMiniController::SobitMiniController( ) : SobitTurtlebotController( ) {
    pub_body_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    pub_left_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/left_arm_trajectory_controller/command", 1); 
    pub_right_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/right_arm_trajectory_controller/command", 1);
    loadPose();
}

void SobitMiniController::loadPose() {
    XmlRpc::XmlRpcValue pose_val;
    if ( !nh_.hasParam("/mini_pose") ) return; 
    nh_.getParam("/mini_pose", pose_val);
    int pose_num = pose_val.size();
    pose_list_.clear();
    for ( int i = 0; i < pose_num; i++ ) {
        Pose pose;
        std::vector<double> joint_val(11, 0.0);
        pose.pose_name = static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::HEAD_PAN_JOINT] = static_cast<double>(pose_val[i]["head_pan_joint"]);
        joint_val[Joint::HEAD_TILT_JOINT] = static_cast<double>(pose_val[i]["head_tilt_joint"]);
        joint_val[Joint::BODY_ROLL_JOINT] = static_cast<double>(pose_val[i]["body_roll_joint"]);
        joint_val[Joint::RIGHT_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["right_shoulder_roll_joint"]);
        joint_val[Joint::RIGHT_SHOULDER_FLEX_JOINT] = static_cast<double>(pose_val[i]["right_shoulder_flex_joint"]);
        joint_val[Joint::RIGHT_ELBOW_ROLL_JOINT] = static_cast<double>(pose_val[i]["right_elbow_roll_joint"]);
        joint_val[Joint::RIGHT_HAND_MOTOR_JOINT] = static_cast<double>(pose_val[i]["right_hand_motor_joint"]);
        joint_val[Joint::LEFT_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["left_shoulder_roll_joint"]);
        joint_val[Joint::LEFT_SHOULDER_FLEX_JOINT] = static_cast<double>(pose_val[i]["left_shoulder_flex_joint"]);
        joint_val[Joint::LEFT_ELBOW_ROLL_JOINT] = static_cast<double>(pose_val[i]["left_elbow_roll_joint"]);
        joint_val[Joint::LEFT_HAND_MOTOR_JOINT] = static_cast<double>(pose_val[i]["left_hand_motor_joint"]);                                                             
        pose.joint_val = joint_val;
        pose_list_.push_back( pose );
    }
    return;
}

bool SobitMiniController::moveAllJoint( 
    const double head_pan, 
    const double head_tilt, 
    const double body_roll, 
    const double right_shoulder_roll, 
    const double right_shoulder_flex, 
    const double right_elbow_roll, 
    const double right_hand_motor, 
    const double left_shoulder_roll, 
    const double left_shoulder_flex, 
    const double left_elbow_roll, 
    const double left_hand_motor, 
    const double sec, 
    bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory head_joint_trajectory;
        trajectory_msgs::JointTrajectory body_joint_trajectory;
        trajectory_msgs::JointTrajectory right_arm_joint_trajectory;
        trajectory_msgs::JointTrajectory left_arm_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT], head_pan, sec, &head_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], head_tilt, sec, &head_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::BODY_ROLL_JOINT], body_roll, sec, &body_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_ROLL_JOINT], right_shoulder_roll, sec, &right_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_FLEX_JOINT], right_shoulder_flex, sec, &right_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_ELBOW_ROLL_JOINT], right_elbow_roll, sec, &right_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_HAND_MOTOR_JOINT], right_hand_motor, sec, &right_arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_ROLL_JOINT], left_shoulder_roll, sec, &left_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_FLEX_JOINT], left_shoulder_flex, sec, &left_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_ELBOW_ROLL_JOINT], left_elbow_roll, sec, &left_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_HAND_MOTOR_JOINT], left_hand_motor, sec, &left_arm_joint_trajectory );
        checkPublishersConnection ( pub_head_control_ );
        checkPublishersConnection ( pub_body_control_ );
        checkPublishersConnection ( pub_right_arm_control_ );
        checkPublishersConnection ( pub_left_arm_control_ );
        pub_head_control_.publish( head_joint_trajectory );
        pub_body_control_.publish( body_joint_trajectory );
        pub_right_arm_control_.publish( right_arm_joint_trajectory );
        pub_left_arm_control_.publish( left_arm_joint_trajectory );
        ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniController::moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        if( joint_num < Joint::BODY_ROLL_JOINT ) {
            checkPublishersConnection ( pub_head_control_ );
            pub_head_control_.publish( joint_trajectory );
        } else {
            checkPublishersConnection ( pub_body_control_ );
            pub_body_control_.publish( joint_trajectory );
        }
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniController::moveHeadPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT], pan_rad, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], tilt_rad, sec, &joint_trajectory );
        checkPublishersConnection ( pub_head_control_ );
        pub_head_control_.publish( joint_trajectory );
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniController::movePose( const std::string &pose_name ) {
    bool is_find = false;
    std::vector<double> joint_val;
    for ( auto& pose : pose_list_ ) {
        if ( pose_name != pose.pose_name ) continue;
        is_find = true;
        joint_val = pose.joint_val;
        break;
    }
    if ( is_find ) {
        ROS_INFO("I found a '%s'", pose_name.c_str() );
        return moveAllJoint( 
                            joint_val[HEAD_PAN_JOINT],
                            joint_val[HEAD_TILT_JOINT],
                            joint_val[BODY_ROLL_JOINT],
                            joint_val[RIGHT_SHOULDER_ROLL_JOINT],
                            joint_val[RIGHT_SHOULDER_FLEX_JOINT],
                            joint_val[RIGHT_ELBOW_ROLL_JOINT],
                            joint_val[RIGHT_HAND_MOTOR_JOINT],
                            joint_val[LEFT_SHOULDER_ROLL_JOINT],
                            joint_val[LEFT_SHOULDER_FLEX_JOINT],
                            joint_val[LEFT_ELBOW_ROLL_JOINT],
                            joint_val[LEFT_HAND_MOTOR_JOINT], 
                            5.0 );
    } else {
        ROS_ERROR("'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitMiniController::moveRightArm ( const double shoulder_roll, const double shoulder_flex, const double elbow_roll, const double hand_motor ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_ROLL_JOINT], shoulder_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_FLEX_JOINT], shoulder_flex, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_ELBOW_ROLL_JOINT], elbow_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::RIGHT_HAND_MOTOR_JOINT], hand_motor, 5.0, &joint_trajectory );
        checkPublishersConnection ( pub_right_arm_control_ );
        pub_right_arm_control_.publish( joint_trajectory );
        ros::Duration( 5.0 ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniController::moveLeftArm ( const double shoulder_roll, const double shoulder_flex, const double elbow_roll, const double hand_motor ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_ROLL_JOINT], shoulder_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_FLEX_JOINT], shoulder_flex, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_ELBOW_ROLL_JOINT], elbow_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::LEFT_HAND_MOTOR_JOINT], hand_motor, 5.0, &joint_trajectory );
        checkPublishersConnection ( pub_left_arm_control_ );
        pub_left_arm_control_.publish( joint_trajectory );
        ros::Duration( 5.0 ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}


PYBIND11_MODULE(sobit_mini_module, m) {
    py::enum_<Joint>( m, "Joint" )
        .value( "HEAD_PAN_JOINT", Joint::HEAD_PAN_JOINT )
        .value( "HEAD_TILT_JOINT", Joint::HEAD_TILT_JOINT )
        .value( "BODY_ROLL_JOINT", Joint::BODY_ROLL_JOINT )
        .value( "RIGHT_SHOULDER_ROLL_JOINT", Joint::RIGHT_SHOULDER_ROLL_JOINT )
        .value( "RIGHT_SHOULDER_FLEX_JOINT", Joint::RIGHT_SHOULDER_FLEX_JOINT )
        .value( "RIGHT_ELBOW_ROLL_JOINT", Joint::RIGHT_ELBOW_ROLL_JOINT )
        .value( "RIGHT_HAND_MOTOR_JOINT", Joint::RIGHT_HAND_MOTOR_JOINT )
        .value( "LEFT_SHOULDER_ROLL_JOINT", Joint::LEFT_SHOULDER_ROLL_JOINT )
        .value( "LEFT_SHOULDER_FLEX_JOINT", Joint::LEFT_SHOULDER_FLEX_JOINT )
        .value( "LEFT_ELBOW_ROLL_JOINT", Joint::LEFT_ELBOW_ROLL_JOINT )
        .value( "LEFT_HAND_MOTOR_JOINT", Joint::LEFT_HAND_MOTOR_JOINT )
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
        .def( "movePose", &SobitMiniController::movePose, "move Pose" )
        .def( "moveRightArm", &SobitMiniController::moveRightArm, "move Right Arm" )
        .def( "moveLeftArm", &SobitMiniController::moveLeftArm, "move Left Arm" );
}
