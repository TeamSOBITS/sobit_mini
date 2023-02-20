#include <sobit_mini_library/sobit_mini_controller.hpp>

using namespace sobit_mini;

SobitMiniController::SobitMiniController( const std::string &name ) : SobitTurtlebotController( name ) {
    pub_body_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    // pub_left_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/left_arm_trajectory_controller/command", 1);
    pub_l_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_trajectory_controller/command", 1);  
    // pub_right_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/right_arm_trajectory_controller/command", 1);
    pub_r_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_trajectory_controller/command", 1);
    loadPose();
}

SobitMiniController::SobitMiniController( ) : SobitTurtlebotController( ) {
    pub_body_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    // pub_left_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/left_arm_trajectory_controller/command", 1);
    pub_l_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_trajectory_controller/command", 1);  
    // pub_right_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/right_arm_trajectory_controller/command", 1);
    pub_r_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_trajectory_controller/command", 1);
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
        std::vector<double> joint_val(13, 0.0);
        pose.pose_name = static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::L_ARM_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["l_arm_shoulder_roll_joint"]);
        joint_val[Joint::L_ARM_SHOULDER_PAN_JOINT] = static_cast<double>(pose_val[i]["l_arm_shoulder_pan_joint"]);
        joint_val[Joint::L_ARM_ELBOW_TILT_JOINT]    = static_cast<double>(pose_val[i]["l_arm_elbow_tilt_joint"]);
        joint_val[Joint::L_ARM_WRIST_TILT_JOINT]    = static_cast<double>(pose_val[i]["l_arm_wrist_tilt_joint"]);
        joint_val[Joint::L_HAND_JOINT]              = static_cast<double>(pose_val[i]["l_hand_joint"]);
        joint_val[Joint::R_ARM_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["r_arm_shoulder_roll_joint"]);
        joint_val[Joint::R_ARM_SHOULDER_PAN_JOINT] = static_cast<double>(pose_val[i]["r_arm_shoulder_pan_joint"]);
        joint_val[Joint::R_ARM_ELBOW_ROLL_JOINT]    = static_cast<double>(pose_val[i]["r_arm_elbow_tilt_joint"]);
        joint_val[Joint::R_ARM_WRIST_TILT_JOINT]    = static_cast<double>(pose_val[i]["r_arm_wrist_tilt_joint"]);
        joint_val[Joint::R_HAND_JOINT]              = static_cast<double>(pose_val[i]["r_hand_joint"]);
        joint_val[Joint::BODY_ROLL_JOINT]           = static_cast<double>(pose_val[i]["body_roll_joint"]);
        joint_val[Joint::HEAD_PAN_JOINT]            = static_cast<double>(pose_val[i]["head_pan_joint"]);
        joint_val[Joint::HEAD_TILT_JOINT]           = static_cast<double>(pose_val[i]["head_tilt_joint"]);
        // joint_val[Joint::HEAD_PAN_JOINT] = static_cast<double>(pose_val[i]["head_pan_joint"]);
        // joint_val[Joint::HEAD_TILT_JOINT] = static_cast<double>(pose_val[i]["head_tilt_joint"]);
        // joint_val[Joint::BODY_ROLL_JOINT] = static_cast<double>(pose_val[i]["body_roll_joint"]);
        // joint_val[Joint::RIGHT_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["right_shoulder_roll_joint"]);
        // joint_val[Joint::RIGHT_SHOULDER_FLEX_JOINT] = static_cast<double>(pose_val[i]["right_shoulder_flex_joint"]);
        // joint_val[Joint::RIGHT_ELBOW_ROLL_JOINT] = static_cast<double>(pose_val[i]["right_elbow_roll_joint"]);
        // joint_val[Joint::RIGHT_HAND_MOTOR_JOINT] = static_cast<double>(pose_val[i]["right_hand_motor_joint"]);
        // joint_val[Joint::LEFT_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i]["left_shoulder_roll_joint"]);
        // joint_val[Joint::LEFT_SHOULDER_FLEX_JOINT] = static_cast<double>(pose_val[i]["left_shoulder_flex_joint"]);
        // joint_val[Joint::LEFT_ELBOW_ROLL_JOINT] = static_cast<double>(pose_val[i]["left_elbow_roll_joint"]);
        // joint_val[Joint::LEFT_HAND_MOTOR_JOINT] = static_cast<double>(pose_val[i]["left_hand_motor_joint"]);                                                             
        pose.joint_val = joint_val;
        pose_list_.push_back( pose );
    }
    return;
}

bool SobitMiniController::moveAllJoint( const double l_arm_shoulder_roll_joint,
                                        const double l_arm_shoulder_pan_joint,
                                        const double l_arm_elbow_tilt_joint,
                                        const double l_arm_wrist_tilt_joint,
                                        const double l_hand_joint,
                                        const double r_arm_shoulder_roll_joint,
                                        const double r_arm_shoulder_pan_joint,
                                        const double r_arm_elbow_tilt_joint,
                                        const double r_arm_wrist_tilt_joint,
                                        const double r_hand_joint,
                                        const double body_roll_joint,
                                        const double head_pan_joint,
                                        const double head_tilt_joint, 
    // const double head_pan, 
    // const double head_tilt, 
    // const double body_roll, 
    // const double right_shoulder_roll, 
    // const double right_shoulder_flex, 
    // const double right_elbow_roll, 
    // const double right_hand_motor, 
    // const double left_shoulder_roll, 
    // const double left_shoulder_flex, 
    // const double left_elbow_roll, 
    // const double left_hand_motor, 
    const double sec, 
    bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory head_joint_trajectory;
        trajectory_msgs::JointTrajectory body_joint_trajectory;
        trajectory_msgs::JointTrajectory r_arm_joint_trajectory;
        trajectory_msgs::JointTrajectory l_arm_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_ROLL_JOINT], -l_arm_shoulder_roll_joint, sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_PAN_JOINT], l_arm_shoulder_pan_joint, sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_ELBOW_TILT_JOINT], -l_arm_elbow_tilt_joint, sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_WRIST_TILT_JOINT], -l_arm_wrist_tilt_joint, sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_HAND_JOINT], -l_hand_joint, sec, &l_arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_ROLL_JOINT], r_arm_shoulder_roll_joint, sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_PAN_JOINT], -r_arm_shoulder_pan_joint, sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_ELBOW_ROLL_JOINT], r_arm_elbow_tilt_joint, sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_WRIST_TILT_JOINT], r_arm_wrist_tilt_joint, sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_HAND_JOINT], r_hand_joint, sec, &r_arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::BODY_ROLL_JOINT], -body_roll_joint, sec, &body_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT], head_pan_joint, sec, &head_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], head_tilt_joint, sec, &head_joint_trajectory );

        // setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT], head_pan, sec, &head_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], head_tilt, sec, &head_joint_trajectory );
        // setJointTrajectory( joint_names_[Joint::BODY_ROLL_JOINT], body_roll, sec, &body_joint_trajectory );
        // setJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_ROLL_JOINT], right_shoulder_roll, sec, &right_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_FLEX_JOINT], right_shoulder_flex, sec, &right_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_ELBOW_ROLL_JOINT], right_elbow_roll, sec, &right_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_HAND_MOTOR_JOINT], right_hand_motor, sec, &right_arm_joint_trajectory );
        // setJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_ROLL_JOINT], left_shoulder_roll, sec, &left_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_FLEX_JOINT], left_shoulder_flex, sec, &left_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_ELBOW_ROLL_JOINT], left_elbow_roll, sec, &left_arm_joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_HAND_MOTOR_JOINT], left_hand_motor, sec, &left_arm_joint_trajectory );
        checkPublishersConnection ( pub_head_control_ );
        checkPublishersConnection ( pub_body_control_ );
        // checkPublishersConnection ( pub_right_arm_control_ );
        checkPublishersConnection ( pub_r_arm_control_ );
        // checkPublishersConnection ( pub_left_arm_control_ );
        checkPublishersConnection ( pub_l_arm_control_ );
        pub_head_control_.publish( head_joint_trajectory );
        pub_body_control_.publish( body_joint_trajectory );
        // pub_right_arm_control_.publish( r_arm_joint_trajectory );
        pub_r_arm_control_.publish( r_arm_joint_trajectory );
        // pub_left_arm_control_.publish( l_arm_joint_trajectory );
        pub_l_arm_control_.publish( l_arm_joint_trajectory );
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
        if(joint_num == Joint::BODY_ROLL_JOINT || joint_num == Joint::L_ARM_SHOULDER_ROLL_JOINT || joint_num == Joint::L_ARM_ELBOW_TILT_JOINT || joint_num == Joint::L_ARM_WRIST_TILT_JOINT || joint_num == Joint::L_HAND_JOINT || joint_num == Joint:: R_ARM_SHOULDER_PAN_JOINT){
            setJointTrajectory( joint_names_[joint_num], -rad, sec, &joint_trajectory);
        }else{
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory);
        } 
        // setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        // if( joint_num < Joint::BODY_ROLL_JOINT ) {
        //     checkPublishersConnection ( pub_head_control_ );
        //     pub_head_control_.publish( joint_trajectory );
        // } 
        if( joint_num == Joint::HEAD_PAN_JOINT  || joint_num == Joint::HEAD_TILT_JOINT) {
            checkPublishersConnection ( pub_head_control_ );
            pub_head_control_.publish( joint_trajectory );
        }else if( joint_num == Joint::BODY_ROLL_JOINT){
            checkPublishersConnection( pub_body_control_);
            pub_body_control_.publish( joint_trajectory );
        }else if( joint_num < Joint::R_ARM_SHOULDER_ROLL_JOINT){
            checkPublishersConnection( pub_l_arm_control_);
            pub_l_arm_control_.publish( joint_trajectory );
        }else {
            checkPublishersConnection ( pub_r_arm_control_ );
            pub_r_arm_control_.publish( joint_trajectory );
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

bool SobitMiniController::moveToPose( const std::string &pose_name ) {
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
                            joint_val[L_ARM_SHOULDER_ROLL_JOINT],
                            joint_val[L_ARM_SHOULDER_PAN_JOINT],
                            joint_val[L_ARM_ELBOW_TILT_JOINT],
                            joint_val[L_ARM_WRIST_TILT_JOINT],
                            joint_val[L_HAND_JOINT],
                            joint_val[R_ARM_SHOULDER_ROLL_JOINT],
                            joint_val[R_ARM_SHOULDER_PAN_JOINT],
                            joint_val[R_ARM_ELBOW_ROLL_JOINT],
                            joint_val[R_ARM_WRIST_TILT_JOINT],
                            joint_val[R_HAND_JOINT],
                            joint_val[BODY_ROLL_JOINT],
                            joint_val[HEAD_PAN_JOINT],
                            joint_val[HEAD_TILT_JOINT],
                            // joint_val[HEAD_PAN_JOINT],
                            // joint_val[HEAD_TILT_JOINT],
                            // joint_val[BODY_ROLL_JOINT],
                            // joint_val[RIGHT_SHOULDER_ROLL_JOINT],
                            // joint_val[RIGHT_SHOULDER_FLEX_JOINT],
                            // joint_val[RIGHT_ELBOW_ROLL_JOINT],
                            // joint_val[RIGHT_HAND_MOTOR_JOINT],
                            // joint_val[LEFT_SHOULDER_ROLL_JOINT],
                            // joint_val[LEFT_SHOULDER_FLEX_JOINT],
                            // joint_val[LEFT_ELBOW_ROLL_JOINT],
                            // joint_val[LEFT_HAND_MOTOR_JOINT], 
                            5.0 );
    } else {
        ROS_ERROR("'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitMiniController::moveRightArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_ROLL_JOINT], shoulder_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_PAN_JOINT], -shoulder_pan, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_ELBOW_ROLL_JOINT], elbow_tilt, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_WRIST_TILT_JOINT], wrist_tilt, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_HAND_JOINT], hand_motor, 5.0, &joint_trajectory );
        // setJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_ROLL_JOINT], shoulder_roll, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_SHOULDER_FLEX_JOINT], shoulder_flex, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_ELBOW_ROLL_JOINT], elbow_roll, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::RIGHT_HAND_MOTOR_JOINT], hand_motor, 5.0, &joint_trajectory );
        // checkPublishersConnection ( pub_right_arm_control_ );
        checkPublishersConnection ( pub_r_arm_control_ );
        // pub_right_arm_control_.publish( joint_trajectory );
        pub_r_arm_control_.publish( joint_trajectory );
        ros::Duration( 5.0 ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniController::moveLeftArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_ROLL_JOINT], -shoulder_roll, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_PAN_JOINT], shoulder_pan, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_ELBOW_TILT_JOINT], -elbow_tilt, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_WRIST_TILT_JOINT], -wrist_tilt, 5.0, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_HAND_JOINT], -hand_motor, 5.0, &joint_trajectory );
        // setJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_ROLL_JOINT], shoulder_roll, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_SHOULDER_FLEX_JOINT], shoulder_flex, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_ELBOW_ROLL_JOINT], elbow_roll, 5.0, &joint_trajectory );
        // addJointTrajectory( joint_names_[Joint::LEFT_HAND_MOTOR_JOINT], hand_motor, 5.0, &joint_trajectory );
        // checkPublishersConnection ( pub_left_arm_control_ );
        checkPublishersConnection ( pub_l_arm_control_ );
        // pub_left_arm_control_.publish( joint_trajectory );
        pub_l_arm_control_.publish( joint_trajectory );
        ros::Duration( 5.0 ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
