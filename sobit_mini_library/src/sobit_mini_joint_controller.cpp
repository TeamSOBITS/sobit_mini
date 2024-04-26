#include "sobit_mini_library/sobit_mini_joint_controller.hpp"
#include "sobit_mini_library/sobit_mini_wheel_controller.hpp"

using namespace sobit_mini;

SobitMiniJointController::SobitMiniJointController( const std::string &name ) : ROSCommonNode( name ), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
    pub_body_control_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    pub_l_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_trajectory_controller/command", 1);  
    pub_r_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_trajectory_controller/command", 1);
    loadPose();
}

SobitMiniJointController::SobitMiniJointController( ) : ROSCommonNode( ), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
    pub_body_control_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/body_trajectory_controller/command", 1);
    pub_head_control_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_trajectory_controller/command", 1);
    pub_l_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_trajectory_controller/command", 1);  
    pub_r_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_trajectory_controller/command", 1);
    loadPose();
}

void SobitMiniJointController::loadPose() {
    XmlRpc::XmlRpcValue pose_val;
    if ( !nh_.hasParam("/sobit_mini_pose") ) return; 
    nh_.getParam("/sobit_mini_pose", pose_val);

    int pose_num = pose_val.size();
    pose_list_.clear();

    for ( int i = 0; i < pose_num; i++ ) {
        Pose pose;
        std::vector<double> joint_val(Joint::JOINT_NUM, 0.0);

        pose.pose_name                              = static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::L_ARM_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i][joint_names_[Joint::L_ARM_SHOULDER_ROLL_JOINT]]);
        joint_val[Joint::L_ARM_SHOULDER_PAN_JOINT]  = static_cast<double>(pose_val[i][joint_names_[Joint::L_ARM_SHOULDER_PAN_JOINT]]);
        joint_val[Joint::L_ARM_ELBOW_TILT_JOINT]    = static_cast<double>(pose_val[i][joint_names_[Joint::L_ARM_ELBOW_TILT_JOINT]]);
        joint_val[Joint::L_ARM_WRIST_TILT_JOINT]    = static_cast<double>(pose_val[i][joint_names_[Joint::L_ARM_WRIST_TILT_JOINT]]);
        joint_val[Joint::L_HAND_JOINT]              = static_cast<double>(pose_val[i][joint_names_[Joint::L_HAND_JOINT]]);
        joint_val[Joint::R_ARM_SHOULDER_ROLL_JOINT] = static_cast<double>(pose_val[i][joint_names_[Joint::R_ARM_SHOULDER_ROLL_JOINT]]);
        joint_val[Joint::R_ARM_SHOULDER_PAN_JOINT]  = static_cast<double>(pose_val[i][joint_names_[Joint::R_ARM_SHOULDER_PAN_JOINT]]);
        joint_val[Joint::R_ARM_ELBOW_ROLL_JOINT]    = static_cast<double>(pose_val[i][joint_names_[Joint::R_ARM_ELBOW_ROLL_JOINT]]);
        joint_val[Joint::R_ARM_WRIST_TILT_JOINT]    = static_cast<double>(pose_val[i][joint_names_[Joint::R_ARM_WRIST_TILT_JOINT]]);
        joint_val[Joint::R_HAND_JOINT]              = static_cast<double>(pose_val[i][joint_names_[Joint::R_HAND_JOINT]]);
        joint_val[Joint::BODY_ROLL_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::BODY_ROLL_JOINT]]);
        joint_val[Joint::HEAD_PAN_JOINT]            = static_cast<double>(pose_val[i][joint_names_[Joint::HEAD_PAN_JOINT]]);
        joint_val[Joint::HEAD_TILT_JOINT]           = static_cast<double>(pose_val[i][joint_names_[Joint::HEAD_TILT_JOINT]]);
                                                             
        pose.joint_val = joint_val;
        pose_list_.push_back( pose );
    }
    return;
}

bool SobitMiniJointController::moveAllJoint( const double l_arm_shoulder_roll_joint,
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
                                        const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory head_joint_trajectory;
        trajectory_msgs::JointTrajectory body_joint_trajectory;
        trajectory_msgs::JointTrajectory r_arm_joint_trajectory;
        trajectory_msgs::JointTrajectory l_arm_joint_trajectory;

        setJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_ROLL_JOINT], -l_arm_shoulder_roll_joint, sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_PAN_JOINT] , l_arm_shoulder_pan_joint  , sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_ELBOW_TILT_JOINT]   , -l_arm_elbow_tilt_joint   , sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_WRIST_TILT_JOINT]   , -l_arm_wrist_tilt_joint   , sec, &l_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_HAND_JOINT]             , -l_hand_joint             , sec, &l_arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_ROLL_JOINT], r_arm_shoulder_roll_joint , sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_PAN_JOINT] , -r_arm_shoulder_pan_joint , sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_ELBOW_ROLL_JOINT]   , r_arm_elbow_tilt_joint    , sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_WRIST_TILT_JOINT]   , r_arm_wrist_tilt_joint    , sec, &r_arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_HAND_JOINT]             , r_hand_joint              , sec, &r_arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::BODY_ROLL_JOINT]          , -body_roll_joint * (116 / 22), sec, &body_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT]           , head_pan_joint            , sec, &head_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT]          , head_tilt_joint           , sec, &head_joint_trajectory );

        checkPublishersConnection ( pub_head_control_ );
        checkPublishersConnection ( pub_body_control_ );
        checkPublishersConnection ( pub_r_arm_control_ );
        checkPublishersConnection ( pub_l_arm_control_ );

        pub_head_control_.publish( head_joint_trajectory );
        pub_body_control_.publish( body_joint_trajectory );
        pub_r_arm_control_.publish( r_arm_joint_trajectory );
        pub_l_arm_control_.publish( l_arm_joint_trajectory );

        if ( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniJointController::moveJoint ( const Joint joint_num,
                                           const double rad,
                                           const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        if(joint_num == Joint::L_ARM_SHOULDER_ROLL_JOINT || joint_num == Joint::L_ARM_ELBOW_TILT_JOINT || joint_num == Joint::L_ARM_WRIST_TILT_JOINT || joint_num == Joint::L_HAND_JOINT || joint_num == Joint:: R_ARM_SHOULDER_PAN_JOINT){
            setJointTrajectory( joint_names_[joint_num], -rad, sec, &joint_trajectory);

        }else if(joint_num == Joint::BODY_ROLL_JOINT){
            double body_rad = rad * (116 / 22);
            setJointTrajectory( joint_names_[joint_num], -body_rad, sec, &joint_trajectory);

        }else{
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory);
        } 

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

bool SobitMiniJointController::moveHeadPanTilt ( const double pan_rad,
                                                 const double tilt_rad,
                                                 const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        setJointTrajectory( joint_names_[Joint::HEAD_PAN_JOINT] , pan_rad , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_TILT_JOINT], tilt_rad, sec, &joint_trajectory );
        
        checkPublishersConnection ( pub_head_control_ );
        pub_head_control_.publish( joint_trajectory );

        if ( is_sleep ) ros::Duration( sec ).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniJointController::moveToPose( const std::string &pose_name, const double sec) {
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
                            sec );
    } else {
        ROS_ERROR("'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitMiniJointController::moveRightArm ( const double shoulder_roll,
                                              const double shoulder_pan,
                                              const double elbow_tilt,
                                              const double wrist_tilt,
                                              const double hand_motor,
                                              const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        setJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_ROLL_JOINT], shoulder_roll, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_SHOULDER_PAN_JOINT] , -shoulder_pan, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_ELBOW_ROLL_JOINT]   , elbow_tilt   , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_ARM_WRIST_TILT_JOINT]   , wrist_tilt   , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::R_HAND_JOINT]             , hand_motor   , sec, &joint_trajectory );
        
        checkPublishersConnection ( pub_r_arm_control_ );
        pub_r_arm_control_.publish( joint_trajectory );

        if (is_sleep) ros::Duration( sec ).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniJointController::moveLeftArm ( const double shoulder_roll,
                                             const double shoulder_pan,
                                             const double elbow_tilt,
                                             const double wrist_tilt,
                                             const double hand_motor,
                                             const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        setJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_ROLL_JOINT], -shoulder_roll, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_SHOULDER_PAN_JOINT] , shoulder_pan  , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_ELBOW_TILT_JOINT]   , -elbow_tilt   , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_ARM_WRIST_TILT_JOINT]   , -wrist_tilt   , sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::L_HAND_JOINT]             , -hand_motor   , sec, &joint_trajectory );
        
        checkPublishersConnection ( pub_l_arm_control_ );
        pub_l_arm_control_.publish( joint_trajectory );

        if (is_sleep) ros::Duration( sec ).sleep();

        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitMiniJointController::moveGripperToTargetCoord( const int arm_mode,
                                                         const double hand_rad,
                                                         const double goal_position_x, const double goal_position_y, const double goal_position_z,
                                                         const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z,
                                                         const double sec, bool is_sleep ){
    sobit_mini::SobitMiniWheelController wheel_ctrl;
    geometry_msgs::Point shift;

    // Calculate goal_position_pos + difference(gap)
    const double base_to_goal_position_x = goal_position_x + shift.x + diff_goal_position_x;
    const double base_to_goal_position_y = goal_position_y + shift.y + diff_goal_position_y;
    const double base_to_goal_position_z = goal_position_z + shift.z + diff_goal_position_z;

    // Calculate angle between footbase_pos and the sifted goal_position_pos (XY平面)
    double tan_rad = std::atan2(base_to_goal_position_y,base_to_goal_position_x);

    // Change goal_position_pos units (m->cm)
    const double goal_position_pos_x_cm = base_to_goal_position_x * 100.0;
    const double goal_position_pos_y_cm = base_to_goal_position_y * 100.0;
    const double goal_position_pos_z_cm = base_to_goal_position_z * 100.0;

    //Stores values used in inverse kinematics
    double goal_object_distance_x_cm = std::sqrt(std::pow(goal_position_pos_x_cm,2) + std::pow(goal_position_pos_y_cm,2));
    double goal_object_distance_z_cm = -goal_position_pos_z_cm;

    //Determination of grasp direction for objects
    const double total_degrees = 90.0;

    // Check if the object is graspable
    if (goal_object_distance_z_cm <= grasp_max_z_cm) {
        std::cout << "The target is located too tall(" << -goal_object_distance_z_cm << ">22.0)" << std::endl;
        return false;
    } else if (goal_object_distance_z_cm >= grasp_min_z_cm) {
        std::cout << "The target is located too low(" << -goal_object_distance_z_cm << "<-22.0)" << std::endl;
        return false;
    }

    //Value is determined by the height of the object
    double linear_m = 0.0;
    if ((goal_object_distance_z_cm < -20.0) || (20.0 < goal_object_distance_z_cm)) {
        linear_m = goal_object_distance_x_cm - 19.0;
        goal_object_distance_x_cm = goal_object_distance_x_cm - linear_m;
    } 
    else if ((-20.0 <= goal_object_distance_z_cm && goal_object_distance_z_cm <= -15.0) || (15.0 <= goal_object_distance_z_cm && goal_object_distance_z_cm <= 20.0)) {
        linear_m = goal_object_distance_x_cm - 23.0;
        goal_object_distance_x_cm = goal_object_distance_x_cm - linear_m;
    } 
    else if ((-15.0 <= goal_object_distance_z_cm && goal_object_distance_z_cm <= -11.0) || (11.0 < goal_object_distance_z_cm && goal_object_distance_z_cm < 15.0)) {
        linear_m = goal_object_distance_x_cm - 30.0;
        goal_object_distance_x_cm = goal_object_distance_x_cm - linear_m;
    } 
    else if (-11.0 <= goal_object_distance_z_cm && goal_object_distance_z_cm <= 11.0) {
        linear_m = goal_object_distance_x_cm - 34.0;
        goal_object_distance_x_cm = goal_object_distance_x_cm - linear_m;
    }

    // - Move forward the robot
    linear_m = linear_m / 100.0;

    // Inverse Kinematics Calculations
    const double x_p = goal_object_distance_z_cm - (arm_hand_link_cm * std::cos(total_degrees * M_PI / 180.0));
    const double y_p = goal_object_distance_x_cm - (arm_hand_link_cm * std::sin(total_degrees * M_PI / 180.0));

    const double o_b = std::sqrt(std::pow(x_p,2) + std::pow(y_p,2));
    double alpha = std::acos(((std::pow(arm_upper_link_cm,2) + std::pow(o_b,2)) - std::pow(arm_forearm_link_cm,2)) / (2 * arm_upper_link_cm * o_b));
    double beta = std::acos(((std::pow(arm_upper_link_cm,2) + std::pow(arm_forearm_link_cm,2)) - std::pow(o_b,2)) / (2 * arm_upper_link_cm * arm_forearm_link_cm));
    double gamma = std::atan2(y_p,x_p);

    // std::cout << "alpha:" << alpha * 180.0 / M_PI <<std::endl;
    // std::cout << "beta:" << beta * 180.0 / M_PI <<std::endl;
    // std::cout << "gamma:" << gamma * 180.0 / M_PI <<std::endl;

    // std::cout << "alpha:" << alpha << std::endl;
    // std::cout << "beta:" << beta << std::endl;
    // std::cout << "gamma:" << gamma << std::endl;

    //Determine the angle of each joint
    const double arm_shoulder_roll_joint_rad = gamma - alpha;
    const double arm_elbow_tilt_joint_rad = (180.0 * M_PI / 180.0) - beta;
    const double arm_wrist_tilt_joint_rad = (total_degrees * M_PI / 180.0) - arm_shoulder_roll_joint_rad - arm_elbow_tilt_joint_rad;

    //tan_rad rotation
    wheel_ctrl.controlWheelRotateRad(tan_rad);
    ros::Duration(3.0).sleep();
    // std::cout << "arm_shoulder_roll_joint_rad:" << arm_shoulder_roll_joint_rad * 180.0 / M_PI <<std::endl;
    // std::cout << "arm_elbow_tilt_joint_rad:" << arm_elbow_tilt_joint_rad * 180.0 / M_PI <<std::endl;
    // std::cout << "arm_wrist_tilt_joint_rad:" << arm_wrist_tilt_joint_rad * 180.0 / M_PI <<std::endl;
    // std::cout << "linear_m:" << linear_m << std::endl;


    if (arm_mode == 0){//left_arm
        // moveLeftArm((80.0 * M_PI / 180.0), -(90.0 * M_PI / 180.0), (60.0 * M_PI / 180.0), (90.0 * M_PI / 180.0), 0.0, 2.0, true);
        ros::Duration(2.0).sleep();
        moveLeftArm( arm_shoulder_roll_joint_rad, -(90.0 * M_PI / 180.0), arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, hand_rad,
                     2.0, true);
        wheel_ctrl.controlWheelLinear(linear_m);
        ros::Duration(3.0).sleep();
        bool is_reached = moveLeftArm( arm_shoulder_roll_joint_rad, -(90.0 * M_PI / 180.0), arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, 0.0,
                                       2.0, true);
        ros::Duration(2.0).sleep();
        return is_reached;
    } else if (arm_mode == 1){//right_arm
        // moveRightArm((80.0 * M_PI / 180.0), -(90.0 * M_PI / 180.0), (60.0 * M_PI / 180.0), (90.0 * M_PI / 180.0), 0.0, 2.0, true);
        ros::Duration(2.0).sleep();
        moveRightArm( arm_shoulder_roll_joint_rad, -(90.0 * M_PI / 180.0), arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, hand_rad,
                      2.0, true);
        wheel_ctrl.controlWheelLinear(linear_m);
        ros::Duration(3.0).sleep();
        bool is_reached = moveRightArm( arm_shoulder_roll_joint_rad, -(90.0 * M_PI / 180.0), arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, 0.0,
                                        1.0, true);
        ros::Duration(2.0).sleep();
        return is_reached;
    }

    // return is_reached;
}

bool SobitMiniJointController::moveGripperToTargetTF( const int arm_mode,
                                                      const std::string &goal_position_name,
                                                      const double hand_rad,
                                                      const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z,
                                                      const double sec, bool is_sleep ){
    sobit_mini::SobitMiniWheelController wheel_ctrl;
    // tf2::StampedTransform transform_base_to_target;
    geometry_msgs::TransformStamped transform_base_to_target;
    // tf::StampedTransform transform_base_to_target;
    geometry_msgs::Point shift;
    double goal_position_x = 0.0;
    double goal_position_y = 0.0;
    double goal_position_z = 0.0;

    bool tf_flag = false;

    if (arm_mode == 0){//left_arm
        try{
            tfBuffer_.canTransform("l_arm_shoulder_roll_link", goal_position_name, ros::Time(0), ros::Duration(2.0));
            transform_base_to_target = tfBuffer_.lookupTransform("l_arm_shoulder_roll_link", goal_position_name, ros::Time(0));//transform_base_to_targetで物体のxyz成分を格納している
            tf_flag = true;

            // goal_position_x = transform_base_to_target.getOrigin().x();
            // goal_position_y = transform_base_to_target.getOrigin().y() - 0.05375;
            // goal_position_z = transform_base_to_target.getOrigin().z();

            goal_position_x = transform_base_to_target.transform.translation.x;
            goal_position_y = transform_base_to_target.transform.translation.y - 0.05375;
            goal_position_z = transform_base_to_target.transform.translation.z;

        }  catch (tf2::TransformException ex){
            ROS_ERROR("ERROR: %s", ex.what() );
            return false;
        }
    } else if (arm_mode == 1){//right_arm
        try{
            tfBuffer_.canTransform("r_arm_shoulder_roll_link", goal_position_name, ros::Time(0), ros::Duration(2.0));
            transform_base_to_target = tfBuffer_.lookupTransform("r_arm_shoulder_roll_link", goal_position_name, ros::Time(0));//transform_base_to_targetで物体のxyz成分を格納している
            tf_flag = true;

            // goal_position_x = transform_base_to_target.getOrigin().x();
            // goal_position_y = transform_base_to_target.getOrigin().y() + 0.05375;
            // goal_position_z = transform_base_to_target.getOrigin().z();

            goal_position_x = transform_base_to_target.transform.translation.x;
            goal_position_y = transform_base_to_target.transform.translation.y + 0.05375;
            goal_position_z = transform_base_to_target.transform.translation.z;
        }  catch (tf2::TransformException ex){
            ROS_ERROR("ERROR: %s", ex.what() );
            return false;
        }
    } else {
        std::cout << "There is no mode type! mode type is 0 or 1." << std::endl;
        return false;
    }

    bool is_reached = moveGripperToTargetCoord(arm_mode,
                                               hand_rad,
                                               goal_position_x, goal_position_y, goal_position_z,
                                               diff_goal_position_x, diff_goal_position_y, diff_goal_position_z,
                                               sec, is_sleep);

    return is_reached;
}
