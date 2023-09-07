#ifndef SOBIT_MINI_CONTROLLER
#define SOBIT_MINI_CONTROLLER

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <sobit_mini_library/sobit_turtlebot_controller.hpp>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sobit_common_msg/current_state.h>
#include <sobit_common_msg/current_state_array.h>

namespace sobit_mini {
    enum Joint { L_ARM_SHOULDER_ROLL_JOINT = 0,
                 L_ARM_SHOULDER_PAN_JOINT,
                 L_ARM_ELBOW_TILT_JOINT,
                 L_ARM_WRIST_TILT_JOINT,
                 L_HAND_JOINT,
                 R_ARM_SHOULDER_ROLL_JOINT,
                 R_ARM_SHOULDER_PAN_JOINT,
                 R_ARM_ELBOW_ROLL_JOINT,
                 R_ARM_WRIST_TILT_JOINT,
                 R_HAND_JOINT,
                 BODY_ROLL_JOINT,
                 HEAD_PAN_JOINT,
                 HEAD_TILT_JOINT,
                 JOINT_NUM  
                };

    typedef struct {         
        std::string pose_name;
        std::vector<double> joint_val;
    } Pose;

    class SobitMiniController : public SobitTurtlebotController {
        private:
            ros::Publisher pub_body_control_;
            ros::Publisher pub_head_control_; 
            ros::Publisher pub_l_arm_control_; 
            ros::Publisher pub_r_arm_control_; 
            tf::TransformListener listener_;
            
            const std::vector<std::string> joint_names_ = { "l_arm_shoulder_roll_joint",
                                                            "l_arm_shoulder_pan_joint",  
                                                            "l_arm_elbow_tilt_joint",                               
                                                            "l_arm_wrist_tilt_joint",
                                                            "l_hand_joint",
                                                            "r_arm_shoulder_roll_joint",
                                                            "r_arm_shoulder_pan_joint",
                                                            "r_arm_elbow_tilt_joint",
                                                            "r_arm_wrist_tilt_joint",
                                                            "r_hand_joint",
                                                            "body_roll_joint",
                                                            "head_pan_joint",
                                                            "head_tilt_joint"
                                                            };
            std::vector<Pose> pose_list_;

            static const double arm_upper_link_cm;
            static const double arm_forearm_link_cm;
            static const double arm_hand_link_cm;
            static const double grasp_min_z_cm;//sobitminiの場合通常のx-yグラフを+90°回転させた状態を基準としているため
            static const double grasp_max_z_cm;

            void loadPose();
            bool moveAllJoint( const double l_arm_shoulder_roll_joint,
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
                               const double sec,
                               bool is_sleep = true 
            );
        public:
            SobitMiniController( const std::string &name );
            SobitMiniController( );
            bool moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep = true );
            bool moveHeadPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep = true );  
            bool moveRightArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor, const double sec, bool is_sleep = true );
            bool moveLeftArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor, const double sec, bool is_sleep = true );
            bool moveToPose( const std::string &pose_name );
            bool moveGripperToTargetCoord(const int arm_mode, const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToTargetTF(const int arm_mode, const std::string &goal_position_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z);
    };
}
#endif