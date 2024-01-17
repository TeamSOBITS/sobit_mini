#ifndef SOBIT_MINI_JOINT_CONTROLLER_H
#define SOBIT_MINI_JOINT_CONTROLLER_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include "sobit_mini_library/sobit_mini_library.h"

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

    class SobitMiniJointController : private ROSCommonNode {
        private:

            ros::NodeHandle   nh_;
            ros::NodeHandle   pnh_;

            ros::Publisher pub_body_control_;
            ros::Publisher pub_head_control_; 
            ros::Publisher pub_l_arm_control_; 
            ros::Publisher pub_r_arm_control_;

            tf2_ros::Buffer            tfBuffer_;
            tf2_ros::TransformListener tfListener_;
            // tf::TransformListener listener_;
            
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

            void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void checkPublishersConnection( const ros::Publisher& pub );
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
            SobitMiniJointController( const std::string &name );
            SobitMiniJointController( );
            bool moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep = true );
            bool moveHeadPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep = true );  
            bool moveRightArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor, const double sec, bool is_sleep = true );
            bool moveLeftArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor, const double sec, bool is_sleep = true );
            bool moveToPose( const std::string &pose_name );
            bool moveGripperToTargetCoord(const int arm_mode, const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToTargetTF(const int arm_mode, const std::string &goal_position_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z);
    };

    inline void sobit_mini::SobitMiniJointController::setJointTrajectory( const std::string& joint_name, 
                                                                        const double rad, 
                                                                        const double sec, 
                                                                    trajectory_msgs::JointTrajectory* jt ) {
        trajectory_msgs::JointTrajectory joint_trajectory;
        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point; 

        joint_trajectory.joint_names.push_back( joint_name ); 
        joint_trajectory_point.positions.push_back( rad );
        joint_trajectory_point.velocities.push_back( 0.0 );
        joint_trajectory_point.accelerations.push_back( 0.0 );
        joint_trajectory_point.effort.push_back( 0.0 );
        joint_trajectory_point.time_from_start = ros::Duration( sec );
        joint_trajectory.points.push_back( joint_trajectory_point );

        *jt = joint_trajectory;

        return;
    }

    inline void sobit_mini::SobitMiniJointController::addJointTrajectory( const std::string& joint_name, 
                                                                        const double rad, 
                                                                        const double sec, 
                                                                        trajectory_msgs::JointTrajectory* jt ) {
        trajectory_msgs::JointTrajectory joint_trajectory = *jt;

        joint_trajectory.joint_names.push_back( joint_name ); 
        joint_trajectory.points[0].positions.push_back( rad );
        joint_trajectory.points[0].velocities.push_back( 0.0 );
        joint_trajectory.points[0].accelerations.push_back( 0.0 );
        joint_trajectory.points[0].effort.push_back( 0.0 );
        joint_trajectory.points[0].time_from_start = ros::Duration( sec );

        *jt = joint_trajectory;

        return;
    }

    inline void sobit_mini::SobitMiniJointController::checkPublishersConnection ( const ros::Publisher& pub ) {

        ros::Rate loop_rate( 10 );
        while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
            try { loop_rate.sleep();
            } catch ( const std::exception& ex ) { break; }
        }
        return; 
    }
}
#endif /* SOBIT_MINI_JOINT_CONTROLLER_H */