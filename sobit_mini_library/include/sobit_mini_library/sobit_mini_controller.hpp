#ifndef SOBIT_MINI_CONTROLLER
#define SOBIT_MINI_CONTROLLER

#include <sobit_mini_library/sobit_turtlebot_controller.hpp>

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
                    // HEAD_PAN_JOINT = 0,
                    // HEAD_TILT_JOINT,
                    // BODY_ROLL_JOINT,
                    // RIGHT_SHOULDER_ROLL_JOINT,
                    // RIGHT_SHOULDER_FLEX_JOINT,
                    // RIGHT_ELBOW_ROLL_JOINT,
                    // RIGHT_HAND_MOTOR_JOINT,
                    // LEFT_SHOULDER_ROLL_JOINT,
                    // LEFT_SHOULDER_FLEX_JOINT,
                    // LEFT_ELBOW_ROLL_JOINT,
                    // LEFT_HAND_MOTOR_JOINT, 
                    // JOINT_NUM 
                };

    typedef struct {         
        std::string pose_name;
        std::vector<double> joint_val;
    } Pose;

    class SobitMiniController : public SobitTurtlebotController {
        private:
            ros::Publisher pub_body_control_;
            ros::Publisher pub_head_control_; 
            // ros::Publisher pub_left_arm_control_; 
            ros::Publisher pub_l_arm_control_; 
            // ros::Publisher pub_right_arm_control_; 
            ros::Publisher pub_r_arm_control_; 
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
                                                            // "head_pan_joint",
                                                            // "head_tilt_joint",  
                                                            // "body_roll_joint",                               
                                                            // "right_shoulder_roll_joint",
                                                            // "right_shoulder_flex_joint",
                                                            // "right_elbow_roll_joint",
                                                            // "right_hand_motor_joint",
                                                            // "left_shoulder_roll_joint",
                                                            // "left_shoulder_flex_joint",
                                                            // "left_elbow_roll_joint",
                                                            // "left_hand_motor_joint"
                                                            };
            std::vector<Pose> pose_list_;
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
                // const double sec, 
                // bool is_sleep = true 
            );
        public:
            SobitMiniController( const std::string &name );
            SobitMiniController( );
            bool moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep = true );
            bool moveHeadPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep = true );  
            bool moveRightArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor );
            bool moveLeftArm ( const double shoulder_roll, const double shoulder_pan, const double elbow_tilt, const double wrist_tilt, const double hand_motor );
            // bool movePose( const std::string &pose_name );
            bool moveToPose( const std::string &pose_name );
    };
}
#endif