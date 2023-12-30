#ifndef SOBIT_MINI_WHEEL_CONTROLLER_H_
#define SOBIT_MINI_WHEEL_CONTROLLER_H_

#include <cmath>
#include <cstring>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <trajectory_msgs/JointTrajectory.h>

#include "sobit_mini_library/sobit_mini_library.h"


namespace sobit_mini {
    class SobitMiniWheelController  : private ROSCommonNode {
        protected :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_cmd_vel_;
            ros::Subscriber sub_odom_;
            nav_msgs::Odometry curt_odom_;
            
            void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void checkPublishersConnection ( const ros::Publisher& pub );
            void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );
            double geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat );
            double rad2Deg ( const double rad );
            double deg2Rad ( const double deg );
        public :
            SobitMiniWheelController( const std::string &name );
            SobitMiniWheelController( );
            bool controlWheelLinear( const double distance );
            bool controlWheelRotateRad( const double angle_rad );
            bool controlWheelRotateDeg( const double angle_deg );
    };
}

inline void sobit_mini::SobitMiniWheelController::setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point; 
    joint_trajectory.joint_names.push_back(joint_name); 
    joint_trajectory_point.positions.push_back( rad );
    joint_trajectory_point.velocities.push_back( 0.0 );
    joint_trajectory_point.accelerations.push_back( 0.0 );
    joint_trajectory_point.effort.push_back( 0.0 );
    joint_trajectory_point.time_from_start = ros::Duration( sec );
    joint_trajectory.points.push_back(joint_trajectory_point);
    *jt = joint_trajectory;
    return;
}

inline void sobit_mini::SobitMiniWheelController::addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory = *jt;
    joint_trajectory.joint_names.push_back(joint_name); 
    joint_trajectory.points[0].positions.push_back( rad );
    joint_trajectory.points[0].velocities.push_back( 0.0 );
    joint_trajectory.points[0].accelerations.push_back( 0.0 );
    joint_trajectory.points[0].effort.push_back( 0.0 );
    joint_trajectory.points[0].time_from_start = ros::Duration( sec );
    *jt = joint_trajectory;
    return;
}

inline void sobit_mini::SobitMiniWheelController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate(10);
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep(); }
        catch ( const std::exception& ex ) { break; }
    }
    return; 
}

inline void sobit_mini::SobitMiniWheelController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_odom_ = *odom_msg; }

inline double sobit_mini::SobitMiniWheelController::geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat ) {
    tf2::Quaternion quat_tf;
    double roll, pitch, yaw;
    // quaternionMsgToTF(geometry_quat, quat_tf);
    tf2::fromMsg(geometry_quat, quat_tf);
    quat_tf.normalize();
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    return yaw;    
}

inline double sobit_mini::SobitMiniWheelController::rad2Deg ( const double rad ) { return rad * 180.0 / M_PI;  }

inline double sobit_mini::SobitMiniWheelController::deg2Rad ( const double deg ) { return deg * M_PI / 180.0; }

#endif /*SOBIT_MINI_WHEEL_CONTROLLER_H_*/