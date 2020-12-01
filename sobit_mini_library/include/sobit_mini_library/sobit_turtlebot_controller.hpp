#ifndef SOBIT_TURTLEBOT_CONTROLLER
#define SOBIT_TURTLEBOT_CONTROLLER

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <pybind11/pybind11.h>


class ROSCommonNode
{
  public:
    ROSCommonNode( const std::string &name ) {
        char* cstr = new char[name.size() + 1];
        std::strcpy(cstr, name.c_str()); 
        char **argv = &cstr;
        int argc = 0;
        delete[] cstr;
        ros::init( argc, argv, "sobit_turtlebot_controller_node");
    }
    ROSCommonNode( ) { }
};

namespace sobit {
    class SobitTurtlebotController  : private ROSCommonNode {
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
            SobitTurtlebotController( const std::string &name );
            SobitTurtlebotController( );
            bool controlWheelLinear( const double distance );
            bool controlWheelRotateRad( const double angle_rad );
            bool controlWheelRotateDeg( const double angle_deg );
    };
}

inline void sobit::SobitTurtlebotController::setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
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

inline void sobit::SobitTurtlebotController::addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
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

inline void sobit::SobitTurtlebotController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate(10);
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep(); }
        catch ( const std::exception& ex ) { break; }
    }
    return; 
}

inline void sobit::SobitTurtlebotController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_odom_ = *odom_msg; }

inline double sobit::SobitTurtlebotController::geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat ) {
    tf::Quaternion quat;
    double roll, pitch, yaw;
    quaternionMsgToTF(geometry_quat, quat);
    quat.normalize();
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;    
}

inline double sobit::SobitTurtlebotController::rad2Deg ( const double rad ) { return rad * 180.0 / M_PI;  }

inline double sobit::SobitTurtlebotController::deg2Rad ( const double deg ) { return deg * M_PI / 180.0; }

#endif