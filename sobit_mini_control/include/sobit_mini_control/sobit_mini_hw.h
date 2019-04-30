#ifndef SOBIT_MINI_HW_H_
#define SOBIT_MINI_HW_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <fstream>   // debug
#include <iostream>  // debug
#include "sobit_common_dynamixel.h"

class SobitMiniDynamixel : public SobitCommonDynamixel {
 public:
  SobitMiniDynamixel();
  ~SobitMiniDynamixel();
  void                    initializeDynamixel();
  void                    writeDynamixelMotors(const trajectory_msgs::JointTrajectory &pose);
  sensor_msgs::JointState readDynamixelMotors();

 private:
  int   getJointNumber(std::string joint_name);
  float toBit(int id, float rad);
  float toRad(std::string joint_name, int bit);
};

class SobitMiniControl : public SobitMiniDynamixel {
 public:
  ros::NodeHandle nh_;

 protected:
  hardware_interface::JointStateInterface                  jnt_state_interface_;
  hardware_interface::PositionJointInterface               jnt_pos_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limit_interface_;
  double                                                   cmd_[12];
  double                                                   pos_[12];
  double                                                   vel_[12];
  double                                                   eff_[12];

 public:
  SobitMiniControl();
  ros::Time     getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }
  void          writeInitialJoint();
  void          read(ros::Time, ros::Duration);
  void          write(ros::Time, ros::Duration);
};

#endif /* SOBIT_MINI_HW_H_ */