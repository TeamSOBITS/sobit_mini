#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sobit_mini_control/sobit_mini_hw.h>
#include <trajectory_msgs/JointTrajectory.h>

SobitMiniControl::SobitMiniControl() {
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_body_lift("body_lift_joint", &pos_[0], &vel_[0], &eff_[0]);
  hardware_interface::JointStateHandle state_handle_body_roll("body_roll_joint", &pos_[1], &vel_[1], &eff_[1]);
  hardware_interface::JointStateHandle state_handle_head_tilt("head_tilt_joint", &pos_[2], &vel_[2], &eff_[2]);
  hardware_interface::JointStateHandle state_handle_head_pan("head_pan_joint", &pos_[3], &vel_[3], &eff_[3]);
  hardware_interface::JointStateHandle state_handle_right_shoulder_roll(
      "right_shoulder_roll_joint", &pos_[4], &vel_[4], &eff_[4]);
  hardware_interface::JointStateHandle state_handle_right_shoulder_flex(
      "right_shoulder_flex_joint", &pos_[5], &vel_[5], &eff_[5]);
  hardware_interface::JointStateHandle state_handle_right_elbow_roll(
      "right_elbow_roll_joint", &pos_[6], &vel_[6], &eff_[6]);
  hardware_interface::JointStateHandle state_handle_right_hand_motor(
      "right_hand_motor_joint", &pos_[7], &vel_[7], &eff_[7]);
  hardware_interface::JointStateHandle state_handle_left_shoulder_roll(
      "left_shoulder_roll_joint", &pos_[8], &vel_[8], &eff_[8]);
  hardware_interface::JointStateHandle state_handle_left_shoulder_flex(
      "left_shoulder_flex_joint", &pos_[9], &vel_[9], &eff_[9]);
  hardware_interface::JointStateHandle state_handle_left_elbow_roll(
      "left_elbow_roll_joint", &pos_[10], &vel_[10], &eff_[10]);
  hardware_interface::JointStateHandle state_handle_left_hand_motor(
      "left_hand_motor_joint", &pos_[11], &vel_[11], &eff_[11]);
  jnt_state_interface_.registerHandle(state_handle_body_lift);
  jnt_state_interface_.registerHandle(state_handle_body_roll);
  jnt_state_interface_.registerHandle(state_handle_head_tilt);
  jnt_state_interface_.registerHandle(state_handle_head_pan);
  jnt_state_interface_.registerHandle(state_handle_right_shoulder_roll);
  jnt_state_interface_.registerHandle(state_handle_right_shoulder_flex);
  jnt_state_interface_.registerHandle(state_handle_right_elbow_roll);
  jnt_state_interface_.registerHandle(state_handle_right_hand_motor);
  jnt_state_interface_.registerHandle(state_handle_left_shoulder_roll);
  jnt_state_interface_.registerHandle(state_handle_left_shoulder_flex);
  jnt_state_interface_.registerHandle(state_handle_left_elbow_roll);
  jnt_state_interface_.registerHandle(state_handle_left_hand_motor);
  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_body_lift(jnt_state_interface_.getHandle("body_lift_joint"), &cmd_[0]);
  hardware_interface::JointHandle pos_handle_body_roll(jnt_state_interface_.getHandle("body_roll_joint"), &cmd_[1]);
  hardware_interface::JointHandle pos_handle_head_tilt(jnt_state_interface_.getHandle("head_tilt_joint"), &cmd_[2]);
  hardware_interface::JointHandle pos_handle_head_pan(jnt_state_interface_.getHandle("head_pan_joint"), &cmd_[3]);
  hardware_interface::JointHandle pos_handle_right_shoulder_roll(
      jnt_state_interface_.getHandle("right_shoulder_roll_joint"), &cmd_[4]);
  hardware_interface::JointHandle pos_handle_right_shoulder_flex(
      jnt_state_interface_.getHandle("right_shoulder_flex_joint"), &cmd_[5]);
  hardware_interface::JointHandle pos_handle_right_elbow_roll(jnt_state_interface_.getHandle("right_elbow_roll_joint"),
                                                              &cmd_[6]);
  hardware_interface::JointHandle pos_handle_right_hand_motor(jnt_state_interface_.getHandle("right_hand_motor_joint"),
                                                              &cmd_[7]);
  hardware_interface::JointHandle pos_handle_left_shoulder_roll(
      jnt_state_interface_.getHandle("left_shoulder_roll_joint"), &cmd_[8]);
  hardware_interface::JointHandle pos_handle_left_shoulder_flex(
      jnt_state_interface_.getHandle("left_shoulder_flex_joint"), &cmd_[9]);
  hardware_interface::JointHandle pos_handle_left_elbow_roll(jnt_state_interface_.getHandle("left_elbow_roll_joint"),
                                                             &cmd_[10]);
  hardware_interface::JointHandle pos_handle_left_hand_motor(jnt_state_interface_.getHandle("left_hand_motor_joint"),
                                                             &cmd_[11]);
  jnt_pos_interface_.registerHandle(pos_handle_body_lift);
  jnt_pos_interface_.registerHandle(pos_handle_body_roll);
  jnt_pos_interface_.registerHandle(pos_handle_head_tilt);
  jnt_pos_interface_.registerHandle(pos_handle_head_pan);
  jnt_pos_interface_.registerHandle(pos_handle_right_shoulder_roll);
  jnt_pos_interface_.registerHandle(pos_handle_right_shoulder_flex);
  jnt_pos_interface_.registerHandle(pos_handle_right_elbow_roll);
  jnt_pos_interface_.registerHandle(pos_handle_right_hand_motor);
  jnt_pos_interface_.registerHandle(pos_handle_left_shoulder_roll);
  jnt_pos_interface_.registerHandle(pos_handle_left_shoulder_flex);
  jnt_pos_interface_.registerHandle(pos_handle_left_elbow_roll);
  jnt_pos_interface_.registerHandle(pos_handle_left_hand_motor);
  registerInterface(&jnt_pos_interface_);

  // register the joint limits interface
  joint_limits_interface::JointLimits     joint_limits;
  joint_limits_interface::SoftJointLimits soft_joint_limits;
  getJointLimits("body_lift_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_body_lift(
      pos_handle_body_lift, joint_limits, soft_joint_limits);
  getJointLimits("body_roll_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_body_roll(
      pos_handle_body_roll, joint_limits, soft_joint_limits);
  getJointLimits("head_tilt_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_head_tilt(
      pos_handle_head_tilt, joint_limits, soft_joint_limits);
  getJointLimits("head_pan_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_head_pan(
      pos_handle_head_pan, joint_limits, soft_joint_limits);
  getJointLimits("right_shoulder_roll_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_right_shoulder_roll(
      pos_handle_right_shoulder_roll, joint_limits, soft_joint_limits);
  getJointLimits("right_shoulder_flex_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_right_shoulder_flex(
      pos_handle_right_shoulder_flex, joint_limits, soft_joint_limits);
  getJointLimits("right_elbow_roll_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_right_elbow_roll(
      pos_handle_right_elbow_roll, joint_limits, soft_joint_limits);
  getJointLimits("right_hand_motor", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_right_hand_motor(
      pos_handle_right_hand_motor, joint_limits, soft_joint_limits);
  getJointLimits("left_shoulder_roll_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_left_shoulder_roll(
      pos_handle_left_shoulder_roll, joint_limits, soft_joint_limits);
  getJointLimits("left_shoulder_flex", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_left_shoulder_flex(
      pos_handle_left_shoulder_flex, joint_limits, soft_joint_limits);
  getJointLimits("left_elbow_roll", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_left_elbow_roll(
      pos_handle_left_elbow_roll, joint_limits, soft_joint_limits);
  getJointLimits("left_hand_motor", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_left_hand_motor(
      pos_handle_left_hand_motor, joint_limits, soft_joint_limits);
  jnt_limit_interface_.registerHandle(joint_limit_body_lift);
  jnt_limit_interface_.registerHandle(joint_limit_body_roll);
  jnt_limit_interface_.registerHandle(joint_limit_head_tilt);
  jnt_limit_interface_.registerHandle(joint_limit_head_pan);
  jnt_limit_interface_.registerHandle(joint_limit_right_shoulder_roll);
  jnt_limit_interface_.registerHandle(joint_limit_right_shoulder_flex);
  jnt_limit_interface_.registerHandle(joint_limit_right_elbow_roll);
  jnt_limit_interface_.registerHandle(joint_limit_right_hand_motor);
  jnt_limit_interface_.registerHandle(joint_limit_left_shoulder_roll);
  jnt_limit_interface_.registerHandle(joint_limit_left_shoulder_flex);
  jnt_limit_interface_.registerHandle(joint_limit_left_elbow_roll);
  jnt_limit_interface_.registerHandle(joint_limit_left_hand_motor);
  registerInterface(&jnt_limit_interface_);
}

void SobitMiniControl::read(ros::Time time, ros::Duration period) {
  sensor_msgs::JointState pose = readDynamixelMotors();
  for (int i = 0; i < pose.name.size(); i++) {
    if (pose.name[i] == "body_lift_joint") {
      pos_[0] = pose.position[i];
      // std::cout << pos_[0] << std::endl;
    } else if (pose.name[i] == "body_roll_joint") {
      pos_[1] = pose.position[i];
    } else if (pose.name[i] == "head_tilt_joint") {
      pos_[2] = pose.position[i];
    } else if (pose.name[i] == "head_pan_joint") {
      pos_[3] = pose.position[i];
    } else if (pose.name[i] == "right_shoulder_roll_joint") {
      pos_[4] = pose.position[i];
    } else if (pose.name[i] == "right_shoulder_flex_joint") {
      pos_[5] = pose.position[i];
    } else if (pose.name[i] == "right_elbow_roll_joint") {
      pos_[6] = pose.position[i];
    } else if (pose.name[i] == "right_hand_motor_joint") {
      pos_[7] = pose.position[i];
    } else if (pose.name[i] == "left_shoulder_roll_joint") {
      pos_[8] = pose.position[i];
    } else if (pose.name[i] == "left_shoulder_flex_joint") {
      pos_[9] = pose.position[i];
    } else if (pose.name[i] == "left_elbow_roll_joint") {
      pos_[10] = pose.position[i];
    } else if (pose.name[i] == "left_hand_motor_joint") {
      pos_[11] = pose.position[i];
    }
  }
}

void SobitMiniControl::write(ros::Time time, ros::Duration period) {
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("body_lift_joint");
  traj.joint_names.push_back("body_roll_joint");
  traj.joint_names.push_back("head_tilt_joint");
  traj.joint_names.push_back("head_pan_joint");
  traj.joint_names.push_back("right_shoulder_roll_joint");
  traj.joint_names.push_back("right_shoulder_flex_joint");
  traj.joint_names.push_back("right_elbow_roll_joint");
  traj.joint_names.push_back("right_hand_motor_joint");
  traj.joint_names.push_back("left_shoulder_roll_joint");
  traj.joint_names.push_back("left_shoulder_flex_joint");
  traj.joint_names.push_back("left_elbow_roll_joint");
  traj.joint_names.push_back("left_hand_motor_joint");
  traj.points.resize(1);
  traj.points[0].positions.resize(12);
  traj.points[0].positions[0]    = cmd_[0];
  traj.points[0].positions[1]    = cmd_[1];
  traj.points[0].positions[2]    = cmd_[2];
  traj.points[0].positions[3]    = cmd_[3];
  traj.points[0].positions[4]    = cmd_[4];
  traj.points[0].positions[5]    = cmd_[5];
  traj.points[0].positions[6]    = cmd_[6];
  traj.points[0].positions[7]    = cmd_[7];
  traj.points[0].positions[8]    = cmd_[8];
  traj.points[0].positions[9]    = cmd_[9];
  traj.points[0].positions[10]   = cmd_[10];
  traj.points[0].positions[11]   = cmd_[11];
  traj.points[0].time_from_start = ros::Duration(0.0);
  writeDynamixelMotors(traj);
}