#include "sobit_mini_library/sobit_mini_wheel_action_server.hpp"

namespace sobit_mini {

WheelActionServer::WheelActionServer(const rclcpp::NodeOptions & options)
: Node("wheel_action_server", options)
{
  rclcpp::QoS qos_profile(1);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  this->action_server_move_wheel_linear_ = rclcpp_action::create_server<MoveWheelLinear>(
      this, "move_wheel_linear",
      std::bind(&WheelActionServer::handle_move_wheel_linear_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_linear_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_linear_accepted, this, std::placeholders::_1));

  this->action_server_move_wheel_rotate_ = rclcpp_action::create_server<MoveWheelRotate>(
      this, "move_wheel_rotate",
      std::bind(&WheelActionServer::handle_move_wheel_rotate_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_accepted, this, std::placeholders::_1));

  this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("commands/velocity", qos_profile);
  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_profile, std::bind(&WheelActionServer::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "WheelActionServer has been initialized.");
}

WheelActionServer::~WheelActionServer()
{
  this->action_server_move_wheel_linear_.reset();
  this->action_server_move_wheel_rotate_.reset();
  this->pub_cmd_vel_.reset();
  this->sub_odom_.reset();
}

rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_linear_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveWheelLinear::Goal> goal)
{
  (void)uuid; (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_rotate_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveWheelRotate::Goal> goal)
{
  (void)uuid; (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_linear_cancel(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_rotate_cancel(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WheelActionServer::handle_move_wheel_linear_accepted(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_linear, this, std::placeholders::_1), goal_handle}.detach();
}

void WheelActionServer::handle_move_wheel_rotate_accepted(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_rotate, this, std::placeholders::_1), goal_handle}.detach();
}

void WheelActionServer::exe_move_wheel_linear(const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelLinear::Result>();

  double goal_dist = std::abs(goal->target_point.x);
  double curt_dist = 0.0;
  double integral = 0.0;
  double prev_error = goal_dist;
  double current_vel = 0.0;

  double kp = 0.5;
  double ki = 0.02;
  double kd = 0.1;
  double tolerance = 0.05;
  double dt = 0.1;

  double max_vel = 0.3;
  double min_vel = 0.05;
  double max_accel = 0.02;

  auto start_time = this->now();
  this->init_odom_ = this->curt_odom_;
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      this->pub_cmd_vel_->publish(zero_vel_);
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    curt_dist = std::sqrt(
        std::pow(this->curt_odom_.pose.pose.position.x - this->init_odom_.pose.pose.position.x, 2) +
        std::pow(this->curt_odom_.pose.pose.position.y - this->init_odom_.pose.pose.position.y, 2));

    double error = goal_dist - curt_dist;
    if (error <= tolerance) break;

    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double target_vel = (kp * error) + (ki * integral) + (kd * derivative);

    double vel_diff = target_vel - current_vel;
    if (std::abs(vel_diff) > max_accel) {
        target_vel = current_vel + std::copysign(max_accel, vel_diff);
    }

    target_vel = std::clamp(target_vel, min_vel, max_vel);
    current_vel = target_vel;

    geometry_msgs::msg::Twist out_vel;
    out_vel.linear.x = (goal->target_point.x > 0) ? current_vel : -current_vel;
    this->pub_cmd_vel_->publish(out_vel);

    auto feedback = std::make_shared<MoveWheelLinear::Feedback>();
    feedback->current_point.x = curt_dist * ((goal->target_point.x > 0) ? 1.0 : -1.0);
    goal_handle->publish_feedback(feedback);

    prev_error = error;
    loop_rate.sleep();
  }

  this->pub_cmd_vel_->publish(zero_vel_);
  result->success = true;
  result->total_elapsed_time = this->now() - start_time;
  goal_handle->succeed(result);
}

void WheelActionServer::exe_move_wheel_rotate(const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelRotate::Result>();

  this->init_odom_ = this->curt_odom_;
  double prev_real_angle = this->get_euler_from_quat(this->init_odom_.pose.pose.orientation).z;
  double moved_angle = 0.0;
  double goal_angle = std::abs(goal->target_yaw);
  double current_vel = 0.0;

  double kp = 0.6;
  double ki = 0.03;
  double kd = 0.1;
  double integral = 0.0;
  double prev_error = goal_angle;
  double tolerance = 0.05;
  double dt = 0.1;

  double max_vel = 0.6;
  double min_vel = 0.1;
  double max_accel = 0.05;

  auto start_time = this->now();
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      this->pub_cmd_vel_->publish(zero_vel_);
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    double curt_real_angle = this->get_euler_from_quat(this->curt_odom_.pose.pose.orientation).z;
    double delta_angle = curt_real_angle - prev_real_angle;
    if (delta_angle > M_PI) delta_angle -= 2.0 * M_PI;
    else if (delta_angle < -M_PI) delta_angle += 2.0 * M_PI;

    moved_angle += std::abs(delta_angle);
    prev_real_angle = curt_real_angle;

    double error = goal_angle - moved_angle;
    if (error <= tolerance) break;

    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double target_vel = (kp * error) + (ki * integral) + (kd * derivative);

    double vel_diff = target_vel - current_vel;
    if (std::abs(vel_diff) > max_accel) {
        target_vel = current_vel + std::copysign(max_accel, vel_diff);
    }

    target_vel = std::clamp(target_vel, min_vel, max_vel);
    current_vel = target_vel;

    geometry_msgs::msg::Twist out_vel;
    out_vel.angular.z = (goal->target_yaw > 0) ? current_vel : -current_vel;
    this->pub_cmd_vel_->publish(out_vel);

    auto feedback = std::make_shared<MoveWheelRotate::Feedback>();
    feedback->current_yaw = moved_angle * ((goal->target_yaw > 0) ? 1.0 : -1.0);
    goal_handle->publish_feedback(feedback);

    prev_error = error;
    loop_rate.sleep();
  }

  this->pub_cmd_vel_->publish(zero_vel_);
  result->success = true;
  result->total_elapsed_time = this->now() - start_time;
  goal_handle->succeed(result);
}

void WheelActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->curt_odom_ = *msg;
}

} // namespace sobit_mini