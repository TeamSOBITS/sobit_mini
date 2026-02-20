#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobit_mini
{

struct PoseParams
{
  std::string pose_name;
  double r_arm_shoulder_roll;
  double r_arm_shoulder_pan;
  double r_arm_elbow_tilt;
  double r_arm_wrist_tilt;
  double r_hand;
  double l_arm_shoulder_roll;
  double l_arm_shoulder_pan;
  double l_arm_elbow_tilt;
  double l_arm_wrist_tilt;
  double l_hand;
  double body_roll;
  double head_camera_pan;
  double head_camera_tilt;
};

enum JointIds
{
  R_ArmShoulderRollJoint,
  R_ArmShoulderPanJoint,
  R_ArmElbowTiltJoint,
  R_ArmWristTiltJoint,
  R_HandJoint,
  L_ArmShoulderRollJoint,
  L_ArmShoulderPanJoint,
  l_ArmElbowTiltJoint,
  L_ArmWristTiltJoint,
  L_HandJoint,
  BodyRollJoint,
  HeadCameraPanJoint,
  HeadCameraTiltJoint,
  JointNum
};

class JointActionServer : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  using GetHandToTargetCoord = sobits_interfaces::srv::GetHandToTargetCoord;
  using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;

  using GoalHandleMoveJoints =
    rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose =
    rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;

  explicit JointActionServer(const rclcpp::NodeOptions &options);
  ~JointActionServer();

  geometry_msgs::msg::Vector3 get_euler_from_quat(const geometry_msgs::msg::Quaternion &quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Vector3 &rpy);
  geometry_msgs::msg::TransformStamped
  forward_kinematics(const std::vector<double> &target_joint_rad, const bool is_right,
                     const double target_yaw);
  std::vector<double> inverse_kinematics(const geometry_msgs::msg::TransformStamped &goal_coord,
                                         const bool is_right, bool is_one_rink,
                                         const double target_yaw);
  trajectory_msgs::msg::JointTrajectory set_joints(
    const std::vector<std::string> &target_joint_names, const std::vector<double> &target_joint_rad,
    const builtin_interfaces::msg::Duration &time_allowance, const std::string &group_name);

private:
  const std::vector<std::string> JointNames = {"r_arm_shoulder_roll_joint",
                                               "r_arm_shoulder_pan_joint",
                                               "r_arm_elbow_tilt_joint",
                                               "r_arm_wrist_tilt_joint",
                                               "r_hand_joint",
                                               "l_arm_shoulder_roll_joint",
                                               "l_arm_shoulder_pan_joint",
                                               "l_arm_elbow_tilt_joint",
                                               "l_arm_wrist_tilt_joint",
                                               "l_hand_joint",
                                               "body_roll_joint",
                                               "head_camera_pan_joint",
                                               "head_camera_tilt_joint"};

  const std::vector<std::string> JointNamesArmLeft = {
    "l_arm_shoulder_roll_joint", "l_arm_shoulder_pan_joint", "l_arm_elbow_tilt_joint",
    "l_arm_wrist_tilt_joint"};

  const std::vector<std::string> JointNamesArmRight = {
    "r_arm_shoulder_roll_joint", "r_arm_shoulder_pan_joint", "r_arm_elbow_tilt_joint",
    "r_arm_wrist_tilt_joint"};

  const std::vector<std::string> JointNamesHandLeft = {"l_hand_joint"};

  const std::vector<std::string> JointNamesHandRight = {"r_hand_joint"};

  const std::vector<std::string> JointNamesHead = {"head_camera_pan_joint",
                                                   "head_camera_tilt_joint"};

  const std::vector<std::string> JointNamesBody = {"body_roll_joint"};

  static constexpr double BaseToShoulderDX = 0.0;
  static constexpr double BaseToShoulderDY = 0.195;
  static constexpr double BaseToShoulderDZ = 0.705;
  static constexpr double LengthShoulderElbow = 0.113;
  static constexpr double LengthElbowWrist = 0.105;
  static constexpr double LengthHand = 0.165;

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;

  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_left_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_left_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_right_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_right_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_one_left_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_one_left_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_one_right_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_one_right_;

  rclcpp_action::GoalResponse handle_move_joints_goal(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse
  handle_move_to_pose_goal(const rclcpp_action::GoalUUID &uuid,
                           std::shared_ptr<const MoveToPose::Goal> goal);

  rclcpp_action::CancelResponse
  handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  rclcpp_action::CancelResponse
  handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void serve_get_hand_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request,
                               std::shared_ptr<GetHandToTargetCoord::Response> response,
                               bool is_right, bool is_one_rink);
  void serve_get_hand_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request,
                            std::shared_ptr<GetHandToTargetTF::Response> response, bool is_right,
                            bool is_one_rink);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_left_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_right_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_hand_left_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_hand_right_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_body_joint_control_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

inline geometry_msgs::msg::Vector3
JointActionServer::get_euler_from_quat(const geometry_msgs::msg::Quaternion &msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;
}

inline geometry_msgs::msg::Quaternion
JointActionServer::get_quat_from_euler(const geometry_msgs::msg::Vector3 &euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_mini

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_mini::JointActionServer)