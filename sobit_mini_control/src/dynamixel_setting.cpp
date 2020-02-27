#include "sobit_mini_control/dynamixel_setting.h"

#include "sobit_mini_control/dynamixel_control.h"

namespace dynamixel_setting {
DynamixelSetting::DynamixelSetting(ros::NodeHandle nh) {
  nh_        = nh;
  joint_num_ = 0;
  port_name_ = "";
  joint_list_.clear();
}

bool DynamixelSetting::load() {
  if (!loadPortName()) {
    return false;
  }
  if (!loadBaudRate()) {
    return false;
  }
  if (!loadJointList()) {
    return false;
  }
  if (!loadJointParam()) {
    return false;
  }
  return true;
}

bool DynamixelSetting::loadPortName() {
  std::string key_port_name = KEY_DXL_PORT + KEY_PORT_NAME;
  std::string port_name;
  if (!nh_.getParam(key_port_name, port_name)) {
    ROS_ERROR("Undefined key %s.", key_port_name.c_str());
    return false;
  }
  port_name_ = port_name;
  return true;
}

bool DynamixelSetting::loadBaudRate() {
  std::string key_baud_rate = KEY_DXL_PORT + KEY_BAUDARTE;
  int         baud_rate;
  if (!nh_.getParam(key_baud_rate, baud_rate)) {
    ROS_ERROR("Undefined key %s.", key_baud_rate.c_str());
    return false;
  }
  port_baud_rate_ = (uint32_t)baud_rate;
  return true;
}

bool DynamixelSetting::loadJointList() {
  std::string         key_joint_list = KEY_DXL_PORT + KEY_JOINTS;
  XmlRpc::XmlRpcValue load_joints;
  if (!nh_.getParam(key_joint_list, load_joints)) {
    ROS_ERROR("Undefined key %s.", key_joint_list.c_str());
    return false;
  }
  if (load_joints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("XmlRpc get type error! func: %s, line%d", __func__, __LINE__);
    return false;
  }
  for (int32_t i = 0; i < load_joints.size(); i++) {
    if (load_joints[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR("XmlRpc get type error! func: %s, line%d.", __func__, __LINE__);
      return false;
    }
    DxlSettingParam work;
    work.name = (std::string)load_joints[i];
    joint_list_.push_back(work);
  }
  joint_num_ = joint_list_.size();
  return true;
}

bool DynamixelSetting::loadJointParam() {
  std::string         key_base_param = KEY_DXL_PORT + "/";
  XmlRpc::XmlRpcValue load_joint_param;
  for (int8_t i = 0; i < joint_list_.size(); i++) {
    std::string key_joint_param = key_base_param + joint_list_[i].name;
    int         load_id;
    int         load_center;
    double      load_home;
    int         load_mode;
    int         load_vel;
    int         load_acc;
    int         load_lim = 0;
    int         load_pos_i_gain;
    double      gear_ratio = 1;
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_ID, load_id)) {
      ROS_ERROR("%s", (key_joint_param + KEY_JOINTS_ID).c_str());
      ROS_ERROR("Undefined %s id key func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_CENTER, load_center)) {
      ROS_ERROR("Undefined %s center func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_HOME, load_home)) {
      ROS_ERROR("Undefined %s home func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_MODE, load_mode)) {
      ROS_ERROR("Undefined %s mode func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_VEL, load_vel)) {
      ROS_ERROR("Undefined %s vel func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_JOINTS_ACC, load_acc)) {
      ROS_ERROR("Undefined %s acc func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_POSITION_I_GAIN, load_pos_i_gain)) {
      ROS_ERROR("Undefined %s i_gain func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (!nh_.getParam(key_joint_param + KEY_GEAR_RATIO, gear_ratio)) {
      ROS_ERROR("Undefined %s i_gain func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
      return false;
    }
    if (load_mode == dynamixel_control::OPERATING_MODE_CURR_POS) {
      if (!nh_.getParam(key_joint_param + KEY_JOINTS_LIM, load_lim)) {
        ROS_ERROR("Undefined %s limi func: %s, line%d.", joint_list_[i].name.c_str(), __func__, __LINE__);
        return false;
      }
    }
    joint_list_[i].id         = load_id;
    joint_list_[i].center     = load_center;
    joint_list_[i].home       = load_home;
    joint_list_[i].mode       = load_mode;
    joint_list_[i].vel        = load_vel;
    joint_list_[i].acc        = load_acc;
    joint_list_[i].lim        = load_lim;
    joint_list_[i].pos_i_gain = load_pos_i_gain;
    joint_list_[i].gear_ratio = gear_ratio;
  }
  return true;
}

}  // namespace dynamixel_setting