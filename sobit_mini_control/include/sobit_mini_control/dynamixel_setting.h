#ifndef DYNAMIXEL_SETTING_H_
#define DYNAMIXEL_SETTING_H_

#include <string>
#include <vector>

#include <ros/ros.h>

namespace dynamixel_setting {
/* SETTING KEY STRINGS */
static const std::string KEY_DXL_PORT        = "dynamixel_port";
static const std::string KEY_PORT_NAME       = "/port_name";
static const std::string KEY_BAUDARTE        = "/baud_rate";
static const std::string KEY_JOINTS          = "/joints";
static const std::string KEY_JOINTS_ID       = "/id";
static const std::string KEY_JOINTS_CENTER   = "/center";
static const std::string KEY_JOINTS_HOME     = "/home";
static const std::string KEY_JOINTS_MODE     = "/mode";
static const std::string KEY_JOINTS_VEL      = "/vel";
static const std::string KEY_JOINTS_ACC      = "/acc";
static const std::string KEY_JOINTS_LIM      = "/lim";
static const std::string KEY_POSITION_I_GAIN = "/pos_i_gain";
static const std::string KEY_GEAR_RATIO      = "/gear_ratio";

typedef struct {
  std::string name;
  uint8_t     id;
  int16_t     center;
  double      home;
  uint8_t     mode;
  uint32_t    vel;
  uint32_t    acc;
  uint16_t    lim;
  uint8_t     pos_i_gain;
  double      gear_ratio;
} DxlSettingParam;

class DynamixelSetting {
 public:
  DynamixelSetting(ros::NodeHandle nh);
  ~DynamixelSetting(){};
  bool                         load();
  std::string                  getPortName() { return port_name_; }
  std::vector<DxlSettingParam> getDxlSettingParam() { return joint_list_; }
  uint8_t                      getJointNum() { return joint_num_; }
  uint32_t                     getBaudRate() { return port_baud_rate_; }

 private:
  ros::NodeHandle              nh_;
  uint8_t                      joint_num_;
  std::string                  port_name_;
  uint32_t                     port_baud_rate_;
  std::vector<DxlSettingParam> joint_list_;

  bool loadPortName();
  bool loadBaudRate();
  bool loadJointList();
  bool loadJointParam();
};

}  // namespace dynamixel_setting

#endif /* DYNAMIXEL_SETTING_H_ */