#ifndef DYNAMIXEL_JOINT_CONTROL_H_
#define DYNAMIXEL_JOINT_CONTROL_H_
#include <string>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace dynamixel_control {
static const int      PROTOCOL_VERSION = 2;
static const int      TORQUE_ENABLE    = 1;
static const int      TORQUE_DISABLE   = 0;
static const uint16_t REG_LENGTH_BYTE  = 1;
static const uint16_t REG_LENGTH_WORD  = 2;
static const uint16_t REG_LENGTH_DWORD = 4;

typedef struct {
  std::string name;
  uint16_t    address;
  uint16_t    length;
  uint32_t    default_val;
} DynamixelRegTable;

typedef enum {
  TABLE_ID_RETURN_DELAY = 0,
  TABLE_ID_DRIVE_MODE,
  TABLE_ID_OPE_MODE,
  TABLE_ID_HOMING_OFFSET,
  TABLE_ID_MOVING_THRESHOLD,
  TABLE_ID_TEMP_LIMIT,
  TABLE_ID_MAX_VOL_LIMIT,
  TABLE_ID_MIN_VOL_LIMIT,
  TABLE_ID_CURRENT_LIMIT,
  TABLE_ID_SHUT_DOWN,
  TABLE_ID_TORQUE_ENABLE,
  TABLE_ID_VELOCITY_I_GAIN,
  TABLE_ID_VELOCITY_P_GAIN,
  TABLE_ID_POSITION_D_GAIN,
  TABLE_ID_POSITION_I_GAIN,
  TABLE_ID_POSITION_P_GAIN,
  TABLE_ID_BUS_WATCHDOG,
  TABLE_ID_GOAL_CURRENT,
  TABLE_ID_GOAL_VELOCITY,
  TABLE_ID_PROFILE_ACCELERATION,
  TABLE_ID_PROFILE_VELOCITY,
  TABLE_ID_GOAL_POSITION,
  TABLE_ID_PRESENT_CURRENT,
  TABLE_ID_PRESENT_VELOCITY,
  TABLE_ID_PRESENT_POSITION,
  TABLE_ID_PRESENT_TEMP,
} TableID;

static const DynamixelRegTable DYNAMIXEL_REG_TABLE[] = {
    /*  NAME                ADDR    LEN INIT  DEFAULT_VAL */
    {"RETURN_DELAY_TIME", 9, REG_LENGTH_BYTE, 250}, {"DRIVE_MODE", 10, REG_LENGTH_BYTE, 0},
    {"OPERATION_MODE", 11, REG_LENGTH_BYTE, 3},     {"HOMING_OFFSET", 20, REG_LENGTH_DWORD, 0},
    {"MOVING_THRESHOLD", 24, REG_LENGTH_DWORD, 10}, {"TEMPRATURE_LIMIT", 31, REG_LENGTH_BYTE, 80},
    {"MAX_VOL_LIMIT", 32, REG_LENGTH_WORD, 160},    {"MIN_VOL_LIMIT", 34, REG_LENGTH_WORD, 95},
    {"CURRENT_LIMIT", 38, REG_LENGTH_WORD, 1193},   {"SHUTDOWN", 63, REG_LENGTH_BYTE, 52},
    {"TORQUE_ENABLE", 64, REG_LENGTH_BYTE, 0},      {"VELOCITY_I_GAIN", 76, REG_LENGTH_WORD, 1920},
    {"VELOCITY_P_GAIN", 78, REG_LENGTH_WORD, 100},  {"POSITION_D_GAIN", 80, REG_LENGTH_WORD, 0},
    {"POSITION_I_GAIN", 82, REG_LENGTH_WORD, 0},    {"POSITION_P_GAIN", 84, REG_LENGTH_WORD, 800},
    {"BUS_WATCHDOG", 98, REG_LENGTH_BYTE, 0},       {"GOAL_CURRENT", 102, REG_LENGTH_WORD, 0},
    {"GOAL_VELOCITY", 104, REG_LENGTH_WORD, 0},     {"PROFILE_ACCELERATION", 108, REG_LENGTH_DWORD, 0},
    {"PROFILE_VELOCITY", 112, REG_LENGTH_DWORD, 0}, {"GOAL_POSITION", 116, REG_LENGTH_DWORD, 0},
    {"PRESENT_CURRENT", 126, REG_LENGTH_WORD, 0},   {"PRESENT_VELOCITY", 128, REG_LENGTH_DWORD, 0},
    {"PRESENT_POSITION", 132, REG_LENGTH_DWORD, 0}, {"PRESENT_TEMPRATURE", 146, REG_LENGTH_BYTE, 0},
};

typedef struct {
  uint8_t  dxl_id;
  uint8_t  return_delay_time;
  uint8_t  drive_mode;
  uint8_t  operation_mode;
  uint16_t moving_threshold;
  int32_t  homing_offset;
  uint8_t  temprature_limit;
  uint8_t  max_vol_limit;
  uint8_t  min_vol_limit;
  uint16_t current_limit;
  uint8_t  torque_enable;
  uint16_t velocity_i_gain;
  uint16_t velocity_p_gain;
  uint16_t position_d_gain;
  uint16_t position_i_gain;
  uint16_t position_p_gain;
} DynamixelJointParam;

typedef enum {
  OPERATING_MODE_CURRENT,
  OPERATING_MODE_VELOCITY,
  OPERATING_MODE_POSITION = 3,
  OPERATING_MODE_EXT_POS,
  OPERATING_MODE_CURR_POS,
  OPERATING_MODE_PWM = 16,
} OperatingModeID;

class DynamixelControl {
 public:
  DynamixelControl(std::string name,
                   uint8_t     dxl_id,
                   int32_t     center,
                   double      home,
                   uint8_t     mode,
                   uint32_t    dxl_vel_lim,
                   uint32_t    dxl_acc_lim,
                   uint16_t    dxl_current_lim,
                   uint16_t    dxl_pos_i_gain,
                   double      gear_ratio);
  virtual ~DynamixelControl(){};
  void setJoinName(std::string set_name) { name_ = set_name; }
  void setDynamixelID(uint8_t set_id) { id_ = set_id; }
  void setPosition(double set_rad) { pos_ = set_rad; }
  void setVelocity(double set_vel) { vel_ = set_vel; }
  void setEffort(double set_eff) { eff_ = set_eff; }
  void setCommand(double set_cmd) { cmd_ = set_cmd; }
  void setCurrent(double set_curr) { current_ = set_curr; }
  void setTempature(double set_temp) { temprature_ = set_temp; }
  void setTorque(bool set_torque) { torque_ = set_torque; }
  void setCenter(uint32_t set_center) { center_ = set_center; }
  void setHome(double set_home) { home_ = set_home; }
  void setConnect(bool set_connect) { connect_ = set_connect; }
  void setOpeMode(uint8_t set_ope_mode) { ope_mode_ = set_ope_mode; }
  void setGearRatio(double set_gear_ratio) { gear_ratio_ = set_gear_ratio; }
  void setLimits(joint_limits_interface::JointLimits& set_limits) { limits_ = set_limits; }
  void setDxlPresentPos(int32_t set_dxl_pos) { dxl_present_pos_ = set_dxl_pos; }
  void setDxlPresentVel(int32_t set_dxl_vel) { dxl_present_vel_ = set_dxl_vel; }
  void setDxlPresentAcc(int32_t set_dxl_acc) { dxl_present_acc_ = set_dxl_acc; }
  void setDxlVelocityLim(uint32_t set_dxl_vel_lim) { dxl_vel_lim_ = set_dxl_vel_lim; }
  void setDxlAccelerationLim(uint32_t set_dxl_acc_lim) { dxl_acc_lim_ = set_dxl_acc_lim; }
  void setDxlTorqueLimit(uint16_t set_torque_lim) { dxl_current_lim_ = set_torque_lim; }
  void setDxlPositionIGain(uint16_t set_pos_i_gain) { dxl_pos_i_gain_ = set_pos_i_gain; }
  void setParam(DynamixelJointParam set_param) { param_ = set_param; }

  std::string                         getJointName() { return name_; }
  uint8_t                             getDxlId() { return id_; }
  double                              getPosition() { return pos_; }
  double*                             getPositionAddr() { return &pos_; }
  double                              getVelocity() { return vel_; }
  double*                             getVelocityAddr() { return &vel_; }
  double                              getEffor() { return eff_; }
  double*                             getEffortAddr() { return &eff_; }
  double                              getCommand() { return cmd_; }
  double*                             getCommandAddr() { return &cmd_; }
  double                              getCurrent() { return current_; }
  double                              getTemprature() { return temprature_; }
  bool                                getTorque() { return torque_; }
  int32_t                             getCenter() { return center_; }
  double                              getHome() { return home_; }
  joint_limits_interface::JointLimits getJointLimit() { return limits_; }
  bool                                isConnect() { return connect_; }
  uint8_t                             getOpeMode() { return ope_mode_; }
  double                              getGearRatio() { return gear_ratio_; }
  uint8_t*                            getDxlGoalPosAddr() { return dxl_goal_pos_; }
  int32_t                             getDxlPresentPos() { return dxl_present_pos_; }
  int32_t                             getDxlPresentVel() { return dxl_present_vel_; }
  int32_t                             getDxlPresentAcc() { return dxl_present_acc_; }
  uint16_t                            getDxlTemprature() { return dxl_temprature_; }
  uint32_t                            getDxlVelocityLim() { return dxl_vel_lim_; }
  uint32_t                            getDxlAccelerationLim() { return dxl_acc_lim_; }
  uint16_t                            getDxlCurrentLimit() { return dxl_current_lim_; }
  uint16_t                            getDxlPositionIGain() { return dxl_pos_i_gain_; }
  DynamixelJointParam                 getParam() { return param_; }

  double   dxlPos2Rad(int32_t dxl_pos) { return (dxl_pos - center_) * ((M_PI * 2) / 4096) * (1 / gear_ratio_); }
  int32_t  rad2DxlPos(double rad) { return center_ + rad / (M_PI * 2) * 4096 * gear_ratio_; }
  double   dxlCurrent2Current(int32_t dxl_current) { return dxl_current * 2.69; }
  uint32_t current2DxlCurrent(double current) { return (uint32_t)(current / 2.69); }
  double   dxlCurrent2Effort(int16_t dxl_current) { return dxl_current * 2.69 * 1.79 * 0.001; }
  double   dxlVel2RadPS(int32_t dxl_vel) { return dxl_vel * 0.229 * 0.1047; }

 private:
  std::string name_;
  uint8_t     id_;
  double      pos_;
  double      vel_;
  double      eff_;
  double      cmd_;
  double      current_;
  double      temprature_;
  bool        torque_;
  int32_t     center_;
  double      home_;
  bool        connect_;
  uint8_t     ope_mode_;
  double      gear_ratio_;

  joint_limits_interface::JointLimits limits_;

  uint8_t  dxl_goal_pos_[4];
  int32_t  dxl_present_pos_;
  int32_t  dxl_present_vel_;
  int32_t  dxl_present_acc_;
  uint16_t dxl_temprature_;
  uint32_t dxl_vel_lim_;
  uint32_t dxl_acc_lim_;
  uint16_t dxl_current_lim_;
  uint16_t dxl_pos_i_gain_;

  DynamixelJointParam param_;
};
}  // namespace dynamixel_control

#endif /* DYNAMIXEL_JOINT_CONTROL_H_ */