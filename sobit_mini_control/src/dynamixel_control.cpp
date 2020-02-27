#include "sobit_mini_control/dynamixel_control.h"

namespace dynamixel_control {
DynamixelControl::DynamixelControl(std::string name,
                                   uint8_t     dxl_id,
                                   int32_t     center,
                                   double      home,
                                   uint8_t     mode,
                                   uint32_t    dxl_vel_lim,
                                   uint32_t    dxl_acc_lim,
                                   uint16_t    dxl_current_lim,
                                   uint16_t    dxl_pos_i_gain,
                                   double      gear_ratio) {
  name_            = name;
  id_              = dxl_id;
  pos_             = 0.0;
  vel_             = 0.0;
  eff_             = 0.0;
  cmd_             = 0.0;
  torque_          = false;
  center_          = center;
  home_            = home;
  connect_         = false;
  ope_mode_        = mode;
  dxl_vel_lim_     = dxl_vel_lim;
  dxl_acc_lim_     = dxl_acc_lim;
  dxl_current_lim_ = dxl_current_lim;
  dxl_pos_i_gain_  = dxl_pos_i_gain;
  gear_ratio_      = gear_ratio;
}
}  // namespace dynamixel_control