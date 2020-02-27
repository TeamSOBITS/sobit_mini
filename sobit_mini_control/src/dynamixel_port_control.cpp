#include "sobit_mini_control/dynamixel_port_control.h"
#include <iostream>

namespace dynamixel_port_control {
DynamixelPortControl::DynamixelPortControl(ros::NodeHandle nh, dynamixel_setting::DynamixelSetting &setting) {
  pub_current_                                               = nh.advertise<sobit_common_msg::current_state_array>("/current_state_array", 100);
  dxl_res_                                                   = true;
  joint_num_                                                 = setting.getJointNum();
  std::vector<dynamixel_setting::DxlSettingParam> joint_list = setting.getDxlSettingParam();
  for (int i = 0; i < joint_num_; i++) {
    dynamixel_control::DynamixelControl work(joint_list[i].name,
                                             joint_list[i].id,
                                             joint_list[i].center,
                                             joint_list[i].home,
                                             joint_list[i].mode,
                                             joint_list[i].vel,
                                             joint_list[i].acc,
                                             joint_list[i].lim,
                                             joint_list[i].pos_i_gain,
                                             joint_list[i].gear_ratio);
    joint_list_.push_back(work);
  }

  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(dynamixel_control::PROTOCOL_VERSION);
  port_handler_   = dynamixel::PortHandler::getPortHandler(setting.getPortName().c_str());
  read_status_group_.reset(new dynamixel::GroupBulkRead(port_handler_, packet_handler_));
  write_position_group_.reset(new dynamixel::GroupBulkWrite(port_handler_, packet_handler_));

  for (int i = 0; i < joint_num_; i++) {
    uint8_t dxl_id = joint_list_[i].getDxlId();
    if (!read_status_group_->addParam(dxl_id,
                                      dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].address,
                                      dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_POSITION].length
                                          + dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].length
                                          + dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_VELOCITY].length)) {
      ROS_ERROR("[ID:%03d] groupBulkReadStatus addparam failed.", dxl_id);
    }
  }

  // open_port
  while (ros::ok()) {
    if (port_handler_->openPort()) {
      port_handler_->setBaudRate(setting.getBaudRate());
      ROS_INFO("Dynamixel Connection Success.");
      break;
    } else {
      ROS_ERROR("Dynamixel Connection Error.");
      ros::Duration(0.5).sleep();
    }
  }

  // connect and register the joint state interface
  // register the joint limits interface
  joint_limits_interface::JointLimits     joint_limits;
  joint_limits_interface::SoftJointLimits soft_joint_limits;
  for (int i = 0; i < joint_num_; i++) {
    hardware_interface::JointStateHandle state_handle(
        joint_list_[i].getJointName(), joint_list_[i].getPositionAddr(), joint_list_[i].getVelocityAddr(), joint_list_[i].getEffortAddr());
    jnt_state_interface_.registerHandle(state_handle);
  }
  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interface
  for (int i = 0; i < joint_num_; i++) {
    hardware_interface::JointHandle joint_handle(jnt_state_interface_.getHandle(joint_list_[i].getJointName()), joint_list_[i].getCommandAddr());
    jnt_pos_interface_.registerHandle(joint_handle);

    if (joint_limits_interface::getJointLimits(joint_list_[i].getJointName(), nh, joint_limits)) {
      joint_list_[i].setLimits(joint_limits);
      soft_joint_limits.k_position   = 1.0;
      soft_joint_limits.k_velocity   = 1.0;
      soft_joint_limits.max_position = joint_limits.max_position;
      soft_joint_limits.min_position = joint_limits.min_position;
      joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_handle(joint_handle, joint_limits, soft_joint_limits);
      jnt_limit_interface_.registerHandle(joint_limit_handle);
    }
  }
  registerInterface(&jnt_pos_interface_);
  registerInterface(&jnt_limit_interface_);
}

bool DynamixelPortControl::read(ros::Time time, ros::Duration period) {
  // read_status
  dxl_res_            = true;
  int dxl_comm_result = read_status_group_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    dxl_res_ = false;
    ROS_ERROR("group_status_read->txRxPacket failed.");
    return false;
  }
  readPosition(time, period);
  readVelocity(time, period);
  readCurrent(time, period);
  return true;
}

void DynamixelPortControl::readPosition(ros::Time time, ros::Duration period) {
  for (int i = 0; i < joint_num_; i++) {
    uint8_t dxl_id             = joint_list_[i].getDxlId();
    bool    dxl_getdata_result = read_status_group_->isAvailable(dxl_id,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_POSITION].address,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_POSITION].length);
    if (dxl_getdata_result) {
      int32_t present_pos = read_status_group_->getData(dxl_id,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_POSITION].address,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_POSITION].length);
      joint_list_[i].setDxlPresentPos(present_pos);
      double rad = joint_list_[i].dxlPos2Rad(present_pos);
      joint_list_[i].setPosition(rad);
    }
  }
}

void DynamixelPortControl::readVelocity(ros::Time time, ros::Duration period) {
  for (int i = 0; i < joint_num_; i++) {
    uint8_t dxl_id             = joint_list_[i].getDxlId();
    bool    dxl_getdata_result = read_status_group_->isAvailable(dxl_id,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_VELOCITY].address,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_VELOCITY].length);
    if (dxl_getdata_result) {
      int32_t present_vel = read_status_group_->getData(dxl_id,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_VELOCITY].address,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_VELOCITY].length);
      joint_list_[i].setDxlPresentVel(present_vel);
      double vel = joint_list_[i].dxlVel2RadPS(present_vel);
      joint_list_[i].setVelocity(vel);
    }
  }
}

void DynamixelPortControl::readCurrent(ros::Time time, ros::Duration period) {
  sobit_common_msg::current_state_array current_state_array;
  for (int i = 0; i < joint_num_; i++) {
    uint8_t dxl_id             = joint_list_[i].getDxlId();
    bool    dxl_getdata_result = read_status_group_->isAvailable(dxl_id,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].address,
                                                              dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].length);
    if (dxl_getdata_result) {
      uint32_t dxl_present_current = read_status_group_->getData(dxl_id,
                                                                 dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].address,
                                                                 dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].length);
      double   current             = joint_list_[i].dxlCurrent2Current(dxl_present_current);
      double   effort              = joint_list_[i].dxlCurrent2Effort(dxl_present_current);
      joint_list_[i].setCurrent(current);
      joint_list_[i].setEffort(effort);
      sobit_common_msg::current_state current_state;
      current_state.joint_name = joint_list_[i].getJointName();
      current_state.current_ma = joint_list_[i].getCurrent();
      current_state_array.current_state_array.push_back(current_state);
    }
  }
  pub_current_.publish(current_state_array);
}

void DynamixelPortControl::write(ros::Time time, ros::Duration period) {
  // write position
  for (int i = 0; i < joint_num_; i++) {
    uint8_t                             dxl_id = joint_list_[i].getDxlId();
    double                              cmd    = joint_list_[i].getCommand();
    joint_limits_interface::JointLimits lim    = joint_list_[i].getJointLimit();
    if (cmd > lim.max_position || cmd < lim.min_position) {
      continue;
    }
    int32_t  dxl_pos   = joint_list_[i].rad2DxlPos(cmd);
    uint8_t *goal_data = joint_list_[i].getDxlGoalPosAddr();
    goal_data[0]       = DXL_LOBYTE(DXL_LOWORD(dxl_pos));
    goal_data[1]       = DXL_HIBYTE(DXL_LOWORD(dxl_pos));
    goal_data[2]       = DXL_LOBYTE(DXL_HIWORD(dxl_pos));
    goal_data[3]       = DXL_HIBYTE(DXL_HIWORD(dxl_pos));

    bool dxl_add_param_result = write_position_group_->addParam(dxl_id,
                                                                dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_GOAL_POSITION].address,
                                                                dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_GOAL_POSITION].length,
                                                                goal_data);
    if (!dxl_add_param_result) {
      ROS_ERROR("[ID:%03d] groupBulkWrite addparam failed", dxl_id);
    }
  }
  int dxl_comm_result = write_position_group_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  }
  write_position_group_->clearParam();
}

void DynamixelPortControl::setTorque(uint8_t id, bool torque) {
  uint32_t mode            = torque ? dynamixel_control::TORQUE_ENABLE : dynamixel_control::TORQUE_DISABLE;
  uint8_t  dxl_error       = 0;
  int      dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_,
                                                        id,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_TORQUE_ENABLE].address,
                                                        dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_TORQUE_ENABLE].length,
                                                        &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  } else {
    ROS_INFO("Dynamixel has been successfully connected.");
  }
}

void DynamixelPortControl::setTorqueAll(bool torque) {
  for (int i = 0; i < joint_num_; i++) {
    joint_list_[i].setTorque(torque);
    setTorque(joint_list_[i].getDxlId(), torque);
  }
}

void DynamixelPortControl::setVelocityLim(uint8_t id, uint32_t vel_lim) {
  uint8_t dxl_error       = 0;
  int     dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PROFILE_VELOCITY].address, vel_lim, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
}

void DynamixelPortControl::setAccelerationLim(uint8_t id, uint32_t acc_lim) {
  uint8_t dxl_error       = 0;
  int     dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PROFILE_ACCELERATION].address, acc_lim, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
}

void DynamixelPortControl::setPositionIGain(uint8_t id, uint16_t i_gain) {
  uint8_t dxl_error       = 0;
  int     dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_POSITION_I_GAIN].address, i_gain, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
}

bool DynamixelPortControl::setCurrentLimit(uint8_t id, uint16_t current_lim) {
  if (!dxl_res_) {
    return false;
  }
  uint8_t dxl_error       = 0;
  int     dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_GOAL_CURRENT].address, current_lim, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
  return true;
}

void DynamixelPortControl::setOperationMode(uint8_t id, uint8_t mode) {
  uint8_t dxl_error       = 0;
  int     dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_OPE_MODE].address, mode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
}

void DynamixelPortControl::initializeSettingParam() {
  for (int i = 0; i < joint_num_; i++) {
    setAccelerationLim(joint_list_[i].getDxlId(), joint_list_[i].getDxlAccelerationLim());
    setVelocityLim(joint_list_[i].getDxlId(), joint_list_[i].getDxlVelocityLim());
    setPositionIGain(joint_list_[i].getDxlId(), joint_list_[i].getDxlPositionIGain());
    if (joint_list_[i].getOpeMode() == dynamixel_control::OPERATING_MODE_CURR_POS) {
      setOperationMode(joint_list_[i].getDxlId(), joint_list_[i].getOpeMode());  // 念の為
      setCurrentLimit(joint_list_[i].getDxlId(), joint_list_[i].getDxlCurrentLimit());
    }
  }
}

bool DynamixelPortControl::startUpPosition() {
  ros::Rate                   rate(10);
  int                         step_max = 60;
  ros::Time                   t        = getTime();
  ros::Duration               dt       = getDuration(t);
  std::vector<HomeMotionData> home_motion_data_vec;
  bool                        success = false;
  while (ros::ok()) {
    if (read(t, dt)) {
      success = true;
      break;
    } else {
      rate.sleep();
    }
  }
  if (!success) {
    return false;
  }
  for (int i = 0; i < joint_num_; i++) {
    HomeMotionData home_motion_data;
    home_motion_data.home      = joint_list_[i].rad2DxlPos(joint_list_[i].getHome());
    home_motion_data.home_rad  = joint_list_[i].getHome();
    home_motion_data.start     = joint_list_[i].getDxlPresentPos();
    home_motion_data.start_rad = joint_list_[i].getPosition();
    home_motion_data.step_rad  = (home_motion_data.home > home_motion_data.start) ? ((home_motion_data.home_rad - home_motion_data.start_rad) / step_max)
                                                                                 : -((home_motion_data.start_rad - home_motion_data.home_rad) / step_max);
    joint_list_[i].setCommand(joint_list_[i].getPosition());
    home_motion_data_vec.push_back(home_motion_data);
    std::cout << "ID : " << (int)(joint_list_[i].getDxlId()) << ", home : " << home_motion_data.home_rad << ", start : " << home_motion_data.start_rad
              << ", step : " << home_motion_data.step_rad << std::endl;
  }
  write(t, dt);
  setTorqueAll(true);

  for (int step = 0; step < step_max; step++) {
    dt = getDuration(t);
    t  = getTime();
    read(t, dt);
    for (int i = 0; i < joint_num_; i++) {
      joint_list_[i].setCommand(joint_list_[i].getCommand() + home_motion_data_vec[i].step_rad);
    }
    write(t, dt);
    rate.sleep();
  }
  for (int i = 0; i < joint_num_; i++) {
    joint_list_[i].setCommand(home_motion_data_vec[i].home_rad);
  }
  write(t, dt);
  return true;
}

// Debug

int DynamixelPortControl::getCurrentLoad(int id) {
  uint8_t  dxl_error = 0;
  uint16_t dxl_current;
  int      dxl_comm_result = packet_handler_->read2ByteTxRx(
      port_handler_, id, dynamixel_control::DYNAMIXEL_REG_TABLE[dynamixel_control::TABLE_ID_PRESENT_CURRENT].address, &dxl_current, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packet_handler_->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packet_handler_->getRxPacketError(dxl_error);
  }
  ROS_INFO("[%03d] : %d", id, dxl_current);
  return dxl_current;
}

}  // namespace dynamixel_port_control