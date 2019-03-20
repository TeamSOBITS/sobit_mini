#ifndef SOBIT_COMMON_DYNAMIXEL_H_
#define SOBIT_COMMON_DYNAMIXEL_H_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

//dynamixel control table address
#define PROTOCOL_VERSION 2
#define OPERATING_MODE 11
#define ADDR_PRO_TORQUE_ENABLE 64
#define ADDR_PRO_HARDWARE_ERROR_STATUS 70
#define TORQUE_LIMIT 102
#define PROFILE_ACCELERATION 108
#define PROFILE_VELOCITY 112
#define ADDR_PRO_GOAL_POSITION 116
#define ADDR_PRO_MOVING_STATUS 123
#define ADDR_PRO_PRESENT_LOAD 126
#define ADDR_PRO_PRESENT_VELOCITY 136
#define ADDR_PRO_PRESENT_POSITION 132
#define ADDR_PRO_PRESENT_INPUT_VOLTAGE 144
#define ADDR_PRO_PRESENT_TEMPERATURE 146

#define BAUDRATE 57600
#define DEVICENAME "/dev/dynamixel"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define COMM_SUCCESS 0
#define COMM_TX_FAIL -1001
#define DXL_MINIMUM_POSITION_VALUE 0
#define DXL_MAXIMUM_POSITION_VALUE 4095
#define DXL_MOVING_STATUS_THRESHOLD 20
#define ESC_ASCII_VALUE 0x1b
#define LEN_PRO_GOAL_POSITION 4
#define LEN_PRO_PRESENT_POSITION 4 // 2

#define MAGNIFICATION_VALUE 651.89864690440329530934 //2048 / 3.14

class SobitCommonDynamixel : public hardware_interface::RobotHW
{
protected:
  int dxl_comm_result;
  uint8_t dxl_error;            // Dynamixel error
  int32_t dxl_present_position; // Present position
  int32_t dxl_speed;
  int16_t dxl_load;
  int16_t dxl_voltage;
  int32_t dxl_temperature;
  int32_t dxl_operation_mode;
  bool dxl_moving;                  // Communication result
  bool dxl_addparam_result = false; // addParam result
  bool dxl_getdata_result = false;
  bool can_move = true;

  int dxl_goal_position[30];
  int saved_dxl_goal_position[30] = {0};
  int used_dynamixel_id[30] = {0};

  int velocity_val;
  int acceleration_val;
  int torque_limit_val;

  //initialise PortHandler instance
  //set the port path
  //get methods and members of PortHandlerLinux
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  //initialise PacketHandler instance
  //set the protocol path
  //get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  //dynamixel::GroupSyncWrite group_sync_write = dynamixel::GroupSyncWrite::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  //dynamixel::GroupSyncRead group_sync_read = dynamixel::GroupSyncRead::GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite writeGoalGroup = dynamixel::GroupBulkWrite(portHandler, packetHandler);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead readPositonGroup = dynamixel::GroupBulkRead(portHandler, packetHandler);

  //int joint_id;
  //int joint_pos;

  std::string used_dynamixel_name[30];
public:
  SobitCommonDynamixel();
  virtual ~SobitCommonDynamixel();
  int getch();
  int kbnit();
  void openPort();
  void closePort();
  void initializeDynamixel();

protected:
  void setVelocity(int id, int value);
  void setTorqueEnable(int id);
  void setTorqueDisable(int id);
  void setAcceleration(int id, int value);
  void setTorqueLimit(int id);
  void setPosition(int id, int pos);
  void setGrouopRead(int id);
  int getCurrentPosition(int id);
  int getCurrentSpeed(int id);
  int getCurrentLoad(int id);
  int getCurrentVoltage(int id);
  int getCurrentTemperature(int id);
  bool getCurrentMovingStatus(int value);
  int getOperationMode(int id);
  int readCurrentPosition(int id);
  void addPositionToStorage(int id, int pos);
  void writeGoalPositon();


};

#endif /* SOBIT_COMMON_DYNAMIXEL_H_ */