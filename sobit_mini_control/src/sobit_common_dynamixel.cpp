#include <sobit_mini_control/sobit_common_dynamixel.h>

SobitCommonDynamixel::SobitCommonDynamixel() {
  this->dxl_comm_result      = COMM_TX_FAIL;
  this->dxl_error            = 0;
  this->dxl_present_position = 0;
  this->dxl_speed            = 0;
  this->dxl_load             = 0;
  this->dxl_voltage          = 0;
  this->dxl_temperature      = 0;
  this->dxl_operation_mode   = 3;
  this->dxl_moving           = false;
  this->velocity_val         = 35;
  this->acceleration_val     = 3;
  this->torque_limit_val     = 100;
}

SobitCommonDynamixel::~SobitCommonDynamixel() {}

int SobitCommonDynamixel::getch() {
#if defined(__linux__)
  struct termios oldt, newt;
  int            ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int SobitCommonDynamixel::kbnit() {
#if defined(__linux__)
  struct termios oldt, newt;
  int            ch;
  int            oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void SobitCommonDynamixel::openPort() {
  while (1) {
    if (portHandler->openPort()) {
      portHandler->setBaudRate(BAUDRATE);
      printf("Dynamixel Connection Success.\n");
      break;
    } else {
      printf("Dynamixel Connection Error.\n");
      usleep(1);
    }
  }
}

void SobitCommonDynamixel::closePort() { portHandler->closePort(); }

void SobitCommonDynamixel::initializeDynamixel() {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      setTorqueEnable(i);
      setAcceleration(i, acceleration_val);
      setVelocity(i, velocity_val);
      setGrouopRead1(i);
      if (i == 20) {
        setTorqueLimit(i);
      }
    }
  }
}

void SobitCommonDynamixel::setVelocity(int id, int value) {
  this->dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, PROFILE_VELOCITY, value, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
}

void SobitCommonDynamixel::setTorqueEnable(int id) {
  // Enable Dynamixel TORQUE
  this->dxl_comm_result =
      packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  else
    printf("Dynamixel has been successfully connected.\n");
}

void SobitCommonDynamixel::setTorqueDisable(int id) {
  // disable dynamixel torque
  this->dxl_comm_result ==
      packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
}

void SobitCommonDynamixel::setAcceleration(int id, int value) {
  this->dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, PROFILE_ACCELERATION, value, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
}

void SobitCommonDynamixel::setTorqueLimit(int id) {
  // set torque_limit
  this->dxl_comm_result =
      packetHandler->write2ByteTxRx(portHandler, id, TORQUE_LIMIT, this->torque_limit_val, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
}

void SobitCommonDynamixel::setPosition(int id, int pos) {
  this->dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, pos, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
}

void SobitCommonDynamixel::setGrouopRead1(int id) {
  dxl_addparam_result = readPositonGroup1.addParam(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_addparam_result != true) fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", id);
}

void SobitCommonDynamixel::setGrouopRead2(int id) {
  dxl_addparam_result = readPositonGroup2.addParam(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_addparam_result != true) fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", id);
}

int SobitCommonDynamixel::getCurrentPosition(int id) {
  // read current_position
  this->dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, id, ADDR_PRO_PRESENT_POSITION, (uint32_t *)&this->dxl_present_position, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  // std::cout << "getCurrentPosition = " << this->dxl_present_position << std::endl;
  return this->dxl_present_position;
}

int SobitCommonDynamixel::getCurrentSpeed(int id) {
  // read current_speed
  this->dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, id, ADDR_PRO_PRESENT_VELOCITY, (uint32_t *)&this->dxl_speed, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  // std::cout << "getCurrentSpeed = " << this->dxl_speed << std::endl;
  return this->dxl_speed;
}

int SobitCommonDynamixel::getCurrentLoad(int id) {
  // read current_load
  this->dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, id, ADDR_PRO_PRESENT_LOAD, (uint16_t *)&this->dxl_load, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  // std::cout << "getCurrentLoad = " << this->dxl_load << std::endl;
  return this->dxl_load;
}

int SobitCommonDynamixel::getCurrentVoltage(int id) {
  // read current_voltage
  this->dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, id, ADDR_PRO_PRESENT_INPUT_VOLTAGE, (uint16_t *)&this->dxl_voltage, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  // std::cout << "getCurrentVoltage = " << this->dxl_voltage << std::endl;
  return this->dxl_voltage;
}

int SobitCommonDynamixel::getCurrentTemperature(int id) {
  // read current_temperature
  this->dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, id, ADDR_PRO_PRESENT_TEMPERATURE, (uint32_t *)&this->dxl_temperature, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  // std::cout << "getCurrentTemperature = " << this->dxl_temperature << std::endl;
  return this->dxl_temperature;
}

bool SobitCommonDynamixel::getCurrentMovingStatus(int value) {
  // judge moving_state
  if (value == 0)
    this->dxl_moving = false;
  else
    this->dxl_moving = true;
  return this->dxl_moving;
}

int SobitCommonDynamixel::getOperationMode(int id) {
  // read operation_mode
  this->dxl_comm_result = packetHandler->read1ByteTxRx(
      portHandler, id, OPERATING_MODE, (uint8_t *)&this->dxl_operation_mode, &this->dxl_error);
  if (this->dxl_comm_result != COMM_SUCCESS)
    packetHandler->getTxRxResult(this->dxl_comm_result);
  else if (this->dxl_error != 0)
    packetHandler->getRxPacketError(this->dxl_error);
  std::cout << "getOperationMode = " << this->dxl_operation_mode << std::endl;
  return this->dxl_operation_mode;
}

int SobitCommonDynamixel::readCurrentPosition1(int id) {
  // Bulkread present position and LED status
  // dxl_comm_result = readPositonGroup.txRxPacket();
  // if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Check if groupbulkread data of Dynamixel#id is available
  dxl_getdata_result = readPositonGroup1.isAvailable(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_getdata_result != true) {
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", id);
    this->can_move = false;
    return -1;
  } else {
    if (this->can_move == false) {
      // initializeDynamixel();
      this->can_move = true;
    }
    // Get present position value
    return readPositonGroup1.getData(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  }
}

int SobitCommonDynamixel::readCurrentPosition2(int id) {
  // Bulkread present position and LED status
  // dxl_comm_result = readPositonGroup.txRxPacket();
  // if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Check if groupbulkread data of Dynamixel#id is available
  dxl_getdata_result = readPositonGroup2.isAvailable(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_getdata_result != true) {
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", id);
    this->can_move = false;
    return -1;
  } else {
    if (this->can_move == false) {
      // initializeDynamixel();
      this->can_move = true;
    }
    // Get present position value
    return readPositonGroup2.getData(id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  }
}

void SobitCommonDynamixel::addPositionToStorage(int id, int pos) {
  uint8_t param_goal_position[4];
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos));
  if (id == 10 || id == 11 || id == 20 || id == 21) {
    dxl_addparam_result =
        writeGoalGroup1.addParam(id, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
  } else {
    dxl_addparam_result =
        writeGoalGroup2.addParam(id, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
  }
  if (dxl_addparam_result != true) {
    fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
  }
}

void SobitCommonDynamixel::writeGoalPositon() {
  // Bulkwrite goal position
  dxl_comm_result = writeGoalGroup1.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  dxl_comm_result = writeGoalGroup2.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  // Clear bulkwrite parameter storage
  writeGoalGroup1.clearParam();
  writeGoalGroup2.clearParam();
}