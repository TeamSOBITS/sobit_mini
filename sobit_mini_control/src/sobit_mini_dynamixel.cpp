#include <sobit_mini_control/sobit_mini_hw.h>

SobitMiniDynamixel::SobitMiniDynamixel() {
  used_dynamixel_id[1]  = 1;
  used_dynamixel_id[2]  = 2;
  used_dynamixel_id[3]  = 3;
  used_dynamixel_id[4]  = 4;
  used_dynamixel_id[10] = 10;
  used_dynamixel_id[11] = 11;
  used_dynamixel_id[12] = 12;
  used_dynamixel_id[13] = 13;
  used_dynamixel_id[20] = 20;
  used_dynamixel_id[21] = 21;
  used_dynamixel_id[22] = 22;
  used_dynamixel_id[23] = 23;

  used_dynamixel_name[1]  = "body_lift_joint";
  used_dynamixel_name[2]  = "body_roll_joint";
  used_dynamixel_name[3]  = "head_tilt_joint";
  used_dynamixel_name[4]  = "head_pan_joint";
  used_dynamixel_name[10] = "right_shoulder_roll_joint";
  used_dynamixel_name[11] = "right_shoulder_flex_joint";
  used_dynamixel_name[12] = "right_wrist_flex_joint";
  used_dynamixel_name[13] = "right_hand_motor_joint";
  used_dynamixel_name[20] = "left_shoulder_roll_joint";
  used_dynamixel_name[21] = "left_shoulder_flex_joint";
  used_dynamixel_name[22] = "left_wrist_flex_joint";
  used_dynamixel_name[23] = "left_hand_motor_joint";
  std::ofstream ofs("/home/kento-nasu/catkin_ws/src/sobit_mini/sobit_mini_control/config/debug.csv");
  ofs << "ID,"
      << "joint_name,"
      << "dynamixel_pos,"
      << "joint_pos" << std::endl;
}

SobitMiniDynamixel::~SobitMiniDynamixel() {}

void SobitMiniDynamixel::initializeDynamixel() {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      setTorqueEnable(i);
      setAcceleration(i, acceleration_val);
      setVelocity(i, velocity_val);
      setGrouopRead(i);
      if (i == 13 || i == 23) {
        setTorqueLimit(i);
      }
    }
  }
}

void SobitMiniDynamixel::writeDynamixelMotors(const trajectory_msgs::JointTrajectory &pose) {
  for (int i = 0; i < pose.joint_names.size(); i++) {
    int joint_id = getJointNumber(pose.joint_names[i]);
    if (!joint_id) {
      continue;
    }
    int joint_pos = toBit(joint_id, pose.points[0].positions[i]);
    if (std::abs(joint_pos == saved_dxl_goal_position[joint_id])) {
      continue;
    }
    addPositionToStorage(joint_id, joint_pos);
    saved_dxl_goal_position[joint_id] = joint_pos;
  }
  writeGoalPositon();
}

sensor_msgs::JointState SobitMiniDynamixel::readDynamixelMotors() {
  sensor_msgs::JointState pose;
  dxl_comm_result = readPositonGroup.txRxPacket();
  std::ofstream ofs("/home/kento-nasu/catkin_ws/src/sobit_mini/sobit_mini_control/config/debug.csv", std::ios::app);
  if (!ofs) {
    ROS_ERROR("ERROR\n");
  }
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      std::string joint_name    = used_dynamixel_name[i];
      int         dynamixel_pos = readCurrentPosition(used_dynamixel_id[i]);
      float       joint_pos     = toRad(joint_name, dynamixel_pos);
      if (dynamixel_pos == -1) {
        joint_pos = saved_dxl_goal_position[i];
      }
      // std::cout << joint_name << " : " << dynamixel_pos << std::endl;
      if (i == 1) {
        ofs << i << "," << joint_name << "," << dynamixel_pos << "," << joint_pos << std::endl;
      }
      pose.name.push_back(joint_name);
      pose.position.push_back(joint_pos);
    }
  }
  return pose;
}

int SobitMiniDynamixel::getJointNumber(std::string joint_name) {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_name[i] == joint_name) {
      return used_dynamixel_id[i];
    }
  }
  return 0;
}

float SobitMiniDynamixel::toBit(int id, float rad) {
  if (id == 1) {
    float bit = 90163.9344262 * rad + 2048;
    if (bit > 29548) bit = 29548;
    if (bit < 2048) bit = 2048;
    return bit;
  } else if (id == 2) {
    return 2048 -
           (rad * 5258 /
            M_PI_2);  // 2048は胴が正面の状態の値,5258は胴が正面を向いている状態から左(-3210)・右(7306)向きの状態を引いた値,
                      // 1.57は正面の状態から左・右の向きに向いた時のrad差
  } else if (id == 3) {
    return 2250 - (rad * 750 /
                   0.5);  // 2250は顔が前向きの状態の値,750は顔が前向きの状態から下(1500)・上(3000)向きの状態を引いた値,
                          // 0.5は前向き状態から下・上の向きに向いた時のrad差
  } else if (id == 4) {
    return 3083 + (rad * 2062 / 3.14);
    // 3083は顔がまっすぐの状態の値,
    // 2062は顔がまっすぐの状態から左(4106)・右(2044)の状態を引いた値、3.14はまっすぐの状態から左・右の向きに向いた時のrad差
  } else if (id == 10)
    return 2048 - (rad / 0.001533203125);
  else if (id == 11) {
    if (rad > 1.57)                                   //平行時より上 1.57 < 3.14
      return ((3.14 - rad) / 0.001533203125) + 1024;  // 2048以上
    else                                              //平行時より下 0 < 3.14
      return ((1.57 - rad) / 0.001533203125) + 2048;  // 2048以下
  } else if (id == 12) {
    if (rad <= 0)                             //内側に曲がる -1.57 < 0
      return 2048 + (rad / -0.001533203125);  // 2048以上
    else                                      //外側に曲がる
      return 2048 - (rad / 0.001533203125);   // 2048以下
  } else if (id == 13) {
    return 2200 -
           (rad * 1000 /
            M_PI_2);  // 2200は閉めた状態　1000は開きの状態(1200)から閉めの状態(2200)を引いた値, 1.57は閉めから開き状態の時のrad差
  } else if (id == 20)
    return 2048 - (rad / -0.001533203125);
  else if (id == 21)
    return 1024 + (rad / -0.001533203125);
  else if (id == 22) {
    if (rad <= 0)                             //外側に曲がる -1.57 < 0
      return 2048 + (rad / -0.001533203125);  // 2048以上
    else                                      //内側に曲がる
      return 2048 - (rad / 0.001533203125);   // 2048以下
  } else if (id == 23) {
    return 1900 + (rad * 1000 /
                   M_PI_2);  // 1900は閉めた状態
                             // 1000は開きの状態(2900)から閉めの状態(1900)を引いた値, 1.57は閉めから開き状態の時のrad差
  } else
    return -1;
}

float SobitMiniDynamixel::toRad(std::string joint_name, int bit) {
  if (joint_name == "body_lift_joint") {
    return (bit - 2048) / 90163.9344262;
  } else if (joint_name == "body_roll_joint") {
    return (2048 - bit) * 1.57 / 5258;
  } else if (joint_name == "head_tilt_joint") {
    return (2250 - bit) * 0.5 / 750;
  } else if (joint_name == "head_pan_joint") {
    return (bit - 3083) * 3.14 / 2062;
  } else if (joint_name == "right_shoulder_roll_joint") {
    return (2048 - bit) * 0.001533203125;
  } else if (joint_name == "right_shoulder_flex_joint") {
    if (bit < 2048) {
      return 3.14 - (0.001533203125 * (bit - 1024));  // 3.17から3.14に変更した
    } else {
      return 1.57 - (0.001533203125 * (bit - 2048));
    }
  } else if (joint_name == "right_wrist_flex_joint") {
    if (bit >= 2048) {
      return (bit - 2048) * -0.001533203125;
    } else {
      return (2048 - bit) * 0.001533203125;
    }
  } else if (joint_name == "right_hand_motor_joint") {
    return (2200 - bit) * M_PI_2 / 1000;
  } else if (joint_name == "left_shoulder_roll_joint") {
    return (2048 - bit) * -0.001533203125;
  } else if (joint_name == "left_shoulder_flex_joint") {
    return (bit - 1024) * -0.001533203125;
  } else if (joint_name == "left_wrist_flex_joint") {
    if (bit >= 2048)  // 1.57
    {
      return (bit - 2048) * -0.001533203125;
    } else  // 1.57
    {       // -1.57
      return (2048 - bit) * 0.001533203125;
    }
  } else if (joint_name == "left_hand_motor_joint") {
    return (bit - 1900) * M_PI_2 / 1000;
  }
  return -1;
}