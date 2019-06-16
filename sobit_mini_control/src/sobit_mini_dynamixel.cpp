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
}

SobitMiniDynamixel::~SobitMiniDynamixel() {}

void SobitMiniDynamixel::initializeDynamixel() {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      setTorqueEnable(i);
      setAcceleration(i, acceleration_val);
      setVelocity(i, velocity_val);
      setPositionIGain(i);
      if (i == 10 || i == 11 || i == 20 || i == 21) {
        setGrouopRead1(i);
      } else {
        setGrouopRead2(i);
      }
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
    if (joint_id == 21) {
      // ROS_INFO("[ID:%03d]: %d", joint_id, joint_pos);
    }
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
  dxl_comm_result = readPositonGroup1.txRxPacket();
  dxl_comm_result = readPositonGroup2.txRxPacket();
  /*std::ofstream ofs("/home/kento-nasu/catkin_ws/src/sobit_mini/sobit_mini_control/config/debug.csv", std::ios::app);
  if (!ofs) {
    ROS_ERROR("ERROR\n");
  }*/
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      std::string joint_name = used_dynamixel_name[i];
      int         dynamixel_pos;
      if (i == 10 || i == 11 || i == 20 || i == 21) {
        dynamixel_pos = readCurrentPosition1(i);
      } else {
        dynamixel_pos = readCurrentPosition2(i);
      }
      float joint_pos = toRad(joint_name, dynamixel_pos);
      if (dynamixel_pos == -1) {
        joint_pos = saved_dxl_goal_position[i];
      }
      // std::cout << joint_name << " : " << dynamixel_pos << std::endl;
      /*if (i == 21) {
        ofs << i << "," << joint_name << "," << dynamixel_pos << "," << joint_pos << std::endl;
      }*/
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
  if (id == 1) {  // 2048->センター、0.045233->１回転で伸びるm
    return 2048 + rad / 0.045233 * 4096;
  } else if (id == 2) {  // 116->big_gearの歯の数, 22->mini_gearの歯の数
    return 2048 + rad / (M_PI * 2) * 4096 * (116.0 / 22.0);
  } else if (id == 3) {
    return 984 + rad / (M_PI * 2) * 4096 * (48.0 / 23.0);
  } else if (id == 4) {
    return 2560 + rad / (M_PI * 2) * 4096;
  } else if (id == 10) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 11) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 12) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 13) {
    return 1910 + rad / (M_PI * 2) * 4096;
  } else if (id == 20)
    return 2048 + rad / (M_PI * 2) * 4096;
  else if (id == 21)
    return 2048 + rad / (M_PI * 2) * 4096;
  else if (id == 22) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 23) {
    return 1900 + rad / (M_PI * 2) * 4096;
  } else
    return -1;
}

float SobitMiniDynamixel::toRad(std::string joint_name, int bit) {
  if (joint_name == "body_lift_joint") {
    return (bit - 2048) * (0.045233 / 4096);
  } else if (joint_name == "body_roll_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096) * (22.0 / 116.0);
  } else if (joint_name == "head_tilt_joint") {
    return (bit - 984) * ((M_PI * 2) / 4096) * (23.0 / 48.0);
  } else if (joint_name == "head_pan_joint") {
    return (bit - 2560) * ((M_PI * 2) / 4096);
  } else if (joint_name == "right_shoulder_roll_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "right_shoulder_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "right_wrist_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "right_hand_motor_joint") {
    return (bit - 1910) * ((M_PI * 2) / 4096);
  } else if (joint_name == "left_shoulder_roll_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "left_shoulder_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "left_wrist_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "left_hand_motor_joint") {
    return (bit - 1900) * ((M_PI * 2) / 4096);
  }
  return -1;
}