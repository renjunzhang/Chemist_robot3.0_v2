#include <hf_utils.h>
#include <rs485_control.h>

RS485Control::RS485Control(ros::NodeHandle &nh, const std::string portName,
                           uint32_t baudrate)
    : readOdomPowerCmd(std::vector<uint8_t>(8)),
      writeVelCrgCmd(std::vector<uint8_t>(29)), oldPower(0), newPower(0),
      continueCharge(0) {

  port_name = portName;
  baud_rate = baudrate;

  subVelocity = nh.subscribe("twist_mux/cmd_vel", 1000,
                             &RS485Control::velocityCallback, this);
  pubPlatformInfo =
      nh.advertise<communication_rs485::platformInfo>("platform_info", 1000);

  RS485CmdInit();
}

RS485Control::~RS485Control() {}

bool RS485Control::serialInit() {
  ros_serial.setPort(port_name);
  ros_serial.setBaudrate(baud_rate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  ros_serial.setTimeout(timeout);
  ros_serial.setBytesize(serial::eightbits);
  ros_serial.setParity(serial::parity_even);
  ros_serial.setStopbits(serial::stopbits_one);

  try {
    ros_serial.open();
    if (ros_serial.isOpen()) {
      ROS_INFO("RS485 Serial Port opened...\n");
    } else {
      return 0;
    }
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable timeout open port...\n");
    return 0;
  }

  return 1;
}

void RS485Control::RS485CmdInit() {
  // 读取里程计信息
  //   0A     04      0005       0018      A17E
  // 从机号 读功能码 基地址5  读24个寄存器   CRC校验
  readOdomPowerCmd[0] = 0x0A;
  readOdomPowerCmd[1] = 0x04;
  readOdomPowerCmd[2] = 0x00;
  readOdomPowerCmd[3] = 0X05;
  readOdomPowerCmd[4] = 0X00;
  readOdomPowerCmd[5] = 0x18;
  readOdomPowerCmd[6] = 0x00;
  readOdomPowerCmd[7] = 0x00;

  // 写速度信息 命令共29字节
  //   0A     10      0078      000A        14      0002          0000 0000
  //   0000   （0000 0000 0000 0000 0000 5个无效字节位）  0000(充电Bit0置1)
  //   xxxx
  // 从机号 写功能码 基地址120  写10个寄存器 共20字节 全向自由模式
  // y(注意！y在前) x    z   5个无效字节位   充电Bit0置1   CRC校验
  writeVelCrgCmd[0] = 0x0A;
  writeVelCrgCmd[1] = 0x10;
  writeVelCrgCmd[2] = 0x00;
  writeVelCrgCmd[3] = 0x78;
  writeVelCrgCmd[4] = 0x00;
  writeVelCrgCmd[5] = 0x0A;
  writeVelCrgCmd[6] = 0x14;
  writeVelCrgCmd[7] = 0x00;
  writeVelCrgCmd[8] = 0x02;
  for (int i = 9; i < writeVelCrgCmd.size(); ++i) {
    writeVelCrgCmd[i] = 0x00;
  }
  // 节点每次启动时清除里程计
  writeVelCrgCmd[26] |= 0b0100; // 清除自带里程计位 置1
}

void RS485Control::velocityCallback(const geometry_msgs::Twist &twist) {

  if (twist.angular.x == 1) {     // 充电标志位
    writeVelCrgCmd[8] = 0x01;     // 静止模式
    writeVelCrgCmd[26] |= 0b0001; // 充电位 置1
    ROS_INFO("Charging......");
  } else {
    writeVelCrgCmd[8] = 0x02;     // 全向自由模式
    writeVelCrgCmd[26] &= 0b1110; // 充电位 置0
  }

  if (twist.angular.y == 1) {
    writeVelCrgCmd[26] |= 0b0100; // 清除自带里程计位 置1
  } else {
    writeVelCrgCmd[26] &= 0b1011; // 清除自带里程计位 置0
  }

  // ROS 下机器人坐标系x 向前，坐标系需要转换一下
  // 行发平台 x 方向速度，正值向右; 在ROS里是 y 轴,正值在左
  writeVelCrgCmd[11] = (uint8_t)((int)(-10000 * twist.linear.y) >> 8);
  writeVelCrgCmd[12] = (uint8_t)((int)(-10000 * twist.linear.y));
  // ROS x 方向速度，正值向前
  writeVelCrgCmd[9] = (uint8_t)((int)(10000 * twist.linear.x) >> 8);
  writeVelCrgCmd[10] = (uint8_t)((int)(10000 * twist.linear.x));
  // z 方向速度，正值为逆时针, 弧度换算成角度
  writeVelCrgCmd[13] =
      (uint8_t)((int)(100 * twist.angular.z * 180 / M_PI) >> 8);
  writeVelCrgCmd[14] = (uint8_t)((int)(100 * twist.angular.z * 180 / M_PI));
}

void RS485Control::receiveFromRS485(std::vector<uint8_t> &readReturn) {
  if (ros_serial.available()) {
    ros_serial.read(readReturn, ros_serial.available());
  }
  ros_serial.flush();
}

void RS485Control::readOdometryPowerCmd() {
  ros_serial.flush();

  uint16_t r = CRC16_MODBUS(readOdomPowerCmd, 6);
  readOdomPowerCmd[6] = (uint8_t)(r >> 8);
  readOdomPowerCmd[7] = (uint8_t)(r);

  ros_serial.write(readOdomPowerCmd);
  ROS_DEBUG("Send readOdom: ");
  // printHex(readOdomPowerCmd);
  ROS_DEBUG("\n");
}

void RS485Control::writeVelocityChargeCmd() {
  ros_serial.flush();

  uint16_t r = CRC16_MODBUS(writeVelCrgCmd, 27);
  writeVelCrgCmd[27] = (uint8_t)(r >> 8);
  writeVelCrgCmd[28] = (uint8_t)(r);

  ros_serial.write(writeVelCrgCmd);
  ROS_DEBUG("Send writeVelocity: ");
  // printHex(writeVelCrgCmd);
  ROS_DEBUG("\n");
}

void RS485Control::readVelOdomInfo(
    const std::vector<uint8_t> &v,
    communication_rs485::platformInfo &platform_info) {
  // 电池信息
  int16_t batteryVoltage = ((int16_t)v[11] << 8) | (int16_t)v[12];
  int16_t batteryPower = ((int16_t)v[13] << 8) | (int16_t)v[14];

  oldPower = newPower;
  newPower = batteryPower;

  // 返回的速度信息Modbus串有固定格式，具体参见航发手册
  std::vector<int16_t> vel(3); // [x y z(theta)]
  std::vector<int32_t> odom(3);

  // ROS 机器人坐标 x 指向前，y轴指向左
  vel[0] = (((int16_t)v[35] << 8) | (int16_t)v[36]);
  vel[1] = ((int16_t)v[33] << 8) | (int16_t)v[34];
  vel[1] *= -1;
  vel[2] = ((int16_t)v[37] << 8) | (int16_t)v[38];
  odom[0] = ((int32_t)v[45] << 24) | ((int32_t)v[46] << 16) |
            ((int32_t)v[43] << 8) | (int32_t)v[44];
  odom[1] = ((int32_t)v[41] << 24) | ((int32_t)v[42] << 16) |
            ((int32_t)v[39] << 8) | (int32_t)v[40];
  odom[1] *= -1;
  odom[2] = ((int32_t)v[49] << 24) | ((int32_t)v[50] << 16) |
            ((int32_t)v[47] << 8) | (int32_t)v[48];

  platform_info.batteryVoltage = batteryVoltage / 10.0f;
  platform_info.batteryPower = batteryPower / 1.0f;

  // 保留一位小数
  platform_info.batteryVoltage =
      floor(platform_info.batteryVoltage * 10000.000f + 0.5) / 10000.000f;
  platform_info.batteryPower =
      floor(platform_info.batteryPower * 10000.000f + 0.5) / 10000.000f;

  for (int i = 0; i < 3; ++i) {
    platform_info.vel.push_back((float)vel[i]);
    platform_info.odom.push_back((float)odom[i]);
  }
}

void RS485Control::writeVelInfo(const std::vector<uint8_t> &v) {
  // 写入的速度信息Modbus串有固定格式（y在前），具体参见航发手册
  int16_t vel_x = ((int16_t)v[11] << 8) | (int16_t)v[12];
  int16_t vel_y = ((int16_t)v[9] << 8) | (int16_t)v[10];
  int16_t vel_z = ((int16_t)v[13] << 8) | (int16_t)v[14];
}

void RS485Control::runOnce(ros::Rate &loop_rate) {

  communication_rs485::platformInfo platform_info;

  // 写入速度
  while (!ros_serial.available()) {
    int count = 0;
    ROS_DEBUG("Try to send WRIRE Vel %d times...", ++count);
    writeVelocityChargeCmd();
    loop_rate.sleep();
  }
  receiveFromRS485(readReturn);
  readReturn.clear();
  // 读取速度、里程计、电池信息
  ROS_DEBUG("READ TIME:");
  while (!ros_serial.available()) {
    int count = 0;
    ROS_DEBUG("Try to send READ Vel %d times...", ++count);
    readOdometryPowerCmd();
    loop_rate.sleep();
  }
  receiveFromRS485(readReturn);
  // 如果有脏数据,丢弃
  if (readReturn[0] != 0x0A ||
      (readReturn[0] == 0x0A && readReturn[1] != 0x04) ||
      readReturn.size() != 53) { // 53 = 48 + 5, 参考 modbus 指令返回值
                                 // 48 是返回的数据大小, 头部尾部加起来 =5
    readReturn.clear();
    loop_rate.sleep();
    return;
  }

  ROS_INFO("\033[33m Odom Receive From RS485: This is Good data! \033[0m");
  printHex(readReturn);
  printf("\n");
  readVelOdomInfo(readReturn, platform_info);

  readReturn.clear();

  if (newPower - oldPower == 1)
    continueCharge = true;
  if (newPower - oldPower == -1)
    continueCharge = false;

  // std::cout << newPower << " " << oldPower << std::endl;

  platform_info.continueCharge = continueCharge;
  platform_info.chargeFlag =
      (writeVelCrgCmd[26] & 0b0001 == 0b0001) ? true : false;

  pubPlatformInfo.publish(platform_info);
}