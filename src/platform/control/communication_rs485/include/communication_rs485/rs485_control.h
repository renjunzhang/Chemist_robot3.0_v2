#ifndef RS485_CONTROL_H
#define RS485_CONTROL_H

#include <communication_rs485/platformInfo.h>
#include <hf_utils.h>

class RS485Control {
private:
  ros::Subscriber subVelocity;
  serial::Serial ros_serial;
  ros::Publisher pubPlatformInfo;

  std::vector<uint8_t> readOdomPowerCmd; // 读电池、速度、里程计信息的 Modbus 命令
  std::vector<uint8_t> writeVelCrgCmd; // 写速度信息的 Modbus 命令
  std::vector<uint8_t> readReturn;

  std::string port_name;
  uint32_t baud_rate;

  bool continueCharge;
  int16_t oldPower, newPower;

public:
  RS485Control(ros::NodeHandle &nh, const std::string portName,
               uint32_t baudrate);

  ~RS485Control();

  bool serialInit();

  void RS485CmdInit();

  void velocityCallback(const geometry_msgs::Twist &twist);

  void receiveFromRS485(std::vector<uint8_t> &readReturn);

  void readOdometryPowerCmd();

  void writeVelocityChargeCmd();

  // 解算成十进制速度,拼凑 uint8_t
  void readVelOdomInfo(const std::vector<uint8_t> &v,
                     communication_rs485::platformInfo &platform_info);

  // 打印写入的速度信息
  void writeVelInfo(const std::vector<uint8_t> &v);

  void runOnce(ros::Rate &loop_rate);
};

#endif