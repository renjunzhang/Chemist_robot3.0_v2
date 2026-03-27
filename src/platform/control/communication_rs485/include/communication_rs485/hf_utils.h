#ifndef HF_UTILS_H
#define HF_UTILS_H

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <unistd.h>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>

#include <serial/serial.h>

#include <communication_rs485/platformInfo.h>
#include <rs485_control.h>

#define BAUDRATE 115200
#define ROSLOOPRATE 40

extern const std::string portName = "/dev/HF";

// 以十六进制打印
void printHex(std::vector<uint8_t> &v);

// CRC 校验
uint16_t CRC16_MODBUS(std::vector<uint8_t> v, uint8_t len);

#endif