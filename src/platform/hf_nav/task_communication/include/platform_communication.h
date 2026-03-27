//
// Created by shao on 22-12-2.
//

#ifndef SRC_PLATFORM_COMMUNICATION_H
#define SRC_PLATFORM_COMMUNICATION_H
#include <atomic>
#include <cmath>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <map>
#include <string>
#include <thread>

#include "aichem_msg_srv/DmsService.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "tf/transform_datatypes.h" //转换函数头文件
#include <exception>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/String.h"

#include "actionlib/client/simple_action_client.h"
#include "communication_rs485/platformInfo.h"
#include "ros/package.h"
#include "ros/ros.h"

namespace platform {
namespace platform_communication {

using MoveBaseActionClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class PlatformCommunication {

public:
  PlatformCommunication();
  PlatformCommunication(const PlatformCommunication &) = delete;
  PlatformCommunication &operator=(const PlatformCommunication &) = delete;

private:
  std::map<std::string, std::vector<double>> station_pose_;
  Json::Value platform_status_;

  ros::NodeHandle nh_;
  // 订阅 task_manager 的目标站指令
  ros::Subscriber sub_dms_cmd_;
  // 订阅 amcl 的当前位姿
  ros::Subscriber sub_amcl_pose_;

  // 发送给 task_manager , 到达目标站的反馈
  ros::Publisher pub_dms_msg_;
  // 充电
  ros::Publisher pub_charge_;
  // 语音播报
  ros::Publisher pub_voide_broadcast_;

  MoveBaseActionClient ac_;
  move_base_msgs::MoveBaseGoal goal_;

  // 当前站点信息
  std::string cur_station_name_;

  // 当前位姿信息
  geometry_msgs::PoseWithCovarianceStamped curr_location_;

  ros::ServiceServer dms_inquiry_server_;

  std::map<std::string, std::vector<double>> refer_camera_pose_;

  // 充电标志
  std::atomic<bool> recharge_flag_;

  // 记录amcl_pose获取时间
  ros::Time amcl_pose_time_;

  // 二维码的id
  int aruco_id_;

private:
  bool DmsInquiryService(aichem_msg_srv::DmsService::Request &req,
                         aichem_msg_srv::DmsService::Response &resp);

  void DmsCallBack(const std_msgs::String::ConstPtr &obsNavigation_in);

  void AmclPoseCallBack(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos);

  // 被动查询线程

  // 朝地图原点方向走 distance (m), distance 范围 [0, 1]
  void StrollToOrigin(double const &distance);

  // 后退distance米
  void StrollToBack(double const &distance);

  // 原地旋转
  void RotateInPlace(double const &yaw);

  void StrollToGoal();
  void ReCharge(); // 充电操作函数

  void StrollToCharge(double x, double y, double z,
                      geometry_msgs::Quaternion q_rotate);
  void ReCharging(); // 持续充电
  bool GetCameraTran(geometry_msgs::PoseStamped &trans_map, const float &scale);
  void PubAmclInitialPosition();
};

} // namespace platform_communication
} // namespace platform

#endif // SRC_PLATFORM_COMMUNICATION_H
