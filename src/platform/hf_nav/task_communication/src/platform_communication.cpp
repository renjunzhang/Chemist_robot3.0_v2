#include "platform_communication.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(log_dir_path, "","log file path");
DEFINE_string(station_pose_path,"","station pose file path");
DEFINE_string(camera_pose_path,"","camera pose file path");
DEFINE_int32(aruco_id, 1,"aruco id ");


namespace {

// 读取站点文件
bool LoadStationPoseJson(
    std::map<std::string, std::vector<double>> &station_pose,
    const std::string &file_json_name ) {
  Json::Value value;
  Json::Reader reader;
  std::ifstream ifs(file_json_name);


  if (!reader.parse(ifs, value)) {
    ifs.close();
    ROS_ERROR("opening station config file failed");
    return false;
  }
  ifs.close();
  try {
    for (auto &it_value : value) {

      if (it_value["pose"].isMember("ox")) {
        station_pose.emplace(std::make_pair(
            it_value["name"].asString(),
            std::vector<double>{std::stod(it_value["pose"]["x"].asString()),
                                std::stod(it_value["pose"]["y"].asString()),
                                std::stod(it_value["pose"]["z"].asString()),
                                std::stod(it_value["pose"]["ox"].asString()),
                                std::stod(it_value["pose"]["oy"].asString()),
                                std::stod(it_value["pose"]["oz"].asString()),
                                std::stod(it_value["pose"]["ow"].asString())}));
      } else {
        station_pose.emplace(std::make_pair(
            it_value["name"].asString(),
            std::vector<double>{std::stod(it_value["pose"]["x"].asString()),
                                std::stod(it_value["pose"]["y"].asString()),
                                std::stod(it_value["pose"]["z"].asString()),
                                std::stod(it_value["pose"]["w"].asString())}));
      }
    }
  } catch (std::exception &e) {
    std::cout << e.what();
    return false;
  }

  return true;
}

geometry_msgs::PoseStamped
CoordinateTransform(Eigen::Isometry3d const &camera_pose,
                    Eigen::Isometry3d const &refer_camera_pose,
                    const float &scale) {
  

  Eigen::Isometry3d move_pose = camera_pose * refer_camera_pose.inverse();

  std::cout << "move_pose: " << move_pose.matrix() << std::endl;
  std::cout << "refer_camera_pose.inverse(): " << refer_camera_pose.inverse().matrix() << std::endl;

  geometry_msgs::PoseStamped pose_tranform;
  pose_tranform.header.frame_id = "hf_base_link";
  pose_tranform.header.stamp = ros::Time(0);
  // Converts an Eigen Isometry3d into a Pose message.
  // void tf::poseEigenToMsg (const Eigen::Isometry3d &e, geometry_msgs::Pose
  // &m)
  tf::poseEigenToMsg(move_pose, pose_tranform.pose);

  pose_tranform.pose.position.x = pose_tranform.pose.position.x * scale;
  pose_tranform.pose.position.y = pose_tranform.pose.position.y * scale;

  geometry_msgs::PoseStamped pose_tranform_map;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  bool transform_flage = false;
  do {
    try {
      pose_tranform_map =
          buffer.transform(pose_tranform, "map", ros::Duration(1));
      transform_flage = true;

    } catch (const std::exception &e) {
      // std::cerr << e.what() << '\n';
      ROS_INFO("wait tf buffer listener.....");
    }

  } while (!transform_flage);

  std::cout << "mappose_tranform.pose.position.x"
            << pose_tranform_map.pose.position.x << std::endl;
  std::cout << "mappose_tranform.pose.position.y"
            << pose_tranform_map.pose.position.y << std::endl;
  std::cout << "mappose_tranform.pose.position.z"
            << pose_tranform_map.pose.position.z << std::endl;

  return pose_tranform_map;
}

std::string GetCurrentTime() {
  std::time_t result = std::time(nullptr);

  std::string ret;
  ret.resize(64);
  int wsize = sprintf((char *)&ret[0], "%s", std::ctime(&result));
  ret.resize(wsize);
  return ret;
}
bool isFileExist(const std::string &file_path) {
  std::ifstream file(file_path.c_str());
  return file.good();
}
/*
 * vector_pose[0,1,2,3,4,5,6]
 * 对应位姿的   [x,y,z,ox,oy,oz,ow]
 */
Eigen::Isometry3d VectorToEigenPose(std::vector<double>vector_pose)
{
    Eigen::Isometry3d eigen_pose=Eigen::Isometry3d::Identity();

    //顺序w,x,y,z
    Eigen::Quaterniond Q (vector_pose[6],vector_pose[3],vector_pose[4],vector_pose[5]);
    Q.normalize();
    //x,y,z
    Eigen::Vector3d t(vector_pose[0],vector_pose[1],vector_pose[2]);

    //Isometry3d的旋转赋值方式
    eigen_pose.linear()=Q.toRotationMatrix();
    eigen_pose.pretranslate(t);
    return eigen_pose;

}

} // namespace

namespace platform {
namespace platform_communication {

PlatformCommunication::PlatformCommunication()
    : ac_("move_base", true), recharge_flag_(false),
      sub_dms_cmd_(nh_.subscribe("/obsNavigation_in", 10,
                                 &PlatformCommunication::DmsCallBack, this)),
      sub_amcl_pose_(nh_.subscribe(
          "/amcl_pose_tf", 10, &PlatformCommunication::AmclPoseCallBack, this)),
      pub_dms_msg_(nh_.advertise<std_msgs::String>("/obsNavigation_out", 10)),
      pub_voide_broadcast_(
          nh_.advertise<std_msgs::String>("/voide_broadcast_msg", 2)),
      dms_inquiry_server_(nh_.advertiseService(
          "/inquiryNavigation_out", &PlatformCommunication::DmsInquiryService,
          this)),
      amcl_pose_time_(0.0) {
  //        if(!nh_.getParam("odom_frame", odom_frame_))
  //            odom_frame_ = "odom";

  if (!LoadStationPoseJson(station_pose_,FLAGS_station_pose_path)) {
    exit(-1);
  }
  if (!LoadStationPoseJson(refer_camera_pose_, FLAGS_camera_pose_path)) {
    exit(-1);
  }

  // 初始化station_cfg_root_
  platform_status_["id"] = "";
  platform_status_["exper_no"] = "";
  platform_status_["stamp"] = GetCurrentTime();
  platform_status_["state"] = "idle";
  platform_status_["detail"] = "";
  platform_status_["cur_station"] = "charge_station";

  // 初始化机器人位姿
  // PubAmclInitialPosition();
}

void PlatformCommunication::PubAmclInitialPosition() {
  std::string file_path = ros::package::getPath("task_communication") +
                          "/station_cfg/" + "current_pose.json";
  Json::Value value;
  Json::Reader reader;
  std::ifstream ifs(file_path);

  if (!reader.parse(ifs, value)) {
    ifs.close();
    ROS_ERROR("opening station config file failed");
    exit(-1);
  }
  ifs.close();
  geometry_msgs::PoseWithCovarianceStamped init_position;
  init_position.header.frame_id = "map";

  init_position.pose.pose.position.x = std::stod(value["x"].asString());
  init_position.pose.pose.position.y = std::stod(value["y"].asString());
  init_position.pose.pose.position.z = std::stod(value["z"].asString());
  init_position.pose.pose.orientation.x = std::stod(value["ox"].asString());
  init_position.pose.pose.orientation.y = std::stod(value["oy"].asString());
  init_position.pose.pose.orientation.z = std::stod(value["oz"].asString());
  init_position.pose.pose.orientation.w = std::stod(value["ow"].asString());

  ros::Publisher amcl_pose_init_pub =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",
                                                              5);

  while (amcl_pose_init_pub.getNumSubscribers() == 0)
    ;
  ROS_INFO("发布初始位姿%d", amcl_pose_init_pub.getNumSubscribers());
  sleep(10);

  // amcl_pose_init_pub.publish(init_position);
  amcl_pose_init_pub.publish(init_position);
}

bool PlatformCommunication::DmsInquiryService(
        aichem_msg_srv::DmsService::Request &req,
        aichem_msg_srv::DmsService::Response &resp) {

  platform_status_["stamp"] = GetCurrentTime();
  // amcl_pose_mtx_.lock();
  auto time_diff = ros::Time::now() - amcl_pose_time_;
  // amcl_pose_mtx_.unlock();
  if (time_diff.sec > 3) {
    auto platform_status_copy = platform_status_;
    platform_status_copy["state"] = "error";
    resp.data = platform_status_copy.toStyledString();
    LOG(WARNING)<<"DmsInquiryService error";
    return true;
  }

  resp.data = platform_status_.toStyledString();

  return true;
}

void PlatformCommunication::StrollToOrigin(double const &distance) {
  move_base_msgs::MoveBaseGoal tmp_goal;

  double origin_x = -0.15;
  double feature_plat_x_length = 3;
  double feature_plat_y_length = 2.4;
  double origin_y = 2.72;

  tmp_goal.target_pose.header.frame_id = "map";
  tmp_goal.target_pose.header.stamp = ros::Time::now();
  tmp_goal.target_pose.pose.orientation = curr_location_.pose.pose.orientation;

  double tmp_x = curr_location_.pose.pose.position.x;
  double tmp_y = curr_location_.pose.pose.position.y;

  double dis_to_origin = (tmp_x - origin_x) * (tmp_x - origin_x) +
                         (tmp_y - origin_y) * (tmp_y - origin_y);
  dis_to_origin = sqrt(dis_to_origin);

  tmp_goal.target_pose.pose.position.x =
      (dis_to_origin < distance + feature_plat_x_length / 2 + 0.3)
          ? tmp_x
          : (tmp_x - origin_x) * (1 - (1 / dis_to_origin) * distance) +
                origin_x;
  tmp_goal.target_pose.pose.position.y =
      (dis_to_origin < distance + feature_plat_y_length / 2 + 0.3)
          ? tmp_y
          : (tmp_y - origin_y) * (1 - (1 / dis_to_origin) * distance) +
                origin_y;

  do {

    {
      std_msgs::String pub_voide_msg;
      pub_voide_msg.data = "机器人移动，注意避让";
      pub_voide_broadcast_.publish(pub_voide_msg);
      pub_voide_broadcast_.publish(pub_voide_msg);
    }
    ac_.sendGoal(tmp_goal);
    ac_.waitForResult();
    if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //usleep(5*1000);
    }
    ROS_INFO("Wait for stroll to origin");
  } while (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
}

void PlatformCommunication::StrollToBack(double const &distance) {
  // 向后平移0.5米解除充电
  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.orientation = curr_location_.pose.pose.orientation;
  geometry_msgs::PoseStamped pose_tranform;
  pose_tranform.header.frame_id = "hf_base_link";
  pose_tranform.header.stamp = ros::Time(0);
  pose_tranform.pose.position.x = 0;
  pose_tranform.pose.position.y = distance;
  pose_tranform.pose.position.z = 0;
  pose_tranform.pose.orientation.x = 0;
  pose_tranform.pose.orientation.y = 0;
  pose_tranform.pose.orientation.z = 0;
  pose_tranform.pose.orientation.w = 1;

  geometry_msgs::PoseStamped pose_tranform_map;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  bool transform_flage = false;
  do {
    try {
      pose_tranform_map =
          buffer.transform(pose_tranform, "map", ros::Duration(1));
      transform_flage = true;

    } catch (const std::exception &e) {
      // std::cerr << e.what() << '\n';
      ROS_INFO("wait tf buffer listener.....");
    }

  } while (!transform_flage);
  double map_x, map_y, map_z;
  map_x = pose_tranform_map.pose.position.x;
  map_y = pose_tranform_map.pose.position.y;
  map_z = 0;

  StrollToCharge(map_x, map_y, map_z, pose_tranform_map.pose.orientation);
}

void PlatformCommunication::StrollToGoal() {
  ROS_INFO("Moving...");

  // 先往地图原点走0.5m
  ROS_INFO("Moving 0.5m to origin");
  // StrollToOrigin(0.3);
  StrollToBack(0.3);
  ROS_INFO("\033[32mMoving 0.5m to origin Done\033[0m");

  // 原地旋转
  ROS_INFO("Inplace Rotating...");
  RotateInPlace(tf::getYaw(goal_.target_pose.pose.orientation));
  ROS_INFO("\033[32mInplace Rotating Done\033[0m");

  // 平动前往目标站点
  ROS_INFO("Now moving towards goal_...");

  double dis_to_goal = 0;

  do {

    {
      std_msgs::String pub_voide_msg;
      pub_voide_msg.data = "机器人移动，注意避让";
      pub_voide_broadcast_.publish(pub_voide_msg);
      pub_voide_broadcast_.publish(pub_voide_msg);
    }

    ac_.sendGoal(goal_);
    // 设置时限, 可以随时响应 goalCallback 对 this->goal_ 的更改
    //            ac_.waitForResult(ros::Duration(3, 0));
    ac_.waitForResult();

    dis_to_goal = pow(goal_.target_pose.pose.position.x -
                          curr_location_.pose.pose.position.x,
                      2) +
                  pow(goal_.target_pose.pose.position.y -
                          curr_location_.pose.pose.position.y,
                      2);
    dis_to_goal = sqrt(dis_to_goal);

    ROS_INFO("Go to: %s STATE: %s dis_to_goal: %.3f", cur_station_name_.c_str(),
             ac_.getState().toString().c_str(), dis_to_goal);

    if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      // 如果 move_base 规划失败, 比如说很多人围着
      // 为了人走开以后机器人也能动, 不断的发送目标点位置
      ROS_WARN("STATE: \033[1;31m Reach Goal ABORTED\033[0m");
      ROS_WARN("STATE: \033[1;32m Resend Goal\033[0m");
    }
      if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          //usleep(5*1000);
      }


  } while (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED ||
           dis_to_goal > 0.05);

  ROS_INFO("STATE: \033[1;32m Reach Goal SUCCEEDED\033[0m");
}

void PlatformCommunication::RotateInPlace(double const &yaw) {
  move_base_msgs::MoveBaseGoal tmp_goal;
  tmp_goal.target_pose.header.frame_id = "map";
  tmp_goal.target_pose.header.stamp = ros::Time::now();
  tmp_goal.target_pose.pose.position = curr_location_.pose.pose.position;
  tmp_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  auto curr_goal_orientate_difference =
      abs(tf::getYaw(curr_location_.pose.pose.orientation) -
          tf::getYaw(tmp_goal.target_pose.pose.orientation));

  if (curr_goal_orientate_difference < 5 * M_PI / 180 ||
      curr_goal_orientate_difference > 355 * M_PI / 180) {
    return;
  }

  do {
    {
      std_msgs::String pub_voide_msg;
      pub_voide_msg.data = "机器人移动，注意避让";
      pub_voide_broadcast_.publish(pub_voide_msg);
      pub_voide_broadcast_.publish(pub_voide_msg);
    }

    curr_goal_orientate_difference =
        abs(tf::getYaw(curr_location_.pose.pose.orientation) -
            tf::getYaw(tmp_goal.target_pose.pose.orientation));

    if (curr_goal_orientate_difference < 5 * M_PI / 180 ||
        curr_goal_orientate_difference > 355 * M_PI / 180) {
      return;
    }

    ac_.sendGoal(tmp_goal);
    ac_.waitForResult();
    if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //usleep(5*1000);
    }
    ROS_INFO("Wait for inplace-rotate");
  } while (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
}

void PlatformCommunication::DmsCallBack(
    const std_msgs::String::ConstPtr &obsNavigation_in) {

  Json::Reader jsonReader;
  Json::Value jsonValue;
  jsonReader.parse(obsNavigation_in->data, jsonValue);
  std::string action = jsonValue["action"].asString();
  std::string dest_station = jsonValue["destination"].asString();

  platform_status_["id"] = jsonValue["id"].asString();
  platform_status_["exper_no"] = jsonValue["exper_no"].asString();

  LoadStationPoseJson(station_pose_, FLAGS_station_pose_path);
  LoadStationPoseJson(refer_camera_pose_, FLAGS_camera_pose_path);

  if ("move" == action) {
    // 修改完文件不需要重新启动
    if (station_pose_.find(dest_station) == station_pose_.end()) {
      LoadStationPoseJson(station_pose_, FLAGS_station_pose_path);
      // 还是找不到就报错
      if (station_pose_.find(dest_station) == station_pose_.end()) {
        ROS_ERROR("%s dose not exist", dest_station.c_str());
        // 插入报错反馈话题
        platform_status_["state"] = "error";
        platform_status_["stamp"] = GetCurrentTime();
        platform_status_["detail"] = dest_station + "站点未标定";
        std_msgs::String pub_msg;
        pub_msg.data = platform_status_.toStyledString();
        sleep(2);
        pub_dms_msg_.publish(pub_msg);
        return;
      }
    }
    if (recharge_flag_) {
      platform_status_["state"] = "error";
      platform_status_["detail"] = "移动前解除充电";
      std_msgs::String pub_msg;
      pub_msg.data = platform_status_.toStyledString();
      sleep(2);
      pub_dms_msg_.publish(pub_msg);
      platform_status_["state"] = "charging";
      return;
    }

    // 发送 move_base 目标点
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();
    goal_.target_pose.pose.position.x = station_pose_[dest_station][0];
    goal_.target_pose.pose.position.y = station_pose_[dest_station][1];
    goal_.target_pose.pose.orientation.z = station_pose_[dest_station][2];
    goal_.target_pose.pose.orientation.w = station_pose_[dest_station][3];

    // 保存当前目标站点信息
    cur_station_name_ = dest_station;

    ROS_INFO("Set moveFlag_ \033[1;32mTrue\033[0m");

    platform_status_["state"] = "running";
    platform_status_["detail"] = "底盘前往" + dest_station;
    StrollToGoal();
    platform_status_["state"] = "done";
    platform_status_["detail"] = "底盘已运动到" + dest_station;
    platform_status_["cur_station"] = dest_station;

    // 主动发布话题
    std_msgs::String pub_msg;
    pub_msg.data = platform_status_.toStyledString();
    pub_dms_msg_.publish(pub_msg);
    platform_status_["state"] = "idle";

  }

  // 充电
  else if ("charge" == action) {
    // 如果已在充电中，直接发布消息返回
    if (recharge_flag_) {

      platform_status_["state"] = "done";
      platform_status_["detail"] = "已在充电中";
      std_msgs::String pub_msg;
      pub_msg.data = platform_status_.toStyledString();
      sleep(2);
      pub_dms_msg_.publish(pub_msg);
      platform_status_["state"] = "charging";
      return;
    }

    ReCharge();

  }
  // 解除充电
  else if ("uncharge" == action) {
    if (!recharge_flag_) {
      platform_status_["state"] = "done";
      platform_status_["detail"] = "已解除充电";
      std_msgs::String pub_msg;
      pub_msg.data = platform_status_.toStyledString();
      sleep(2);
      pub_dms_msg_.publish(pub_msg);
      platform_status_["state"] = "idle";
      return;
    }

    for (int tmp_i = 0; tmp_i < 2; tmp_i++) {
      // 向左平移0.5米解除充电
      platform_status_["state"] = "running";//更改状态
      geometry_msgs::PoseStamped current_pose;
      current_pose.pose.orientation = curr_location_.pose.pose.orientation;
      geometry_msgs::PoseStamped pose_tranform;
      pose_tranform.header.frame_id = "hf_base_link";
      pose_tranform.header.stamp = ros::Time(0);
      pose_tranform.pose.position.x = 0;
      pose_tranform.pose.position.y = 0.25;
      pose_tranform.pose.position.z = 0;
      pose_tranform.pose.orientation.x = 0;
      pose_tranform.pose.orientation.y = 0;
      pose_tranform.pose.orientation.z = 0;
      pose_tranform.pose.orientation.w = 1;

      geometry_msgs::PoseStamped pose_tranform_map;

      tf2_ros::Buffer buffer;
      tf2_ros::TransformListener listener(buffer);
      bool transform_flage = false;
      do {
        try {
          pose_tranform_map =
              buffer.transform(pose_tranform, "map", ros::Duration(1));
          transform_flage = true;

        } catch (const std::exception &e) {
          // std::cerr << e.what() << '\n';
          ROS_INFO("wait tf buffer listener.....");
        }

      } while (!transform_flage);
      double map_x, map_y, map_z;
      map_x = pose_tranform_map.pose.position.x;
      map_y = pose_tranform_map.pose.position.y;
      map_z = 0;

      StrollToCharge(map_x, map_y, map_z, pose_tranform_map.pose.orientation);
    }
    recharge_flag_ = false;
    platform_status_["state"] = "done";
    platform_status_["detail"] = "已解除充电";
    std_msgs::String pub_msg;
    pub_msg.data = platform_status_.toStyledString();
    pub_dms_msg_.publish(pub_msg);
    platform_status_["state"] = "idle";
  } else {

    platform_status_["state"] = "error";
    platform_status_["detail"] = "操作码异常";
    std_msgs::String pub_msg;
    pub_msg.data = platform_status_.toStyledString();
    sleep(2);
    pub_dms_msg_.publish(pub_msg);
    platform_status_["state"] = "idle";
    return;
  }
}

void PlatformCommunication::AmclPoseCallBack(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos) {
  // 实时储存位姿
  curr_location_ = *pos;
  // amcl_pose_mtx_.lock();
  amcl_pose_time_ = ros::Time::now();
  // amcl_pose_mtx_.unlock();
}

bool PlatformCommunication::GetCameraTran(geometry_msgs::PoseStamped &trans_map,
                                          const float &scale) {
  ros::Publisher get_camera_pose_publish =
      nh_.advertise<std_msgs::Int32>("/camera_operation", 1);
  std_msgs::Int32 op_code;
  op_code.data = FLAGS_aruco_id;
  geometry_msgs::PoseStampedConstPtr camera_pose;

  int camera_try_count = 0;
  do {

    while (get_camera_pose_publish.getNumSubscribers() == 0)
      ;
    get_camera_pose_publish.publish(op_code);
    camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "/point_coordinate");
    camera_try_count++;
    if (camera_try_count > 2) {
      platform_status_["state"] = "error";
      platform_status_["detail"] = std::string("充电未检测到二维码");
      std_msgs::String pub_msg;
      pub_msg.data = platform_status_.toStyledString();
      pub_dms_msg_.publish(pub_msg);
      platform_status_["state"] = "idle";
      return false;
    }
  } while (std::abs(camera_pose->pose.position.z) < 1e-5);


    Eigen::Isometry3d T_baselink_cameracoloframe=VectorToEigenPose(refer_camera_pose_["baselink_cameracoloframe"]);
    Eigen::Isometry3d refer_marker_pose_for_recharge=VectorToEigenPose(refer_camera_pose_["充电站相机"]);

    Eigen::Isometry3d Tw_camera_pose;
    tf::poseMsgToEigen(camera_pose->pose, Tw_camera_pose);
    Eigen::Isometry3d eigen_marker_pose=T_baselink_cameracoloframe*Tw_camera_pose;

//  tf2_ros::Buffer buffer;
//  tf2_ros::TransformListener listener(buffer);
//  geometry_msgs::PoseStamped marker_pose =
//      buffer.transform(*camera_pose, "base_link", ros::Duration(1));

//    Eigen::Isometry3d T_tf_marker_pose;
//    tf::poseMsgToEigen(marker_pose.pose, T_tf_marker_pose);
//    LOG(WARNING)<<"T_tf_marker_pose"<<T_tf_marker_pose.matrix();
//    LOG(WARNING)<<"eigen_marker_pose"<<eigen_marker_pose.matrix();

  std::cout << "T_baselink_cameracoloframe: " << T_baselink_cameracoloframe.matrix() << std::endl;
  std::cout << "Tw_camera_pose: " << Tw_camera_pose.matrix() << std::endl;
  std::cout << "eigen_marker_pos: " << eigen_marker_pose.matrix() << std::endl;

  trans_map =
      CoordinateTransform(eigen_marker_pose, refer_marker_pose_for_recharge, scale);

  return true;
}

void PlatformCommunication::ReCharge() {
  // 发送 move_base 目标点
  goal_.target_pose.header.frame_id = "map";
  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position.x = station_pose_["charge_station"][0];
  goal_.target_pose.pose.position.y = station_pose_["charge_station"][1];
  goal_.target_pose.pose.orientation.z = station_pose_["charge_station"][2];
  goal_.target_pose.pose.orientation.w = station_pose_["charge_station"][3];

  platform_status_["state"] = "running";
  platform_status_["detail"] = std::string("底盘前往") + "charging_station";
  StrollToGoal();

  geometry_msgs::PoseStamped trans_map;

  /*

  if (!GetCameraTran(trans_map, 1.0)) {
    return;
  }

  usleep(0.5 * 1000);
  double map_x, map_y, map_z;
  map_x = curr_location_.pose.pose.position.x;
  map_y = curr_location_.pose.pose.position.y;
  map_z = 0;
  // 2 x方向平移
  std::cout << "xxxxxx" << std::endl;
  map_x = trans_map.pose.position.x;
  trans_map.pose.orientation = curr_location_.pose.pose.orientation;
  StrollToCharge(map_x, map_y, map_z, trans_map.pose.orientation);
  usleep(0.5 * 1000);

  // x方向再移动一次
  if (!GetCameraTran(trans_map, 1.0)) {
    return;
  }
  map_x = curr_location_.pose.pose.position.x;
  map_y = curr_location_.pose.pose.position.y;
  map_z = 0;
  // x方向平移
  std::cout << "xxxxxx" << std::endl;
  map_x = trans_map.pose.position.x;
  trans_map.pose.orientation = curr_location_.pose.pose.orientation;
  StrollToCharge(map_x, map_y, map_z, trans_map.pose.orientation);
  usleep(0.5 * 1000);
   */
    double map_x,map_y,map_z=0;
  // 平移
  for (int i = 1; i < 6; i++) {
    if (!GetCameraTran(trans_map, float(i) / 5)) {
      return;
    }
    map_x = trans_map.pose.position.x;
    map_y = trans_map.pose.position.y;
    StrollToCharge(map_x, map_y, map_z, trans_map.pose.orientation);
    usleep(0.3 * 1000);
  }

  recharge_flag_ = true;

  // 主线程结束，持续发命令线程detach
  auto charging_thread =
      std::thread(std::bind(&PlatformCommunication::ReCharging, this));
  charging_thread.detach();
  sleep(1);

  platform_status_["state"] = "done";
  platform_status_["detail"] = std::string("底盘已运动到") + "充电桩";
  platform_status_["cur_station"] = "charge_station";
  std_msgs::String pub_msg;
  pub_msg.data = platform_status_.toStyledString();
  pub_dms_msg_.publish(pub_msg);
  platform_status_["state"] = "charging";
}

void PlatformCommunication::StrollToCharge(double x, double y, double z,
                                           geometry_msgs::Quaternion q_rotate) {

  move_base_msgs::MoveBaseGoal tmp_goal;
  tmp_goal.target_pose.header.frame_id = "map";
  tmp_goal.target_pose.header.stamp = ros::Time::now();
  tmp_goal.target_pose.pose.position.x = x;
  tmp_goal.target_pose.pose.position.y = y;
  tmp_goal.target_pose.pose.position.z = z;
  tmp_goal.target_pose.pose.orientation = q_rotate;
  do {
    ac_.sendGoal(tmp_goal);
    // 设置时限, 可以随时响应 goalCallback 对 this->goal_ 的更改
    //            ac_.waitForResult(ros::Duration(3, 0));
    ac_.waitForResult();

    ROS_INFO("Go to: %s STATE: %s", cur_station_name_.c_str(),
             ac_.getState().toString().c_str());

    if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      // 如果 move_base 规划失败, 比如说很多人围着
      // 为了人走开以后机器人也能动, 不断的发送目标点位置
      ROS_WARN("STATE: \033[1;31m Reach Goal ABORTED\033[0m");
      ROS_WARN("STATE: \033[1;32m Resend Goal\033[0m");
    }
    if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //usleep(5*1000);
    }

  } while (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
}

void PlatformCommunication::ReCharging() {
  ROS_INFO("\033[1;32m start charge...\033[0m");
  geometry_msgs::Twist v;

  ros::Publisher pubCrgMsg =
      nh_.advertise<geometry_msgs::Twist>("/hf_platform/crg_vel", 10);
  //            communication_rs485::platformInfo platform_info;
  sleep(5);
  bool is_full = false;
  int old_quantity = 0, new_quantity;
  bool charge_success = true;
  while (recharge_flag_) {

    auto platform_info =
        ros::topic::waitForMessage<communication_rs485::platformInfo>(
            "/hf_platform/platform_info");
    new_quantity = static_cast<int>(platform_info->batteryPower);
    ROS_INFO("quantity of electricity is %.3f", platform_info->batteryPower);
    if (platform_info->batteryPower >= 100) {

      v.angular.x = 0;
      is_full = true;
      ROS_INFO("\033[1;32m 电池电量大于99，设置状态为充满，停止充电\033[0m");
    }

    if (!is_full) {
      v.angular.x = 1;
      ROS_INFO("\033[1;32m 状态为未充满，继续充电\033[0m");
      charge_success = new_quantity - old_quantity >= 0;
      old_quantity = new_quantity;

    } else if (platform_info->batteryPower <= 90) {
      v.angular.x = 1;
      is_full = false;
      ROS_INFO("\033[1;32m 电池电量低于90，设置状态为未充满，继续充电\033[0m");
      old_quantity = new_quantity;

    } else {
      v.angular.x = 0;
      ROS_INFO("\033[1;32m 电池电量大于90，状态充满，停止充电\033[0m");
    }
    pubCrgMsg.publish(v);
    sleep(2);
     if (!charge_success)
     {
//       StrollToBack(0.02);
//       geometry_msgs::PoseStamped tmp_trans_map;
//       if (!GetCameraTran(tmp_trans_map, 1.0)) {
//         return;
//       }
//       double map_x = tmp_trans_map.pose.position.x;
//       double map_y = tmp_trans_map.pose.position.y;
//       double map_z = 0.0;
//       StrollToCharge(map_x, map_y, map_z, tmp_trans_map.pose.orientation);
        LOG(WARNING)<<"charge error"<<"old_quantity is"<<old_quantity<<"new_quantity is"<<new_quantity;
        v.angular.x = 0;
        pubCrgMsg.publish(v);
         sleep(5);
         v.angular.x = 1;
         pubCrgMsg.publish(v);
       charge_success = true;
     }
  }
  ROS_INFO("\033[1;32ms end charge...\033[0m");
}

} // namespace platform_communication
} // namespace platform

/**
 * note: gflags是一套命令行参数解析工具
 * DEFINE_bool在gflags.h中定义
 * gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等
 * 定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
 * 当参数被定义后, 通过FLAGS_name就可访问到对应的参数
 */
// collect_metrics ：激活运行时度量的集合.如果激活, 可以通过ROS服务访问度量


int main(int argc, char **argv)

{
    google::ParseCommandLineFlags(&argc, &argv, true);
  setlocale(LC_ALL, "");
  google::InitGoogleLogging("charge_glog");
  FLAGS_colorlogtostderr = true;
  FLAGS_log_dir = FLAGS_log_dir_path;
  FLAGS_alsologtostderr = true;
  ros::init(argc, argv, "platform_communication_node");
  ::platform::platform_communication::PlatformCommunication
      platform_communication;
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  return 0;
}