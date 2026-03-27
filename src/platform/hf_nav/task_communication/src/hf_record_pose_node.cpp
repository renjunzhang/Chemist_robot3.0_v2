/*
 * @Description:
 * @Author: speedzjy
 * @Date: 2022-08-02 10:51:17
 */
#include <cmath>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <map>
#include <std_msgs/Int32.h>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "tf/tf.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "platform_communication.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

DEFINE_string(stationJsonName, "","templet/station_english_new.json");
DEFINE_string(cameraJsonName, "", "templet/camera_pose.json");

using namespace std;

geometry_msgs::PoseWithCovarianceStamped cur_pose;

bool isFileExist(const std::string &file_path) {
  std::ifstream file(file_path.c_str());
  return file.good();
}

void poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos) {
  cur_pose = *pos;
}

void recordThread() {
  ros::NodeHandle n;

  setlocale(LC_ALL, "");

  while (n.ok()) {
    cout << "记录机器人位姿输入1， 记录相机位姿输入2: " << endl;
    int record;
    cin >> record;
    if (record == 1) {
      cout << "输入站点名称: " << endl;
      std::string station_name;
      cin >> station_name;

      Json::Value station_positon;
      station_positon["name"] = station_name;

      Json::Value position;
      position["x"] = cur_pose.pose.pose.position.x;
      position["y"] = cur_pose.pose.pose.position.y;
      position["z"] = cur_pose.pose.pose.orientation.z;
      position["w"] = cur_pose.pose.pose.orientation.w;

      station_positon["pose"] = position;

      // 写入文件
      std::string curPkgPath = ros::package::getPath("task_communication");
      // std::string fileName =
      //     curPkgPath + "/station_cfg/hefei_xuexiao/station_english_new.json";
      std::string fileName = FLAGS_stationJsonName;

      if (!isFileExist(fileName)) {

        ofstream ofs(fileName);
        if (ofs.is_open())
          cout << "Create " << fileName << endl;
      }

      cout << fileName << " already exists! Open..." << endl;

      Json::Reader reader;
      Json::Value root;

      // 读取文件信息到 root
      ifstream ifs(fileName);
      reader.parse(ifs, root);
      ifs.close();

      if (root.empty()) {
        cout << "root is empty, just append..." << endl;
        root.append(station_positon);
      } else {
        bool findFlag = false;
        for (int i = 0; i < root.size(); ++i) {
          if (root[i]["name"].asString() == station_name) {
            cout << "工作站已有数据, 修改位姿值..." << endl;
            root[i]["pose"] = position;
            findFlag = true;
            break;
          } // end if
        }   // end for

        if (!findFlag) {
          cout << "Not find station name, just append..." << endl;
          root.append(station_positon);
        }
      } // end if

      Json::StyledWriter writer;
      std::string strWrite = writer.write(root);

      ofstream ofs(fileName);
      ofs << strWrite;
      ofs.close();

      double yaw = tf::getYaw(cur_pose.pose.pose.orientation);
      printf("记录: \nname: %s\npose:\nx: %.3f\ny: %.3f\nyaw: %.3f\n",
             station_name.c_str(), cur_pose.pose.pose.position.x,
             cur_pose.pose.pose.position.y, yaw);
      cout << "记录完毕! 去下一个站点." << endl;
    } else if (2 == record) {
      cout << "输入相机站点名称: " << endl;
      std::string station_name;
      cin >> station_name;

      cout << "输入id: " << endl;
      int op_code_data;
      cin >> op_code_data;
      ros::Publisher get_camera_pose_publish =
          n.advertise<std_msgs::Int32>("/camera_operation", 1);
      std_msgs::Int32 op_code;
      op_code.data = op_code_data;
      while (get_camera_pose_publish.getNumSubscribers() == 0)
        ;
      get_camera_pose_publish.publish(op_code);
      geometry_msgs::PoseStampedConstPtr camera_pose =
          ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
              "/point_coordinate");

      tf2_ros::Buffer buffer;
      tf2_ros::TransformListener listener(buffer);

      geometry_msgs::PoseStamped marker_pose =
          buffer.transform(*camera_pose, "base_link", ros::Duration(1));

      Json::Value station_positon_camera;
      station_positon_camera["name"] = station_name;

      Json::Value position;
      position["x"] = marker_pose.pose.position.x;
      position["y"] = marker_pose.pose.position.y;
      position["z"] = marker_pose.pose.position.z;

      position["ox"] = marker_pose.pose.orientation.x;
      position["oy"] = marker_pose.pose.orientation.y;
      position["oz"] = marker_pose.pose.orientation.z;
      position["ow"] = marker_pose.pose.orientation.w;

      station_positon_camera["pose"] = position;

      // 写入文件
      std::string fileName = FLAGS_cameraJsonName;
      if (!isFileExist(fileName)) {
        ofstream ofs(fileName);
        if (ofs.is_open())
          cout << "Create "
               << "camera_pose.json" << endl;
      }

      cout << "camera_pose.json"
           << " already exists! Open..." << endl;

      Json::Reader reader;
      Json::Value root;

      // 读取文件信息到 root
      ifstream ifs(fileName);
      reader.parse(ifs, root);
      ifs.close();

      if (root.empty()) {
        cout << "root is empty, just append..." << endl;
        root.append(station_positon_camera);
      } else {
        bool findFlag = false;
        for (int i = 0; i < root.size(); ++i) {
          if (root[i]["name"].asString() == station_name) {
            cout << "工作站已有数据, 修改位姿值..." << endl;
            root[i]["pose"] = position;
            findFlag = true;
            break;
          } // end if
        }   // end for

        if (!findFlag) {
          cout << "Not find station name, just append..." << endl;
          root.append(station_positon_camera);
        }
      } // end if

      Json::StyledWriter writer;
      std::string strWrite = writer.write(root);

      ofstream ofs(fileName);
      ofs << strWrite;
      ofs.close();

      double yaw = tf::getYaw(cur_pose.pose.pose.orientation);
      printf("记录: \nname: %s\npose:\nx: %.3f\ny: %.3f\nyaw: %.3f\n",
             station_name.c_str(), cur_pose.pose.pose.position.x,
             cur_pose.pose.pose.position.y, yaw);
      cout << "记录完毕! 去下一个站点." << endl;
    }
  }
}

int main(int argc, char **argv) {

  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "hf_record_pose_node");

  ros::NodeHandle nh;

  std::unique_ptr<boost::thread> record_thread(
      new boost::thread(boost::bind(recordThread)));

  ros::Subscriber subCurrentPose =
      nh.subscribe("/amcl_pose_tf", 10, poseCallback);

  ros::spin();

  return 0;
}