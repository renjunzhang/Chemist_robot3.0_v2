#include <hf_utils.h>

#define MONITOR_LOOPRATE 200

using namespace std;

class PlatformMonitor {
private:
  ros::Subscriber subPlatformInfo;
  ros::Publisher pubPlatformOdom;
  ros::Publisher pubPlatformTrans;
  tf::TransformBroadcaster odom_broadcaster;

  nav_msgs::Odometry last_odom;

  std::string tf_prefix_;

public:
  PlatformMonitor(ros::NodeHandle &nh) {
    subPlatformInfo = nh.subscribe(
        "platform_info", 1000, &PlatformMonitor::platformInfoCallback, this);

    pubPlatformOdom = nh.advertise<nav_msgs::Odometry>("odom", 100);
    pubPlatformTrans =
        nh.advertise<geometry_msgs::TransformStamped>("platform_trans", 100);
    
    ros::NodeHandle nh_("~");
    nh_.param<std::string>("tf_prefix", tf_prefix_, "hf_0");
  }

  ~PlatformMonitor() {}

  void
  platformInfoCallback(const communication_rs485::platformInfo &platform_info) {
    // 以下三个变量后续可能会删除
    // 因为已经在串口节点里对单片机返回的数据进行过滤
    double jump_threshold_x = 1;
    double jump_threshold_y = 1;
    double jump_threshold_z = 1;

    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped trans;

    auto cur_time = ros::Time::now();

    // 里程计转换成 m 单位并保留小数点后四位
    double tmp_odom_x = floor(platform_info.odom[0] + 0.5) / 10000.000f;
    double tmp_odom_y = floor(platform_info.odom[1] + 0.5) / 10000.000f;

    // ------------------tf 变换---------------------
    // ros 机器人坐标系 x前方， y左方， z上方
    trans.header.stamp = cur_time;
    trans.header.frame_id = tf_prefix_ + "/odom";
    trans.child_frame_id = tf_prefix_ + "/base_link";
    // 以米为单位
    trans.transform.translation.x = tmp_odom_x;
    trans.transform.translation.y = tmp_odom_y;

    auto q = tf::createQuaternionMsgFromRollPitchYaw(
        0, 0, platform_info.odom[2] * M_PI / 180.0 / 100.0);
    trans.transform.rotation.w = q.w;
    trans.transform.rotation.x = q.x;
    trans.transform.rotation.y = q.y;
    trans.transform.rotation.z = q.z;

    // ------------------里程计---------------------
    odom.header.stamp = cur_time;
    odom.header.frame_id = tf_prefix_ + "/odom";
    odom.child_frame_id = tf_prefix_ + "/base_link";
    odom.pose.pose.position.x = tmp_odom_x;
    odom.pose.pose.position.y = tmp_odom_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = q;
    odom.twist.twist.linear.x = platform_info.vel[0] / 10000;
    odom.twist.twist.linear.y = platform_info.vel[1] / 10000;
    // 度数转弧度
    odom.twist.twist.angular.z = platform_info.vel[2] * M_PI / 180 / 100;

    if (odom.pose.pose.position.x == 0 &&
        odom.pose.pose.position.y == 0) { // 里程计置零情况
      ROS_DEBUG_STREAM("\033[1;33m Platfrom Odom Set ZERO \033[0m");

      // 发布 tf
      pubPlatformTrans.publish(trans);
      odom_broadcaster.sendTransform(trans);
      // 发布里程计
      pubPlatformOdom.publish(odom);

      last_odom = odom;
    } else if ((abs(odom.pose.pose.position.x -
                    last_odom.pose.pose.position.x) < jump_threshold_x) &&
               (abs(odom.pose.pose.position.y -
                    last_odom.pose.pose.position.y) <
                jump_threshold_y)) { // 正常里程计情况
      ROS_DEBUG_STREAM("\033[1;32m Platfrom Odom Normal Publish \033[0m");

      // 发布 tf
      pubPlatformTrans.publish(trans);
      odom_broadcaster.sendTransform(trans);
      // 发布里程计
      pubPlatformOdom.publish(odom);

      last_odom = odom;
    } else { // 这个情况后续可能会删除
      // 因为已经在串口节点里对单片机返回的数据进行过滤
      ROS_WARN_STREAM("\033[1;31m Platfrom Odom Jump ERROR!!\033[0m");
      ROS_WARN_STREAM("\033[1;31m Dumped Odom Publish!!\033[0m");
      ROS_WARN("\033[1;31m ERROR pose : [%.3f %.3f %.3f]\033[0m",
               odom.pose.pose.position.x, odom.pose.pose.position.y,
               odom.pose.pose.position.z);
    }
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "plarform_monitor");

  ros::NodeHandle nh;
  ros::Rate loop_rate(MONITOR_LOOPRATE);

  PlatformMonitor PM(nh);

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
