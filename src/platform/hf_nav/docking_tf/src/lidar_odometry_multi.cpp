#include <cmath>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <unistd.h>

int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "hf_docking_tf");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  ros::Publisher hf_localization_pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose_tf",
                                                             20);
  std::string tf_prefix;
  nh_.param<std::string>("tf_prefix", tf_prefix, "hf_0");

  tf::TransformListener listener;
  ros::Rate r(20);
  while (nh.ok()) {
    tf::StampedTransform transform;

    try {
      /* code for Try */
      listener.waitForTransform("/map", "/" + tf_prefix + "/base_link",
                                ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/map", "/" + tf_prefix + "/base_link",
                               ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      /* code for Catch */
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::PoseWithCovarianceStamped hf_position;

    hf_position.header.stamp = ros::Time(0); // ros::Time::now()
    hf_position.header.frame_id = "map";

    // set the position
    hf_position.pose.pose.position.x = transform.getOrigin().x();
    hf_position.pose.pose.position.y = transform.getOrigin().y();
    hf_position.pose.pose.position.z = 0.00;

    hf_position.pose.pose.orientation.w = transform.getRotation().getW();
    hf_position.pose.pose.orientation.x = transform.getRotation().getX();
    hf_position.pose.pose.orientation.y = transform.getRotation().getY();
    hf_position.pose.pose.orientation.z = transform.getRotation().getZ();

    hf_localization_pub.publish(hf_position);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
