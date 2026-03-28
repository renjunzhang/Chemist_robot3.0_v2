#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace none_move_base_common {

double normalizeAngle(double angle);
double yawFromPose(const geometry_msgs::Pose &pose);
double distance2D(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
double distance2D(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b);
geometry_msgs::PoseStamped poseStampedFromCovariance(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
geometry_msgs::Point worldToBody(const geometry_msgs::Point &world_point,
                                 const geometry_msgs::Pose &robot_pose);

} // namespace none_move_base_common
