#include "none_move_base_common/pose_utils.h"

#include <cmath>

#include <tf2/utils.h>

namespace none_move_base_common {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromPose(const geometry_msgs::Pose &pose) {
  return tf2::getYaw(pose.orientation);
}

double distance2D(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double distance2D(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b) {
  return distance2D(a.position, b.position);
}

geometry_msgs::PoseStamped poseStampedFromCovariance(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  geometry_msgs::PoseStamped output;
  output.header = msg.header;
  output.pose = msg.pose.pose;
  return output;
}

geometry_msgs::Point worldToBody(const geometry_msgs::Point &world_point,
                                 const geometry_msgs::Pose &robot_pose) {
  geometry_msgs::Point local;
  const double dx = world_point.x - robot_pose.position.x;
  const double dy = world_point.y - robot_pose.position.y;
  const double yaw = yawFromPose(robot_pose);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  local.x = c * dx + s * dy;
  local.y = -s * dx + c * dy;
  local.z = world_point.z - robot_pose.position.z;
  return local;
}

} // namespace none_move_base_common
