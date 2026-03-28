#include "none_move_base_global/path_postprocessor.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include "none_move_base_common/pose_utils.h"
#include "none_move_base_global/grid_adapter.h"

namespace none_move_base_global {

namespace {

bool isCollinear(const GridCell &a, const GridCell &b, const GridCell &c) {
  const int abx = b.x - a.x;
  const int aby = b.y - a.y;
  const int bcx = c.x - b.x;
  const int bcy = c.y - b.y;
  return abx * bcy - aby * bcx == 0;
}

bool hasOrientation(const geometry_msgs::PoseStamped &pose) {
  const auto &q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return norm > 1e-6;
}

geometry_msgs::Quaternion createQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

geometry_msgs::Point interpolate(const geometry_msgs::Point &a,
                                 const geometry_msgs::Point &b, double ratio) {
  geometry_msgs::Point point;
  point.x = a.x + (b.x - a.x) * ratio;
  point.y = a.y + (b.y - a.y) * ratio;
  point.z = a.z + (b.z - a.z) * ratio;
  return point;
}

} // namespace

std::vector<GridCell> PathPostprocessor::removeCollinear(
    const std::vector<GridCell> &raw_path) const {
  if (raw_path.size() < 3) {
    return raw_path;
  }

  std::vector<GridCell> output;
  output.reserve(raw_path.size());
  output.push_back(raw_path.front());

  for (std::size_t i = 1; i + 1 < raw_path.size(); ++i) {
    if (!isCollinear(raw_path[i - 1], raw_path[i], raw_path[i + 1])) {
      output.push_back(raw_path[i]);
    }
  }

  output.push_back(raw_path.back());
  return output;
}

std::vector<GridCell> PathPostprocessor::pruneLineOfSight(
    const std::vector<GridCell> &raw_path, const GridAdapter &grid) const {
  if (raw_path.size() < 3) {
    return raw_path;
  }

  std::vector<GridCell> pruned;
  pruned.reserve(raw_path.size());
  std::size_t anchor = 0;
  pruned.push_back(raw_path.front());

  while (anchor < raw_path.size() - 1) {
    std::size_t furthest = raw_path.size() - 1;
    while (furthest > anchor + 1 &&
           !grid.isSegmentFree(raw_path[anchor].x, raw_path[anchor].y,
                               raw_path[furthest].x, raw_path[furthest].y)) {
      --furthest;
    }
    pruned.push_back(raw_path[furthest]);
    anchor = furthest;
  }

  return pruned;
}

nav_msgs::Path PathPostprocessor::buildPath(
    const std::vector<GridCell> &cell_path, const GridAdapter &grid,
    const geometry_msgs::PoseStamped &start_pose,
    const geometry_msgs::PoseStamped &goal_pose, const std::string &frame_id,
    const ros::Time &stamp, double resample_step) const {
  nav_msgs::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;

  if (cell_path.empty()) {
    return path;
  }

  std::vector<geometry_msgs::Point> waypoints;
  waypoints.reserve(cell_path.size() + 2);
  waypoints.push_back(start_pose.pose.position);
  for (std::size_t i = 1; i + 1 < cell_path.size(); ++i) {
    waypoints.push_back(grid.gridToWorld(cell_path[i].x, cell_path[i].y));
  }
  waypoints.push_back(goal_pose.pose.position);

  std::vector<double> cumulative(waypoints.size(), 0.0);
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    cumulative[i] =
        cumulative[i - 1] +
        none_move_base_common::distance2D(waypoints[i - 1], waypoints[i]);
  }

  const double total_length = cumulative.back();
  std::vector<geometry_msgs::Point> sampled;
  if (total_length <= 1e-6 || resample_step <= 1e-6) {
    sampled = waypoints;
  } else {
    for (double sample = 0.0; sample < total_length; sample += resample_step) {
      auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), sample);
      if (upper == cumulative.end()) {
        break;
      }

      const std::size_t upper_index =
          static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
      if (upper_index == 0) {
        sampled.push_back(waypoints.front());
        continue;
      }

      const std::size_t lower_index = upper_index - 1;
      const double segment_length = cumulative[upper_index] - cumulative[lower_index];
      const double ratio = segment_length <= 1e-6
                               ? 0.0
                               : (sample - cumulative[lower_index]) / segment_length;
      sampled.push_back(
          interpolate(waypoints[lower_index], waypoints[upper_index], ratio));
    }
    sampled.push_back(waypoints.back());
  }

  for (std::size_t i = 0; i < sampled.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = sampled[i];

    double yaw = 0.0;
    if (i + 1 < sampled.size()) {
      yaw = std::atan2(sampled[i + 1].y - sampled[i].y,
                       sampled[i + 1].x - sampled[i].x);
      pose.pose.orientation = createQuaternion(yaw);
    } else if (hasOrientation(goal_pose)) {
      pose.pose.orientation = goal_pose.pose.orientation;
    } else if (!path.poses.empty()) {
      pose.pose.orientation = path.poses.back().pose.orientation;
    } else {
      pose.pose.orientation =
          createQuaternion(none_move_base_common::yawFromPose(start_pose.pose));
    }

    path.poses.push_back(pose);
  }

  return path;
}

} // namespace none_move_base_global
