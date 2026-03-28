#include "none_move_base_local/obstacle_adapter.h"

#include <cmath>

namespace none_move_base_local {

ObstacleAdapter::ObstacleAdapter() : stop_distance_(0.35), sector_width_(0.7) {}

void ObstacleAdapter::configure(double stop_distance, double sector_width) {
  stop_distance_ = stop_distance;
  sector_width_ = sector_width;
}

bool ObstacleAdapter::isBlocked(const sensor_msgs::LaserScan &scan,
                                double heading) const {
  const double half_sector = sector_width_ * 0.5;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range <= 0.0) {
      continue;
    }
    const double angle_error = std::atan2(std::sin(angle - heading), std::cos(angle - heading));
    if (std::fabs(angle_error) <= half_sector && range < stop_distance_) {
      return true;
    }
  }
  return false;
}

} // namespace none_move_base_local
