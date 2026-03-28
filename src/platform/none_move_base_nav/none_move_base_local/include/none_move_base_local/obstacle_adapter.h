#pragma once

#include <sensor_msgs/LaserScan.h>

namespace none_move_base_local {

class ObstacleAdapter {
public:
  ObstacleAdapter();

  void configure(double stop_distance, double sector_width);
  bool isBlocked(const sensor_msgs::LaserScan &scan, double heading) const;

private:
  double stop_distance_;
  double sector_width_;
};

} // namespace none_move_base_local
