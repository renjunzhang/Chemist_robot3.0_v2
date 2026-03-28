#pragma once

#include <cstdint>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

namespace none_move_base_global {

class GridAdapter {
public:
  GridAdapter();

  void setMap(const nav_msgs::OccupancyGrid &map, int occupied_threshold,
              double inflation_radius, bool allow_unknown);

  bool hasMap() const;
  bool worldToGrid(double wx, double wy, int *gx, int *gy) const;
  geometry_msgs::Point gridToWorld(int gx, int gy) const;
  bool isCellFree(int gx, int gy) const;
  bool isSegmentFree(int x0, int y0, int x1, int y1) const;

  int width() const;
  int height() const;
  int index(int gx, int gy) const;
  bool inBounds(int gx, int gy) const;

private:
  void rebuildInflatedMap();
  bool isOccupiedValue(int8_t value) const;

  nav_msgs::OccupancyGrid map_;
  std::vector<uint8_t> inflated_occupied_;
  int occupied_threshold_;
  double inflation_radius_;
  bool allow_unknown_;
};

} // namespace none_move_base_global
