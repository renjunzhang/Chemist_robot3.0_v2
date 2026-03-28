#include "none_move_base_global/grid_adapter.h"

#include <algorithm>
#include <cmath>

namespace none_move_base_global {

GridAdapter::GridAdapter()
    : occupied_threshold_(50), inflation_radius_(0.0), allow_unknown_(false) {}

void GridAdapter::setMap(const nav_msgs::OccupancyGrid &map,
                         int occupied_threshold, double inflation_radius,
                         bool allow_unknown) {
  map_ = map;
  occupied_threshold_ = occupied_threshold;
  inflation_radius_ = inflation_radius;
  allow_unknown_ = allow_unknown;
  rebuildInflatedMap();
}

bool GridAdapter::hasMap() const { return !map_.data.empty(); }

bool GridAdapter::worldToGrid(double wx, double wy, int *gx, int *gy) const {
  if (!hasMap() || gx == nullptr || gy == nullptr) {
    return false;
  }

  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  const int cell_x = static_cast<int>(std::floor((wx - origin_x) / resolution));
  const int cell_y = static_cast<int>(std::floor((wy - origin_y) / resolution));

  if (!inBounds(cell_x, cell_y)) {
    return false;
  }

  *gx = cell_x;
  *gy = cell_y;
  return true;
}

geometry_msgs::Point GridAdapter::gridToWorld(int gx, int gy) const {
  geometry_msgs::Point point;
  point.x =
      map_.info.origin.position.x + (static_cast<double>(gx) + 0.5) * map_.info.resolution;
  point.y =
      map_.info.origin.position.y + (static_cast<double>(gy) + 0.5) * map_.info.resolution;
  point.z = 0.0;
  return point;
}

bool GridAdapter::isCellFree(int gx, int gy) const {
  if (!inBounds(gx, gy)) {
    return false;
  }
  return inflated_occupied_[index(gx, gy)] == 0U;
}

bool GridAdapter::isSegmentFree(int x0, int y0, int x1, int y1) const {
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;

  while (true) {
    if (!isCellFree(x, y)) {
      return false;
    }
    if (x == x1 && y == y1) {
      return true;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

int GridAdapter::width() const { return static_cast<int>(map_.info.width); }

int GridAdapter::height() const { return static_cast<int>(map_.info.height); }

int GridAdapter::index(int gx, int gy) const { return gy * width() + gx; }

bool GridAdapter::inBounds(int gx, int gy) const {
  return gx >= 0 && gy >= 0 && gx < width() && gy < height();
}

void GridAdapter::rebuildInflatedMap() {
  inflated_occupied_.assign(map_.data.size(), 0U);
  if (!hasMap()) {
    return;
  }

  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / map_.info.resolution));

  for (int y = 0; y < height(); ++y) {
    for (int x = 0; x < width(); ++x) {
      const int idx = index(x, y);
      if (!isOccupiedValue(map_.data[idx])) {
        continue;
      }

      for (int oy = -inflation_cells; oy <= inflation_cells; ++oy) {
        for (int ox = -inflation_cells; ox <= inflation_cells; ++ox) {
          const int nx = x + ox;
          const int ny = y + oy;
          if (!inBounds(nx, ny)) {
            continue;
          }

          const double distance =
              std::hypot(static_cast<double>(ox), static_cast<double>(oy)) *
              map_.info.resolution;
          if (distance <= inflation_radius_) {
            inflated_occupied_[index(nx, ny)] = 1U;
          }
        }
      }
    }
  }
}

bool GridAdapter::isOccupiedValue(int8_t value) const {
  if (value < 0) {
    return !allow_unknown_;
  }
  return value >= occupied_threshold_;
}

} // namespace none_move_base_global
