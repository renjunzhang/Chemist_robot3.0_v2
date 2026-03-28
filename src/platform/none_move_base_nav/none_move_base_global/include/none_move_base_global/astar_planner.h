#pragma once

#include <vector>

namespace none_move_base_global {

struct GridCell {
  int x = 0;
  int y = 0;

  bool operator==(const GridCell &other) const {
    return x == other.x && y == other.y;
  }
};

class GridAdapter;

class AStarPlanner {
public:
  bool plan(const GridAdapter &grid, const GridCell &start, const GridCell &goal,
            std::vector<GridCell> *path) const;
};

} // namespace none_move_base_global
