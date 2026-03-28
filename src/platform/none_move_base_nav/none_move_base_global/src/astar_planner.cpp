#include "none_move_base_global/astar_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "none_move_base_global/grid_adapter.h"

namespace none_move_base_global {

namespace {

struct QueueNode {
  int index;
  double f_score;

  bool operator<(const QueueNode &other) const { return f_score > other.f_score; }
};

double heuristic(const GridCell &a, const GridCell &b) {
  return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
}

} // namespace

bool AStarPlanner::plan(const GridAdapter &grid, const GridCell &start,
                        const GridCell &goal,
                        std::vector<GridCell> *path) const {
  if (path == nullptr || !grid.hasMap() || !grid.isCellFree(start.x, start.y) ||
      !grid.isCellFree(goal.x, goal.y)) {
    return false;
  }

  const int total_cells = grid.width() * grid.height();
  std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total_cells, -1);
  std::vector<uint8_t> closed(total_cells, 0U);
  std::priority_queue<QueueNode> open_set;

  const int start_index = grid.index(start.x, start.y);
  const int goal_index = grid.index(goal.x, goal.y);
  g_score[start_index] = 0.0;
  open_set.push({start_index, heuristic(start, goal)});

  static const int kNeighborDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int kNeighborDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  while (!open_set.empty()) {
    const QueueNode current = open_set.top();
    open_set.pop();

    if (closed[current.index] != 0U) {
      continue;
    }
    closed[current.index] = 1U;

    if (current.index == goal_index) {
      std::vector<GridCell> reversed;
      for (int idx = goal_index; idx != -1; idx = parent[idx]) {
        const int x = idx % grid.width();
        const int y = idx / grid.width();
        reversed.push_back({x, y});
      }
      std::reverse(reversed.begin(), reversed.end());
      *path = reversed;
      return true;
    }

    const int current_x = current.index % grid.width();
    const int current_y = current.index / grid.width();
    const GridCell current_cell{current_x, current_y};

    for (int i = 0; i < 8; ++i) {
      const int nx = current_x + kNeighborDx[i];
      const int ny = current_y + kNeighborDy[i];
      if (!grid.isCellFree(nx, ny)) {
        continue;
      }

      const int neighbor_index = grid.index(nx, ny);
      if (closed[neighbor_index] != 0U) {
        continue;
      }

      const GridCell neighbor{nx, ny};
      const double step_cost = heuristic(current_cell, neighbor);
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g >= g_score[neighbor_index]) {
        continue;
      }

      parent[neighbor_index] = current.index;
      g_score[neighbor_index] = tentative_g;
      const double f_score = tentative_g + heuristic(neighbor, goal);
      open_set.push({neighbor_index, f_score});
    }
  }

  return false;
}

} // namespace none_move_base_global
