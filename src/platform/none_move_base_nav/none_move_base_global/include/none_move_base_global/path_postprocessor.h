#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "none_move_base_global/astar_planner.h"

namespace none_move_base_global {

class GridAdapter;

class PathPostprocessor {
public:
  std::vector<GridCell> removeCollinear(
      const std::vector<GridCell> &raw_path) const;
  std::vector<GridCell> pruneLineOfSight(const std::vector<GridCell> &raw_path,
                                         const GridAdapter &grid) const;
  nav_msgs::Path buildPath(const std::vector<GridCell> &cell_path,
                           const GridAdapter &grid,
                           const geometry_msgs::PoseStamped &start_pose,
                           const geometry_msgs::PoseStamped &goal_pose,
                           const std::string &frame_id, const ros::Time &stamp,
                           double resample_step) const;
};

} // namespace none_move_base_global
