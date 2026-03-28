#pragma once

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>

#include "none_move_base_local/goal_checker.h"
#include "none_move_base_local/obstacle_adapter.h"
#include "none_move_base_local/velocity_limiter.h"

namespace none_move_base_local {

struct ControllerParams {
  double preview_distance = 0.4;
  double near_goal_preview_distance = 0.12;
  double near_goal_distance = 0.4;
  double xy_goal_tolerance = 0.12;
  double yaw_goal_tolerance = 0.17;
  bool need_final_yaw = true;
  int success_latch_cycles = 5;
  double k_position = 1.2;
  double k_heading = 1.5;
  double k_final_heading = 1.8;
  double max_vel_x = 0.35;
  double max_vel_y = 0.35;
  double max_wz = 0.8;
  double near_goal_max_vel_x = 0.15;
  double near_goal_max_vel_y = 0.15;
  double near_goal_max_wz = 0.45;
  double min_cmd_vel_xy = 0.03;
  double min_cmd_wz = 0.05;
};

struct ControllerOutput {
  geometry_msgs::Twist command;
  bool has_path = false;
  bool goal_reached = false;
  bool oscillating = false;
  bool blocked = false;
  bool near_goal = false;
  double cross_track_error = 0.0;
  double heading_error = 0.0;
  double remaining_distance = 0.0;
  int tracking_status = 0;
  std::string detail;
};

class PathTrackerController {
public:
  PathTrackerController();

  void configure(const ControllerParams &params, double acc_lim_x, double acc_lim_y,
                 double acc_lim_wz, double obstacle_stop_distance,
                 double obstacle_sector_width);
  void setPath(const nav_msgs::Path &path);
  void clearPath();
  ControllerOutput compute(const geometry_msgs::PoseStamped &current_pose,
                           const geometry_msgs::Twist &current_velocity, bool has_scan,
                           const sensor_msgs::LaserScan &scan, double dt);

private:
  int findNearestIndex(const geometry_msgs::PoseStamped &current_pose) const;
  int findPreviewIndex(int start_index, double preview_distance) const;
  double remainingDistanceFrom(const geometry_msgs::PoseStamped &current_pose,
                               int nearest_index) const;
  geometry_msgs::Twist saturate(const geometry_msgs::Twist &command,
                                bool near_goal) const;
  void applyDeadband(geometry_msgs::Twist *command, bool near_goal) const;

  ControllerParams params_;
  nav_msgs::Path path_;
  GoalChecker goal_checker_;
  ObstacleAdapter obstacle_adapter_;
  VelocityLimiter velocity_limiter_;
  geometry_msgs::Twist last_command_;
};

} // namespace none_move_base_local
