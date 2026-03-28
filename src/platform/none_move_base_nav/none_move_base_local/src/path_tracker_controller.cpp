#include "none_move_base_local/path_tracker_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "none_move_base_common/pose_utils.h"

namespace none_move_base_local {

PathTrackerController::PathTrackerController() = default;

void PathTrackerController::configure(const ControllerParams &params, double acc_lim_x,
                                      double acc_lim_y, double acc_lim_wz,
                                      double obstacle_stop_distance,
                                      double obstacle_sector_width) {
  params_ = params;
  goal_checker_.configure(params.xy_goal_tolerance, params.yaw_goal_tolerance,
                          params.success_latch_cycles, 0.05, 0.10);
  obstacle_adapter_.configure(obstacle_stop_distance, obstacle_sector_width);
  velocity_limiter_.configure(acc_lim_x, acc_lim_y, acc_lim_wz);
}

void PathTrackerController::setPath(const nav_msgs::Path &path) {
  path_ = path;
  goal_checker_.reset();
}

void PathTrackerController::clearPath() {
  path_.poses.clear();
  goal_checker_.reset();
  last_command_ = geometry_msgs::Twist();
}

ControllerOutput PathTrackerController::compute(
    const geometry_msgs::PoseStamped &current_pose,
    const geometry_msgs::Twist &current_velocity, bool has_scan,
    const sensor_msgs::LaserScan &scan, double dt) {
  ControllerOutput output;
  output.has_path = !path_.poses.empty();

  if (path_.poses.empty()) {
    output.tracking_status = 0;
    output.detail = "no_path";
    last_command_ = geometry_msgs::Twist();
    return output;
  }

  const int nearest_index = findNearestIndex(current_pose);
  output.cross_track_error = none_move_base_common::distance2D(
      current_pose.pose, path_.poses[nearest_index].pose);
  output.remaining_distance = remainingDistanceFrom(current_pose, nearest_index);
  output.near_goal = output.remaining_distance <= params_.near_goal_distance;

  const geometry_msgs::PoseStamped &goal_pose = path_.poses.back();
  const double current_yaw = none_move_base_common::yawFromPose(current_pose.pose);
  const double goal_yaw =
      none_move_base_common::yawFromPose(goal_pose.pose);
  const double goal_yaw_error =
      none_move_base_common::normalizeAngle(goal_yaw - current_yaw);

  geometry_msgs::Twist desired_command;
  desired_command.linear.x = 0.0;
  desired_command.linear.y = 0.0;
  desired_command.angular.z = 0.0;

  const double current_linear_speed =
      std::hypot(current_velocity.linear.x, current_velocity.linear.y);

  if (output.remaining_distance <= params_.xy_goal_tolerance) {
    if (params_.need_final_yaw &&
        std::fabs(goal_yaw_error) > params_.yaw_goal_tolerance) {
      output.tracking_status = 2;
      output.heading_error = goal_yaw_error;
      output.detail = "final_yaw_align";
      desired_command.angular.z = params_.k_final_heading * goal_yaw_error;
    } else {
      output.tracking_status = 2;
      output.heading_error = goal_yaw_error;
      output.detail = "goal_latching";
      output.goal_reached = goal_checker_.update(
          output.remaining_distance, goal_yaw_error, current_linear_speed,
          current_velocity.angular.z, params_.need_final_yaw);
      if (output.goal_reached) {
        output.tracking_status = 3;
        output.detail = "goal_reached";
      }
      last_command_ = geometry_msgs::Twist();
      return output;
    }
  } else {
    const double preview_distance =
        output.near_goal ? params_.near_goal_preview_distance
                         : params_.preview_distance;
    const int preview_index = findPreviewIndex(nearest_index, preview_distance);
    const auto &preview_pose = path_.poses[preview_index];
    const geometry_msgs::Point local_point =
        none_move_base_common::worldToBody(preview_pose.pose.position, current_pose.pose);

    output.heading_error = none_move_base_common::normalizeAngle(
        none_move_base_common::yawFromPose(preview_pose.pose) - current_yaw);
    desired_command.linear.x = params_.k_position * local_point.x;
    desired_command.linear.y = params_.k_position * local_point.y;
    desired_command.angular.z =
        (output.near_goal ? params_.k_final_heading : params_.k_heading) *
        output.heading_error;
    output.tracking_status = output.near_goal ? 2 : 1;
    output.detail = output.near_goal ? "near_goal_tracking" : "tracking";
  }

  desired_command = saturate(desired_command, output.near_goal);
  applyDeadband(&desired_command, output.near_goal);

  const double translational_heading =
      std::atan2(desired_command.linear.y, desired_command.linear.x);
  const double translational_speed =
      std::hypot(desired_command.linear.x, desired_command.linear.y);

  if (has_scan && translational_speed > 1e-3 &&
      obstacle_adapter_.isBlocked(scan, translational_heading)) {
    output.blocked = true;
    output.tracking_status = 5;
    output.detail = "blocked_by_scan";
    desired_command.linear.x = 0.0;
    desired_command.linear.y = 0.0;
    desired_command.angular.z = 0.0;
  }

  output.command = velocity_limiter_.limit(desired_command, last_command_, dt);
  last_command_ = output.command;
  return output;
}

int PathTrackerController::findNearestIndex(
    const geometry_msgs::PoseStamped &current_pose) const {
  int nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < path_.poses.size(); ++i) {
    const double distance = none_move_base_common::distance2D(
        current_pose.pose, path_.poses[i].pose);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = static_cast<int>(i);
    }
  }
  return nearest_index;
}

int PathTrackerController::findPreviewIndex(int start_index,
                                            double preview_distance) const {
  if (path_.poses.empty()) {
    return 0;
  }

  double accumulated = 0.0;
  for (std::size_t i = static_cast<std::size_t>(start_index);
       i + 1 < path_.poses.size(); ++i) {
    accumulated += none_move_base_common::distance2D(path_.poses[i].pose,
                                                     path_.poses[i + 1].pose);
    if (accumulated >= preview_distance) {
      return static_cast<int>(i + 1);
    }
  }
  return static_cast<int>(path_.poses.size() - 1);
}

double PathTrackerController::remainingDistanceFrom(
    const geometry_msgs::PoseStamped &current_pose, int nearest_index) const {
  double remaining =
      none_move_base_common::distance2D(current_pose.pose, path_.poses[nearest_index].pose);
  for (std::size_t i = static_cast<std::size_t>(nearest_index);
       i + 1 < path_.poses.size(); ++i) {
    remaining +=
        none_move_base_common::distance2D(path_.poses[i].pose, path_.poses[i + 1].pose);
  }
  return remaining;
}

geometry_msgs::Twist PathTrackerController::saturate(
    const geometry_msgs::Twist &command, bool near_goal) const {
  geometry_msgs::Twist limited = command;

  const double max_vx = near_goal ? params_.near_goal_max_vel_x : params_.max_vel_x;
  const double max_vy = near_goal ? params_.near_goal_max_vel_y : params_.max_vel_y;
  const double max_wz = near_goal ? params_.near_goal_max_wz : params_.max_wz;

  limited.linear.x = std::max(-max_vx, std::min(max_vx, limited.linear.x));
  limited.linear.y = std::max(-max_vy, std::min(max_vy, limited.linear.y));
  limited.angular.z = std::max(-max_wz, std::min(max_wz, limited.angular.z));
  return limited;
}

void PathTrackerController::applyDeadband(geometry_msgs::Twist *command,
                                          bool near_goal) const {
  if (command == nullptr) {
    return;
  }

  if (std::fabs(command->linear.x) < params_.min_cmd_vel_xy) {
    command->linear.x = 0.0;
  }
  if (std::fabs(command->linear.y) < params_.min_cmd_vel_xy) {
    command->linear.y = 0.0;
  }
  if (std::fabs(command->angular.z) < params_.min_cmd_wz) {
    command->angular.z = 0.0;
  }

  if (!near_goal) {
    return;
  }

  if (command->linear.x != 0.0 &&
      std::fabs(command->linear.x) < params_.min_cmd_vel_xy) {
    command->linear.x =
        std::copysign(params_.min_cmd_vel_xy, command->linear.x);
  }
  if (command->linear.y != 0.0 &&
      std::fabs(command->linear.y) < params_.min_cmd_vel_xy) {
    command->linear.y =
        std::copysign(params_.min_cmd_vel_xy, command->linear.y);
  }
}

} // namespace none_move_base_local
