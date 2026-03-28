#include "none_move_base_local/goal_checker.h"

namespace none_move_base_local {

GoalChecker::GoalChecker() : checker_(0.1, 0.17, 3, 0.05, 0.1) {}

void GoalChecker::configure(double xy_tolerance, double yaw_tolerance,
                            int latch_cycles, double linear_stop_threshold,
                            double angular_stop_threshold) {
  checker_ = none_move_base_common::ReachChecker(
      xy_tolerance, yaw_tolerance, latch_cycles, linear_stop_threshold,
      angular_stop_threshold);
}

void GoalChecker::reset() { checker_.reset(); }

bool GoalChecker::update(double distance_to_goal, double yaw_error,
                         double linear_speed, double angular_speed,
                         bool need_final_yaw) {
  return checker_.update(distance_to_goal, yaw_error, linear_speed, angular_speed,
                         need_final_yaw);
}

} // namespace none_move_base_local
