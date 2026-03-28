#include "none_move_base_common/reach_checker.h"

#include <cmath>

namespace none_move_base_common {

ReachChecker::ReachChecker(double xy_tolerance, double yaw_tolerance,
                           int latch_cycles, double linear_stop_threshold,
                           double angular_stop_threshold)
    : xy_tolerance_(xy_tolerance), yaw_tolerance_(yaw_tolerance),
      latch_cycles_(latch_cycles),
      linear_stop_threshold_(linear_stop_threshold),
      angular_stop_threshold_(angular_stop_threshold), consecutive_successes_(0) {}

void ReachChecker::reset() { consecutive_successes_ = 0; }

bool ReachChecker::update(double distance_to_goal, double yaw_error,
                          double linear_speed, double angular_speed,
                          bool need_final_yaw) {
  const bool xy_ok = distance_to_goal <= xy_tolerance_;
  const bool yaw_ok = !need_final_yaw || std::fabs(yaw_error) <= yaw_tolerance_;
  const bool linear_ok = linear_speed <= linear_stop_threshold_;
  const bool angular_ok = std::fabs(angular_speed) <= angular_stop_threshold_;

  if (xy_ok && yaw_ok && linear_ok && angular_ok) {
    ++consecutive_successes_;
  } else {
    consecutive_successes_ = 0;
  }

  return consecutive_successes_ >= latch_cycles_;
}

} // namespace none_move_base_common
