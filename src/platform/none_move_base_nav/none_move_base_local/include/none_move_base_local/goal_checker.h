#pragma once

#include "none_move_base_common/reach_checker.h"

namespace none_move_base_local {

class GoalChecker {
public:
  GoalChecker();

  void configure(double xy_tolerance, double yaw_tolerance, int latch_cycles,
                 double linear_stop_threshold, double angular_stop_threshold);
  void reset();
  bool update(double distance_to_goal, double yaw_error, double linear_speed,
              double angular_speed, bool need_final_yaw);

private:
  none_move_base_common::ReachChecker checker_;
};

} // namespace none_move_base_local
