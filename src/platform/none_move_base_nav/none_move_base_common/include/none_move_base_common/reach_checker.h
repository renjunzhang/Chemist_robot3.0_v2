#pragma once

namespace none_move_base_common {

class ReachChecker {
public:
  ReachChecker(double xy_tolerance, double yaw_tolerance, int latch_cycles,
               double linear_stop_threshold, double angular_stop_threshold);

  void reset();
  bool update(double distance_to_goal, double yaw_error, double linear_speed,
              double angular_speed, bool need_final_yaw);

private:
  double xy_tolerance_;
  double yaw_tolerance_;
  int latch_cycles_;
  double linear_stop_threshold_;
  double angular_stop_threshold_;
  int consecutive_successes_;
};

} // namespace none_move_base_common
