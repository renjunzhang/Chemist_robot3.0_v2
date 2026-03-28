#pragma once

#include <geometry_msgs/Twist.h>

namespace none_move_base_local {

class VelocityLimiter {
public:
  VelocityLimiter();

  void configure(double acc_lim_x, double acc_lim_y, double acc_lim_wz);
  geometry_msgs::Twist limit(const geometry_msgs::Twist &desired,
                             const geometry_msgs::Twist &current,
                             double dt) const;

private:
  double limitAxis(double desired, double current, double accel_limit,
                   double dt) const;

  double acc_lim_x_;
  double acc_lim_y_;
  double acc_lim_wz_;
};

} // namespace none_move_base_local
