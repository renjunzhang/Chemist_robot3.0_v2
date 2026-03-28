#include "none_move_base_local/velocity_limiter.h"

#include <algorithm>

namespace none_move_base_local {

VelocityLimiter::VelocityLimiter() : acc_lim_x_(0.0), acc_lim_y_(0.0), acc_lim_wz_(0.0) {}

void VelocityLimiter::configure(double acc_lim_x, double acc_lim_y,
                                double acc_lim_wz) {
  acc_lim_x_ = acc_lim_x;
  acc_lim_y_ = acc_lim_y;
  acc_lim_wz_ = acc_lim_wz;
}

geometry_msgs::Twist VelocityLimiter::limit(const geometry_msgs::Twist &desired,
                                            const geometry_msgs::Twist &current,
                                            double dt) const {
  geometry_msgs::Twist output = desired;
  if (dt <= 0.0) {
    return output;
  }

  output.linear.x = limitAxis(desired.linear.x, current.linear.x, acc_lim_x_, dt);
  output.linear.y = limitAxis(desired.linear.y, current.linear.y, acc_lim_y_, dt);
  output.angular.z = limitAxis(desired.angular.z, current.angular.z, acc_lim_wz_, dt);
  return output;
}

double VelocityLimiter::limitAxis(double desired, double current,
                                  double accel_limit, double dt) const {
  if (accel_limit <= 0.0 || dt <= 0.0) {
    return desired;
  }

  const double max_delta = accel_limit * dt;
  const double delta = desired - current;
  if (delta > max_delta) {
    return current + max_delta;
  }
  if (delta < -max_delta) {
    return current - max_delta;
  }
  return desired;
}

} // namespace none_move_base_local
