#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include "none_move_base_common/frame_transform.h"
#include "none_move_base_common/pose_utils.h"

namespace none_move_base_local_planner {

namespace {

struct State2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ObstaclePoint {
  double x = 0.0;
  double y = 0.0;
};

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double sqr(double x) { return x * x; }

} // namespace

class MpcDcbfPlannerNode {
public:
  MpcDcbfPlannerNode()
      : private_nh_("~"), has_pose_(false), has_odom_(false), has_scan_(false),
        has_path_(false) {
    private_nh_.param<std::string>("path_topic", path_topic_,
                                   "/none_move_base/global_path");
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/amcl_pose_tf");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan_full_filtered");
    private_nh_.param<std::string>("cmd_topic", cmd_topic_,
                                   "/none_move_base/local_planner_cmd");
    private_nh_.param<std::string>("state_topic", state_topic_,
                                   "/none_move_base/local_planner_state");

    private_nh_.param("control_frequency", control_frequency_, 20.0);
    private_nh_.param("path_timeout", path_timeout_, 1.0);
    private_nh_.param("preview_distance", preview_distance_, 0.30);
    private_nh_.param("near_goal_distance", near_goal_distance_, 0.35);
    private_nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.12);
    private_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.20);
    private_nh_.param("max_vel_x", max_vel_x_, 0.12);
    private_nh_.param("max_vel_y", max_vel_y_, 0.12);
    private_nh_.param("max_wz", max_wz_, 0.35);
    private_nh_.param("near_goal_linear_scale", near_goal_linear_scale_, 0.5);

    private_nh_.param("mpc_horizon_steps", mpc_horizon_steps_, 8);
    private_nh_.param("mpc_dt", mpc_dt_, 0.10);
    private_nh_.param("sample_vx_count", sample_vx_count_, 5);
    private_nh_.param("sample_vy_count", sample_vy_count_, 5);
    private_nh_.param("sample_wz_count", sample_wz_count_, 5);

    private_nh_.param("w_pos", w_pos_, 3.0);
    private_nh_.param("w_yaw", w_yaw_, 0.8);
    private_nh_.param("w_terminal_pos", w_terminal_pos_, 5.0);
    private_nh_.param("w_terminal_yaw", w_terminal_yaw_, 1.2);
    private_nh_.param("w_control", w_control_, 0.05);
    private_nh_.param("w_smooth", w_smooth_, 0.35);
    private_nh_.param("w_obs", w_obs_, 2.0);

    private_nh_.param("obs_hard_distance", obs_hard_distance_, 0.28);
    private_nh_.param("obs_influence_distance", obs_influence_distance_, 0.55);
    private_nh_.param("max_obstacle_points", max_obstacle_points_, 80);

    path_sub_ = nh_.subscribe(path_topic_, 1, &MpcDcbfPlannerNode::pathCallback, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 1, &MpcDcbfPlannerNode::poseCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &MpcDcbfPlannerNode::odomCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &MpcDcbfPlannerNode::scanCallback, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                             &MpcDcbfPlannerNode::timerCallback, this);
    last_path_time_ = ros::Time(0);
  }

private:
  void pathCallback(const nav_msgs::PathConstPtr &msg) {
    current_path_ = *msg;
    has_path_ = !msg->poses.empty();
    last_path_time_ = ros::Time::now();
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    current_pose_ = none_move_base_common::toPoseStamped(*msg);
    has_pose_ = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    current_velocity_ = msg->twist.twist;
    has_odom_ = true;
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    current_scan_ = *msg;
    has_scan_ = true;
  }

  int findNearestIndex() const {
    int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < current_path_.poses.size(); ++i) {
      const double distance = none_move_base_common::distance2D(
          current_pose_.pose, current_path_.poses[i].pose);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = static_cast<int>(i);
      }
    }
    return nearest_index;
  }

  int findPreviewIndex(int start_index) const {
    double accumulated = 0.0;
    for (std::size_t i = static_cast<std::size_t>(start_index);
         i + 1 < current_path_.poses.size(); ++i) {
      accumulated += none_move_base_common::distance2D(current_path_.poses[i].pose,
                                                       current_path_.poses[i + 1].pose);
      if (accumulated >= preview_distance_) {
        return static_cast<int>(i + 1);
      }
    }
    return static_cast<int>(current_path_.poses.size() - 1);
  }

  int findLookaheadIndex(int start_index, double lookahead_distance) const {
    if (current_path_.poses.empty()) {
      return 0;
    }
    if (lookahead_distance <= 0.0) {
      return std::max(0, std::min(start_index,
                                  static_cast<int>(current_path_.poses.size()) - 1));
    }

    double accumulated = 0.0;
    for (std::size_t i = static_cast<std::size_t>(start_index);
         i + 1 < current_path_.poses.size(); ++i) {
      accumulated += none_move_base_common::distance2D(current_path_.poses[i].pose,
                                                       current_path_.poses[i + 1].pose);
      if (accumulated >= lookahead_distance) {
        return static_cast<int>(i + 1);
      }
    }
    return static_cast<int>(current_path_.poses.size() - 1);
  }

  double remainingDistance(int nearest_index) const {
    double remaining = none_move_base_common::distance2D(
        current_pose_.pose, current_path_.poses[nearest_index].pose);
    for (std::size_t i = static_cast<std::size_t>(nearest_index);
         i + 1 < current_path_.poses.size(); ++i) {
      remaining += none_move_base_common::distance2D(current_path_.poses[i].pose,
                                                     current_path_.poses[i + 1].pose);
    }
    return remaining;
  }

  geometry_msgs::Twist clamp(const geometry_msgs::Twist &cmd) const {
    geometry_msgs::Twist out = cmd;
    out.linear.x = std::max(-max_vel_x_, std::min(max_vel_x_, out.linear.x));
    out.linear.y = std::max(-max_vel_y_, std::min(max_vel_y_, out.linear.y));
    out.angular.z = std::max(-max_wz_, std::min(max_wz_, out.angular.z));
    return out;
  }

  std::vector<ObstaclePoint> buildObstaclePoints(const State2D &current_state) const {
    std::vector<ObstaclePoint> obstacles;
    if (!has_scan_ || current_scan_.ranges.empty() || max_obstacle_points_ <= 0) {
      return obstacles;
    }

    const std::size_t sample_stride =
        std::max<std::size_t>(1, current_scan_.ranges.size() /
                                     static_cast<std::size_t>(max_obstacle_points_));

    const double c = std::cos(current_state.yaw);
    const double s = std::sin(current_state.yaw);
    for (std::size_t i = 0; i < current_scan_.ranges.size(); i += sample_stride) {
      const double range = current_scan_.ranges[i];
      if (!std::isfinite(range) || range <= 0.0) {
        continue;
      }

      const double angle =
          current_scan_.angle_min + static_cast<double>(i) * current_scan_.angle_increment;
      const double bx = range * std::cos(angle);
      const double by = range * std::sin(angle);

      ObstaclePoint p;
      p.x = current_state.x + c * bx - s * by;
      p.y = current_state.y + s * bx + c * by;
      obstacles.push_back(p);
    }
    return obstacles;
  }

  State2D propagate(const State2D &state, const geometry_msgs::Twist &u,
                    double dt) const {
    State2D next = state;
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const double vx_world = c * u.linear.x - s * u.linear.y;
    const double vy_world = s * u.linear.x + c * u.linear.y;
    next.x += vx_world * dt;
    next.y += vy_world * dt;
    next.yaw = normalizeAngle(state.yaw + u.angular.z * dt);
    return next;
  }

  bool obstacleConstraintSatisfied(const State2D &state,
                                   const std::vector<ObstaclePoint> &obstacles,
                                   double *obs_cost) const {
    *obs_cost = 0.0;
    if (obstacles.empty()) {
      return true;
    }

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto &p : obstacles) {
      const double d = std::hypot(state.x - p.x, state.y - p.y);
      min_dist = std::min(min_dist, d);
    }

    if (min_dist < obs_hard_distance_) {
      return false;
    }
    if (min_dist < obs_influence_distance_) {
      *obs_cost = w_obs_ * sqr(obs_influence_distance_ - min_dist);
    }
    return true;
  }

  std::vector<double> buildSampleAxis(double max_abs, int sample_count,
                                      bool near_goal) const {
    std::vector<double> samples;
    const int count = std::max(3, sample_count);
    samples.reserve(static_cast<std::size_t>(count));
    const double scale = near_goal ? near_goal_linear_scale_ : 1.0;
    const double limit = std::max(0.01, max_abs * scale);
    for (int i = 0; i < count; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(count - 1);
      samples.push_back(-limit + 2.0 * limit * alpha);
    }
    return samples;
  }

  bool solveHolonomicMpc(const State2D &state0, int nearest_index,
                         const geometry_msgs::Twist &velocity_seed,
                         const std::vector<ObstaclePoint> &obstacles,
                         geometry_msgs::Twist *best_cmd) const {
    if (best_cmd == nullptr || current_path_.poses.empty()) {
      return false;
    }

    const bool near_goal = remainingDistance(nearest_index) <= near_goal_distance_;
    const std::vector<double> vx_samples =
        buildSampleAxis(max_vel_x_, sample_vx_count_, near_goal);
    const std::vector<double> vy_samples =
        buildSampleAxis(max_vel_y_, sample_vy_count_, near_goal);
    const std::vector<double> wz_samples =
        buildSampleAxis(max_wz_, sample_wz_count_, false);

    double best_cost = std::numeric_limits<double>::infinity();
    geometry_msgs::Twist best;
    bool found = false;

    for (double vx : vx_samples) {
      for (double vy : vy_samples) {
        for (double wz : wz_samples) {
          geometry_msgs::Twist u;
          u.linear.x = vx;
          u.linear.y = vy;
          u.angular.z = wz;

          State2D state = state0;
          double cost = 0.0;
          bool feasible = true;

          for (int k = 0; k < std::max(1, mpc_horizon_steps_); ++k) {
            state = propagate(state, u, mpc_dt_);

            const int ref_idx =
                findLookaheadIndex(nearest_index, (k + 1) * preview_distance_);
            const auto &ref_pose = current_path_.poses[ref_idx].pose;
            const double ref_yaw = none_move_base_common::yawFromPose(ref_pose);
            const double pos_err =
                std::hypot(state.x - ref_pose.position.x, state.y - ref_pose.position.y);
            const double yaw_err = normalizeAngle(ref_yaw - state.yaw);

            cost += w_pos_ * sqr(pos_err);
            cost += w_yaw_ * sqr(yaw_err);
            cost += w_control_ * (sqr(u.linear.x) + sqr(u.linear.y) + sqr(u.angular.z));

            double obs_cost = 0.0;
            if (!obstacleConstraintSatisfied(state, obstacles, &obs_cost)) {
              feasible = false;
              break;
            }
            cost += obs_cost;
          }

          if (!feasible) {
            continue;
          }

          const auto &goal_pose = current_path_.poses.back().pose;
          const double goal_yaw = none_move_base_common::yawFromPose(goal_pose);
          const double terminal_pos_err =
              std::hypot(state.x - goal_pose.position.x, state.y - goal_pose.position.y);
          const double terminal_yaw_err = normalizeAngle(goal_yaw - state.yaw);
          cost += w_terminal_pos_ * sqr(terminal_pos_err);
          cost += w_terminal_yaw_ * sqr(terminal_yaw_err);

          cost += w_smooth_ *
                  (sqr(u.linear.x - velocity_seed.linear.x) +
                   sqr(u.linear.y - velocity_seed.linear.y) +
                   sqr(u.angular.z - velocity_seed.angular.z));

          if (cost < best_cost) {
            best_cost = cost;
            best = u;
            found = true;
          }
        }
      }
    }

    if (!found) {
      return false;
    }

    *best_cmd = clamp(best);
    return true;
  }

  void publishState(const std::string &text) {
    std_msgs::String state;
    state.data = text;
    state_pub_.publish(state);
  }

  void publishZero(const std::string &state_text) {
    cmd_pub_.publish(geometry_msgs::Twist());
    publishState(state_text);
  }

  void timerCallback(const ros::TimerEvent &) {
    if (!has_pose_) {
      publishZero("no_pose");
      return;
    }

    if (!has_path_ || (ros::Time::now() - last_path_time_).toSec() > path_timeout_) {
      publishZero("no_path");
      return;
    }

    const int nearest_index = findNearestIndex();
    const double remain = remainingDistance(nearest_index);
    const geometry_msgs::PoseStamped &goal_pose = current_path_.poses.back();

    const double current_yaw = none_move_base_common::yawFromPose(current_pose_.pose);
    const double goal_yaw = none_move_base_common::yawFromPose(goal_pose.pose);
    const double goal_yaw_error = normalizeAngle(goal_yaw - current_yaw);

    if (remain <= xy_goal_tolerance_ && std::fabs(goal_yaw_error) <= yaw_goal_tolerance_) {
      last_command_ = geometry_msgs::Twist();
      publishZero("goal_reached");
      return;
    }

    State2D state0;
    state0.x = current_pose_.pose.position.x;
    state0.y = current_pose_.pose.position.y;
    state0.yaw = current_yaw;

    const geometry_msgs::Twist velocity_seed =
        has_odom_ ? current_velocity_ : last_command_;
    const std::vector<ObstaclePoint> obstacles = buildObstaclePoints(state0);

    geometry_msgs::Twist best_cmd;
    if (!solveHolonomicMpc(state0, nearest_index, velocity_seed, obstacles, &best_cmd)) {
      last_command_ = geometry_msgs::Twist();
      publishZero("mpc_solve_failed");
      return;
    }

    last_command_ = best_cmd;
    cmd_pub_.publish(best_cmd);
    if (remain <= xy_goal_tolerance_) {
      publishState("final_yaw_align");
    } else {
      publishState("mpc_dcbf_tracking");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber path_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher state_pub_;
  ros::Timer timer_;

  std::string path_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_topic_;
  std::string state_topic_;

  double control_frequency_;
  double path_timeout_;
  double preview_distance_;
  double near_goal_distance_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double max_vel_x_;
  double max_vel_y_;
  double max_wz_;
  double near_goal_linear_scale_;

  int mpc_horizon_steps_;
  double mpc_dt_;
  int sample_vx_count_;
  int sample_vy_count_;
  int sample_wz_count_;
  double w_pos_;
  double w_yaw_;
  double w_terminal_pos_;
  double w_terminal_yaw_;
  double w_control_;
  double w_smooth_;
  double w_obs_;
  double obs_hard_distance_;
  double obs_influence_distance_;
  int max_obstacle_points_;

  bool has_pose_;
  bool has_odom_;
  bool has_scan_;
  bool has_path_;
  ros::Time last_path_time_;

  nav_msgs::Path current_path_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::Twist current_velocity_;
  geometry_msgs::Twist last_command_;
  sensor_msgs::LaserScan current_scan_;
};

} // namespace none_move_base_local_planner

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_dcbf_planner_node");
  none_move_base_local_planner::MpcDcbfPlannerNode node;
  ros::spin();
  return 0;
}
