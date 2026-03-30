#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "none_move_base_common/frame_transform.h"
#include "none_move_base_local/obstacle_adapter.h"
#include "none_move_base_local/path_tracker_controller.h"
#include "none_move_base_local/velocity_limiter.h"
#include "none_move_base_msgs/PathTrackingState.h"

namespace none_move_base_local {

class LocalControllerNode {
public:
  LocalControllerNode()
      : private_nh_("~"), has_pose_(false), has_odom_(false), has_scan_(false),
        has_joy_(false), enable_button_pressed_(false), has_seen_enable_press_(false),
        has_active_path_(false), has_local_planner_cmd_(false),
        release_active_(false), release_timeout_cleared_(false) {
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/amcl_pose_tf");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan_full_filtered");
    private_nh_.param<std::string>("path_topic", path_topic_, "/none_move_base/global_path");
    private_nh_.param<std::string>("clear_path_topic", clear_path_topic_,
                                   "/none_move_base/clear_path");
    private_nh_.param<std::string>("tracking_state_topic", tracking_state_topic_,
                                   "/none_move_base/path_tracking_state");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                                   "/hf_platform/nav_vel");
    private_nh_.param<std::string>("local_mode", local_mode_, "tracker");
    private_nh_.param<std::string>("local_planner_cmd_topic",
                     local_planner_cmd_topic_,
                     "/none_move_base/local_planner_cmd");
    private_nh_.param<std::string>("local_planner_state_topic",
                     local_planner_state_topic_,
                     "/none_move_base/local_planner_state");
    private_nh_.param("local_planner_cmd_timeout", local_planner_cmd_timeout_,
              0.5);
    private_nh_.param<std::string>("enable_button_topic", enable_button_topic_,
                     "/hf_platform/joy");
    private_nh_.param("require_enable_button", require_enable_button_, false);
    private_nh_.param("enable_button_index", enable_button_index_, 5);
    private_nh_.param("enable_release_timeout", enable_release_timeout_, 1.5);
    private_nh_.param<std::string>("enable_release_behavior", enable_release_behavior_,
                     "pause");
    private_nh_.param("joy_msg_timeout", joy_msg_timeout_, 0.3);
    private_nh_.param("publish_zero_on_block", publish_zero_on_block_, true);
    private_nh_.param("control_frequency", control_frequency_, 20.0);
    private_nh_.param("path_timeout", path_timeout_, 1.0);

    ControllerParams params;
    private_nh_.param("preview_distance", params.preview_distance, 0.40);
    private_nh_.param("near_goal_preview_distance",
                      params.near_goal_preview_distance, 0.12);
    private_nh_.param("near_goal_distance", params.near_goal_distance, 0.40);
    private_nh_.param("xy_goal_tolerance", params.xy_goal_tolerance, 0.12);
    private_nh_.param("yaw_goal_tolerance", params.yaw_goal_tolerance, 0.17);
    private_nh_.param("need_final_yaw", params.need_final_yaw, true);
    private_nh_.param("success_latch_cycles", params.success_latch_cycles, 5);
    private_nh_.param("k_position", params.k_position, 1.20);
    private_nh_.param("k_heading", params.k_heading, 1.50);
    private_nh_.param("k_final_heading", params.k_final_heading, 1.80);
    private_nh_.param("max_vel_x", params.max_vel_x, 0.35);
    private_nh_.param("max_vel_y", params.max_vel_y, 0.35);
    private_nh_.param("max_wz", params.max_wz, 0.80);
    private_nh_.param("near_goal_max_vel_x", params.near_goal_max_vel_x, 0.15);
    private_nh_.param("near_goal_max_vel_y", params.near_goal_max_vel_y, 0.15);
    private_nh_.param("near_goal_max_wz", params.near_goal_max_wz, 0.45);
    private_nh_.param("min_cmd_vel_xy", params.min_cmd_vel_xy, 0.03);
    private_nh_.param("min_cmd_wz", params.min_cmd_wz, 0.05);

    double acc_lim_x = 0.60;
    double acc_lim_y = 0.60;
    double acc_lim_wz = 1.20;
    double obstacle_stop_distance = 0.35;
    double obstacle_sector_width = 0.70;
    private_nh_.param("acc_lim_x", acc_lim_x, 0.60);
    private_nh_.param("acc_lim_y", acc_lim_y, 0.60);
    private_nh_.param("acc_lim_wz", acc_lim_wz, 1.20);
    private_nh_.param("obstacle_stop_distance", obstacle_stop_distance, 0.35);
    private_nh_.param("obstacle_sector_width", obstacle_sector_width, 0.70);

    max_vel_x_ = params.max_vel_x;
    max_vel_y_ = params.max_vel_y;
    max_wz_ = params.max_wz;
    near_goal_max_vel_x_ = params.near_goal_max_vel_x;
    near_goal_max_vel_y_ = params.near_goal_max_vel_y;
    near_goal_max_wz_ = params.near_goal_max_wz;
    min_cmd_vel_xy_ = params.min_cmd_vel_xy;
    min_cmd_wz_ = params.min_cmd_wz;

    controller_.configure(params, acc_lim_x, acc_lim_y, acc_lim_wz,
                          obstacle_stop_distance, obstacle_sector_width);
    mpc_velocity_limiter_.configure(acc_lim_x, acc_lim_y, acc_lim_wz);
    mpc_obstacle_adapter_.configure(obstacle_stop_distance, obstacle_sector_width);

    pose_sub_ =
        nh_.subscribe(pose_topic_, 1, &LocalControllerNode::poseCallback, this);
    odom_sub_ =
        nh_.subscribe(odom_topic_, 1, &LocalControllerNode::odomCallback, this);
    scan_sub_ =
        nh_.subscribe(scan_topic_, 1, &LocalControllerNode::scanCallback, this);
    path_sub_ =
        nh_.subscribe(path_topic_, 1, &LocalControllerNode::pathCallback, this);
    local_planner_cmd_sub_ = nh_.subscribe(local_planner_cmd_topic_, 1,
                         &LocalControllerNode::localPlannerCmdCallback,
                         this);
    local_planner_state_sub_ = nh_.subscribe(
      local_planner_state_topic_, 1,
      &LocalControllerNode::localPlannerStateCallback, this);
    joy_sub_ =
      nh_.subscribe(enable_button_topic_, 1, &LocalControllerNode::joyCallback, this);
    clear_path_sub_ = nh_.subscribe(clear_path_topic_, 1,
                                    &LocalControllerNode::clearPathCallback, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    tracking_state_pub_ =
        nh_.advertise<none_move_base_msgs::PathTrackingState>(tracking_state_topic_, 1);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_frequency_),
        &LocalControllerNode::controlTimerCallback, this);
    last_path_time_ = ros::Time(0);
  }

private:
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

  void pathCallback(const nav_msgs::PathConstPtr &msg) {
    controller_.setPath(*msg);
    last_path_time_ = ros::Time::now();
    has_active_path_ = !msg->poses.empty();
    if (!has_active_path_) {
      has_local_planner_cmd_ = false;
    }
  }

  void localPlannerCmdCallback(const geometry_msgs::TwistConstPtr &msg) {
    latest_local_planner_cmd_ = *msg;
    last_local_planner_cmd_time_ = ros::Time::now();
    has_local_planner_cmd_ = true;
  }

  void localPlannerStateCallback(const std_msgs::StringConstPtr &msg) {
    latest_local_planner_state_ = msg->data;
  }

  void joyCallback(const sensor_msgs::JoyConstPtr &msg) {
    has_joy_ = true;
    last_joy_time_ = ros::Time::now();

    bool pressed = false;
    if (enable_button_index_ >= 0 &&
        static_cast<size_t>(enable_button_index_) < msg->buttons.size()) {
      pressed = (msg->buttons[enable_button_index_] == 1);
    }

    enable_button_pressed_ = pressed;
    if (pressed) {
      has_seen_enable_press_ = true;
    }
  }

  void clearPathCallback(const std_msgs::EmptyConstPtr & /*msg*/) {
    controller_.clearPath();
    has_active_path_ = false;
    publishZero();
  }

  bool shouldAllowOutput(const ros::Time &now, std::string *detail,
                         bool *clear_path_due_to_timeout) {
    *clear_path_due_to_timeout = false;
    if (!require_enable_button_) {
      release_active_ = false;
      release_timeout_cleared_ = false;
      return true;
    }

    if (!has_joy_ || (now - last_joy_time_).toSec() > joy_msg_timeout_) {
      release_active_ = false;
      release_timeout_cleared_ = false;
      *detail = "joy_timeout_fail_safe";
      return false;
    }

    if (enable_button_pressed_) {
      release_active_ = false;
      release_timeout_cleared_ = false;
      return true;
    }

    if (!release_active_) {
      release_active_ = true;
      release_start_time_ = now;
      release_timeout_cleared_ = false;
    }

    const double release_elapsed = (now - release_start_time_).toSec();
    if (release_elapsed >= enable_release_timeout_ &&
        enable_release_behavior_ == "cancel") {
      *detail = "manual_hold_timeout_cleared";
      if (!release_timeout_cleared_) {
        *clear_path_due_to_timeout = true;
        release_timeout_cleared_ = true;
      }
    } else {
      *detail = has_seen_enable_press_ ? "manual_hold_released"
                                       : "hold_to_run_required";
    }

    return false;
  }

  bool isMpcMode() const { return local_mode_ == "mpc_dcbf"; }

  geometry_msgs::Twist clampMpcCommand(const geometry_msgs::Twist &raw,
                                       bool near_goal) const {
    geometry_msgs::Twist cmd = raw;
    const double max_vx = near_goal ? near_goal_max_vel_x_ : max_vel_x_;
    const double max_vy = near_goal ? near_goal_max_vel_y_ : max_vel_y_;
    const double max_wz = near_goal ? near_goal_max_wz_ : max_wz_;

    cmd.linear.x = std::max(-max_vx, std::min(max_vx, cmd.linear.x));
    cmd.linear.y = std::max(-max_vy, std::min(max_vy, cmd.linear.y));
    cmd.angular.z = std::max(-max_wz, std::min(max_wz, cmd.angular.z));

    if (std::fabs(cmd.linear.x) < min_cmd_vel_xy_) {
      cmd.linear.x = 0.0;
    }
    if (std::fabs(cmd.linear.y) < min_cmd_vel_xy_) {
      cmd.linear.y = 0.0;
    }
    if (std::fabs(cmd.angular.z) < min_cmd_wz_) {
      cmd.angular.z = 0.0;
    }
    return cmd;
  }

  void controlTimerCallback(const ros::TimerEvent &event) {
    none_move_base_msgs::PathTrackingState state;
    state.header.stamp = ros::Time::now();

    if (!has_pose_) {
      state.detail = "no_pose";
      state.tracking_status = 7;
      tracking_state_pub_.publish(state);
      publishZero();
      return;
    }

    std::string gate_detail;
    bool clear_path_due_to_timeout = false;
    if (!shouldAllowOutput(state.header.stamp, &gate_detail,
                           &clear_path_due_to_timeout)) {
      if (clear_path_due_to_timeout) {
        controller_.clearPath();
        has_active_path_ = false;
      }

      state.has_path = has_active_path_;
      state.goal_reached = false;
      state.oscillating = false;
      state.blocked = false;
      state.cross_track_error = 0.0;
      state.heading_error = 0.0;
      state.remaining_distance = 0.0;
      state.commanded_vx = 0.0;
      state.commanded_vy = 0.0;
      state.commanded_wz = 0.0;
      state.tracking_status = 0;
      state.detail = gate_detail;

      tracking_state_pub_.publish(state);
      if (publish_zero_on_block_) {
        publishZero();
      }
      return;
    }

    const double dt = std::max(1e-3, (event.current_real - event.last_real).toSec());
    const geometry_msgs::Twist current_velocity =
        has_odom_ ? current_velocity_ : geometry_msgs::Twist();

    ControllerOutput output =
        controller_.compute(current_pose_, current_velocity, has_scan_, current_scan_, dt);

    if (!output.has_path && (ros::Time::now() - last_path_time_).toSec() > path_timeout_) {
      has_active_path_ = false;
      state.detail = "path_timeout";
      state.tracking_status = 4;
      tracking_state_pub_.publish(state);
      publishZero();
      return;
    }

    has_active_path_ = output.has_path;

    if (isMpcMode()) {
      if (!has_local_planner_cmd_ ||
          (ros::Time::now() - last_local_planner_cmd_time_).toSec() >
              local_planner_cmd_timeout_) {
        state.has_path = has_active_path_;
        state.goal_reached = false;
        state.oscillating = false;
        state.blocked = false;
        state.cross_track_error = output.cross_track_error;
        state.heading_error = output.heading_error;
        state.remaining_distance = output.remaining_distance;
        state.commanded_vx = 0.0;
        state.commanded_vy = 0.0;
        state.commanded_wz = 0.0;
        state.tracking_status = 4;
        state.detail = "mpc_cmd_timeout";
        tracking_state_pub_.publish(state);
        publishZero();
        return;
      }

      geometry_msgs::Twist command =
          clampMpcCommand(latest_local_planner_cmd_, output.near_goal);
      const double command_heading =
          std::atan2(command.linear.y, command.linear.x);
      const double command_xy_speed =
          std::hypot(command.linear.x, command.linear.y);
      if (has_scan_ && command_xy_speed > 1e-3 &&
          mpc_obstacle_adapter_.isBlocked(current_scan_, command_heading)) {
        command = geometry_msgs::Twist();
        state.blocked = true;
        state.tracking_status = 5;
        state.detail = "mpc_blocked_by_scan";
      } else {
        command = mpc_velocity_limiter_.limit(command, last_mpc_command_, dt);
      }
      if (output.goal_reached) {
        command = geometry_msgs::Twist();
      }
      last_mpc_command_ = command;

      state.has_path = has_active_path_;
      state.goal_reached = output.goal_reached;
      state.oscillating = output.oscillating;
      state.blocked = output.blocked;
      state.cross_track_error = output.cross_track_error;
      state.heading_error = output.heading_error;
      state.remaining_distance = output.remaining_distance;
      state.commanded_vx = command.linear.x;
      state.commanded_vy = command.linear.y;
      state.commanded_wz = command.angular.z;
      if (state.tracking_status != 5) {
        state.tracking_status = output.tracking_status > 0 ? output.tracking_status : 1;
      }
      if (output.goal_reached) {
        state.detail = "goal_reached";
      } else if (state.detail == "mpc_blocked_by_scan") {
        // Keep explicit blocker detail from final execution safety layer.
      } else if (!latest_local_planner_state_.empty()) {
        state.detail = latest_local_planner_state_;
      } else {
        state.detail = "mpc_dcbf_tracking";
      }

      cmd_pub_.publish(command);
      tracking_state_pub_.publish(state);
      return;
    }

    state.has_path = output.has_path;
    state.goal_reached = output.goal_reached;
    state.oscillating = output.oscillating;
    state.blocked = output.blocked;
    state.cross_track_error = output.cross_track_error;
    state.heading_error = output.heading_error;
    state.remaining_distance = output.remaining_distance;
    state.commanded_vx = output.command.linear.x;
    state.commanded_vy = output.command.linear.y;
    state.commanded_wz = output.command.angular.z;
    state.tracking_status = output.tracking_status;
    state.detail = output.detail;

    cmd_pub_.publish(output.command);
    tracking_state_pub_.publish(state);
  }

  void publishZero() const { cmd_pub_.publish(geometry_msgs::Twist()); }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber local_planner_cmd_sub_;
  ros::Subscriber local_planner_state_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber clear_path_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher tracking_state_pub_;
  ros::Timer control_timer_;

  std::string pose_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string path_topic_;
  std::string clear_path_topic_;
  std::string tracking_state_topic_;
  std::string cmd_vel_topic_;
  std::string local_mode_;
  std::string local_planner_cmd_topic_;
  std::string local_planner_state_topic_;
  std::string enable_button_topic_;
  double control_frequency_;
  double path_timeout_;
  double local_planner_cmd_timeout_;
  bool require_enable_button_;
  int enable_button_index_;
  double enable_release_timeout_;
  std::string enable_release_behavior_;
  double joy_msg_timeout_;
  bool publish_zero_on_block_;

  bool has_pose_;
  bool has_odom_;
  bool has_scan_;
  bool has_joy_;
  bool enable_button_pressed_;
  bool has_seen_enable_press_;
  bool has_active_path_;
  bool has_local_planner_cmd_;
  bool release_active_;
  bool release_timeout_cleared_;
  ros::Time last_path_time_;
  ros::Time last_local_planner_cmd_time_;
  ros::Time last_joy_time_;
  ros::Time release_start_time_;
  std::string latest_local_planner_state_;
  geometry_msgs::Twist latest_local_planner_cmd_;
  geometry_msgs::Twist last_mpc_command_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::Twist current_velocity_;
  sensor_msgs::LaserScan current_scan_;
  PathTrackerController controller_;
  ObstacleAdapter mpc_obstacle_adapter_;
  VelocityLimiter mpc_velocity_limiter_;

  double max_vel_x_;
  double max_vel_y_;
  double max_wz_;
  double near_goal_max_vel_x_;
  double near_goal_max_vel_y_;
  double near_goal_max_wz_;
  double min_cmd_vel_xy_;
  double min_cmd_wz_;
};

} // namespace none_move_base_local

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_controller_node");
  none_move_base_local::LocalControllerNode node;
  ros::spin();
  return 0;
}
