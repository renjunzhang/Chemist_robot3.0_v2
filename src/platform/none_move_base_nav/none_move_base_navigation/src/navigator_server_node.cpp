#include <mutex>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "none_move_base_common/frame_transform.h"
#include "none_move_base_common/pose_utils.h"
#include "none_move_base_common/status_codec.h"
#include "none_move_base_msgs/BehaviorState.h"
#include "none_move_base_msgs/NavigateTaskAction.h"
#include "none_move_base_msgs/NavigationStatus.h"
#include "none_move_base_msgs/PathTrackingState.h"

namespace none_move_base_navigation {

class NavigatorServerNode {
public:
  NavigatorServerNode()
      : private_nh_("~"), action_name_("/none_move_base/navigate_task"),
        action_server_(nh_, action_name_,
                                         boost::bind(&NavigatorServerNode::executeCallback,
                                                     this, _1),
                                         false),
        has_pose_(false) {
    private_nh_.param<std::string>("action_name", action_name_,
                                   "/none_move_base/navigate_task");
    private_nh_.param<std::string>("global_goal_topic", global_goal_topic_,
                                   "/none_move_base/global_goal");
    private_nh_.param<std::string>("global_path_topic", global_path_topic_,
                                   "/none_move_base/global_path");
    private_nh_.param<std::string>("tracking_state_topic", tracking_state_topic_,
                                   "/none_move_base/path_tracking_state");
    private_nh_.param<std::string>("status_topic", status_topic_,
                                   "/none_move_base/status");
    private_nh_.param<std::string>("behavior_state_topic", behavior_state_topic_,
                                   "/none_move_base/behavior_state");
    private_nh_.param<std::string>("clear_path_topic", clear_path_topic_,
                                   "/none_move_base/clear_path");
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/amcl_pose_tf");
    private_nh_.param<std::string>("planner_frame", planner_frame_, "map");
    private_nh_.param("plan_timeout", plan_timeout_, 3.0);
    private_nh_.param("blocked_timeout", blocked_timeout_, 2.0);
    private_nh_.param("action_timeout", action_timeout_, 120.0);
    private_nh_.param("default_xy_goal_tolerance", default_xy_goal_tolerance_, 0.12);
    private_nh_.param("default_yaw_goal_tolerance", default_yaw_goal_tolerance_, 0.17);
    private_nh_.param("default_need_final_yaw", default_need_final_yaw_, true);

    global_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(global_goal_topic_, 1);
    clear_path_pub_ = nh_.advertise<std_msgs::Empty>(clear_path_topic_, 1);
    status_pub_ =
        nh_.advertise<none_move_base_msgs::NavigationStatus>(status_topic_, 1, true);
    behavior_state_pub_ =
        nh_.advertise<none_move_base_msgs::BehaviorState>(behavior_state_topic_, 1, true);

    pose_sub_ =
        nh_.subscribe(pose_topic_, 1, &NavigatorServerNode::poseCallback, this);
    path_sub_ = nh_.subscribe(global_path_topic_, 1,
                              &NavigatorServerNode::pathCallback, this);
    tracking_state_sub_ = nh_.subscribe(
        tracking_state_topic_, 1, &NavigatorServerNode::trackingStateCallback, this);

    action_server_.start();
  }

private:
  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pose_ = none_move_base_common::toPoseStamped(*msg);
    has_pose_ = true;
  }

  void pathCallback(const nav_msgs::PathConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_path_ = *msg;
    latest_path_time_ = ros::Time::now();
  }

  void trackingStateCallback(
      const none_move_base_msgs::PathTrackingStateConstPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_tracking_state_ = *msg;
    latest_tracking_time_ = ros::Time::now();
  }

  void executeCallback(
      const none_move_base_msgs::NavigateTaskGoalConstPtr &goal_msg) {
    none_move_base_msgs::NavigateTaskResult result;
    none_move_base_msgs::NavigateTaskFeedback feedback;

    geometry_msgs::PoseStamped current_pose;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!has_pose_) {
        result.success = false;
        result.error_code = 3;
        result.state = "error";
        result.detail = "no_localization";
        action_server_.setAborted(result, result.detail);
        return;
      }
      current_pose = current_pose_;
    }

    if (goal_msg->use_named_station) {
      result.success = false;
      result.error_code = 2;
      result.state = "error";
      result.detail = "named_station_not_supported_in_phase1";
      action_server_.setAborted(result, result.detail);
      return;
    }

    geometry_msgs::PoseStamped target_pose = goal_msg->target_pose;
    if (target_pose.header.frame_id.empty()) {
      target_pose.header.frame_id = planner_frame_;
    }
    if (target_pose.header.frame_id != planner_frame_) {
      result.success = false;
      result.error_code = 1;
      result.state = "error";
      result.detail = "target_pose_frame_mismatch";
      action_server_.setAborted(result, result.detail);
      return;
    }

    const ros::Time goal_start_time = ros::Time::now();
    clearCurrentPath();
    resetRuntimeState();
    publishBehaviorState("move", "idle", true, false, false, 0, "goal_accepted");
    publishStatus(*goal_msg, current_pose, target_pose, "running", "planning",
                  "goal_accepted", 0.0, 0.0, true, false, 0);
    global_goal_pub_.publish(target_pose);

    ros::Rate loop_rate(10.0);
    ros::Time blocked_since;
    bool blocked_active = false;

    while (ros::ok()) {
      if (action_server_.isPreemptRequested()) {
        clearCurrentPath();
        publishBehaviorState("idle", "move", false, false, false, 7, "canceled");
        publishStatus(*goal_msg, currentPoseSnapshot(), target_pose, "idle", "error",
                      "canceled", 0.0, 0.0, false, false, 7);
        result.success = false;
        result.error_code = 7;
        result.state = "canceled";
        result.detail = "canceled";
        action_server_.setPreempted(result, result.detail);
        return;
      }

      const auto path_snapshot = pathSnapshot();
      const auto tracking_snapshot = trackingSnapshot();
      const auto pose_snapshot = currentPoseSnapshot();
      const double elapsed = (ros::Time::now() - goal_start_time).toSec();

      std::string phase = "planning";
      if (tracking_snapshot.header.stamp.isValid()) {
        phase = none_move_base_common::trackingStatusToPhase(
            tracking_snapshot.tracking_status);
      } else if (!path_snapshot.poses.empty()) {
        phase = "tracking";
      }

      feedback.phase = phase;
      feedback.detail = tracking_snapshot.detail.empty() ? "running" : tracking_snapshot.detail;
      feedback.distance_to_goal = tracking_snapshot.remaining_distance;
      feedback.yaw_error = tracking_snapshot.heading_error;
      feedback.current_station = "";
      action_server_.publishFeedback(feedback);

      publishStatus(*goal_msg, pose_snapshot, target_pose, "running", phase,
                    feedback.detail, feedback.distance_to_goal, feedback.yaw_error,
                    true, false, 0);

      if (!path_snapshot.poses.empty() &&
          tracking_snapshot.header.stamp.isValid() &&
          tracking_snapshot.goal_reached) {
        clearCurrentPath();
        publishBehaviorState("idle", "move", false, true, false, 0, "goal_reached");
        publishStatus(*goal_msg, pose_snapshot, target_pose, "done", "done",
                      "goal_reached", tracking_snapshot.remaining_distance,
                      tracking_snapshot.heading_error, false, false, 0);
        result.success = true;
        result.error_code = 0;
        result.state = "done";
        result.detail = "goal_reached";
        result.current_station = "";
        action_server_.setSucceeded(result, result.detail);
        return;
      }

      if (!path_snapshot.poses.empty() && tracking_snapshot.blocked) {
        if (!blocked_active) {
          blocked_active = true;
          blocked_since = ros::Time::now();
        }
        if ((ros::Time::now() - blocked_since).toSec() > blocked_timeout_) {
          clearCurrentPath();
          publishBehaviorState("idle", "move", false, false, true, 5,
                               "blocked_timeout");
          publishStatus(*goal_msg, pose_snapshot, target_pose, "error", "error",
                        "blocked_timeout", tracking_snapshot.remaining_distance,
                        tracking_snapshot.heading_error, false, false, 5);
          result.success = false;
          result.error_code = 5;
          result.state = "error";
          result.detail = "blocked_timeout";
          action_server_.setAborted(result, result.detail);
          return;
        }
      } else {
        blocked_active = false;
      }

      if (elapsed > action_timeout_) {
        clearCurrentPath();
        publishBehaviorState("idle", "move", false, false, true, 6, "action_timeout");
        publishStatus(*goal_msg, pose_snapshot, target_pose, "error", "error",
                      "action_timeout", tracking_snapshot.remaining_distance,
                      tracking_snapshot.heading_error, false, false, 6);
        result.success = false;
        result.error_code = 6;
        result.state = "error";
        result.detail = "action_timeout";
        action_server_.setAborted(result, result.detail);
        return;
      }

      if (path_snapshot.poses.empty() && elapsed > plan_timeout_) {
        clearCurrentPath();
        publishBehaviorState("idle", "move", false, false, true, 4,
                             "global_plan_failed");
        publishStatus(*goal_msg, pose_snapshot, target_pose, "error", "error",
                      "global_plan_failed", 0.0, 0.0, false, false, 4);
        result.success = false;
        result.error_code = 4;
        result.state = "error";
        result.detail = "global_plan_failed";
        action_server_.setAborted(result, result.detail);
        return;
      }

      loop_rate.sleep();
    }

    clearCurrentPath();
    result.success = false;
    result.error_code = 6;
    result.state = "error";
    result.detail = "ros_shutdown";
    action_server_.setAborted(result, result.detail);
  }

  void resetRuntimeState() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_path_ = nav_msgs::Path();
    latest_tracking_state_ = none_move_base_msgs::PathTrackingState();
    latest_path_time_ = ros::Time(0);
    latest_tracking_time_ = ros::Time(0);
  }

  void clearCurrentPath() {
    std_msgs::Empty empty;
    clear_path_pub_.publish(empty);
  }

  geometry_msgs::PoseStamped currentPoseSnapshot() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_pose_;
  }

  nav_msgs::Path pathSnapshot() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_path_;
  }

  none_move_base_msgs::PathTrackingState trackingSnapshot() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_tracking_state_;
  }

  void publishStatus(const none_move_base_msgs::NavigateTaskGoal &goal,
                     const geometry_msgs::PoseStamped &current_pose,
                     const geometry_msgs::PoseStamped &target_pose,
                     const std::string &state, const std::string &phase,
                     const std::string &detail, double distance_to_goal,
                     double yaw_error, bool is_tracking, bool is_charging,
                     int error_code) {
    none_move_base_msgs::NavigationStatus status;
    status.header.stamp = ros::Time::now();
    status.task_id = goal.id;
    status.exper_no = goal.exper_no;
    status.action = goal.action;
    status.state = state;
    status.phase = phase;
    status.detail = detail;
    status.current_station = "";
    status.current_pose = current_pose;
    status.target_pose = target_pose;
    status.distance_to_goal = distance_to_goal;
    status.yaw_error = yaw_error;
    status.is_tracking = is_tracking;
    status.is_charging = is_charging;
    status.error_code = error_code;
    status_pub_.publish(status);
  }

  void publishBehaviorState(const std::string &active_behavior,
                            const std::string &previous_behavior,
                            bool running, bool done, bool failed, int error_code,
                            const std::string &detail) {
    none_move_base_msgs::BehaviorState state;
    state.header.stamp = ros::Time::now();
    state.active_behavior = active_behavior;
    state.previous_behavior = previous_behavior;
    state.behavior_running = running;
    state.behavior_done = done;
    state.behavior_failed = failed;
    state.error_code = error_code;
    state.detail = detail;
    behavior_state_pub_.publish(state);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string action_name_;
  std::string global_goal_topic_;
  std::string global_path_topic_;
  std::string tracking_state_topic_;
  std::string status_topic_;
  std::string behavior_state_topic_;
  std::string clear_path_topic_;
  std::string pose_topic_;
  std::string planner_frame_;
  double plan_timeout_;
  double blocked_timeout_;
  double action_timeout_;
  double default_xy_goal_tolerance_;
  double default_yaw_goal_tolerance_;
  bool default_need_final_yaw_;

  actionlib::SimpleActionServer<none_move_base_msgs::NavigateTaskAction>
      action_server_;
  ros::Publisher global_goal_pub_;
  ros::Publisher clear_path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher behavior_state_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber tracking_state_sub_;

  std::mutex data_mutex_;
  bool has_pose_;
  geometry_msgs::PoseStamped current_pose_;
  nav_msgs::Path latest_path_;
  none_move_base_msgs::PathTrackingState latest_tracking_state_;
  ros::Time latest_path_time_;
  ros::Time latest_tracking_time_;
};

} // namespace none_move_base_navigation

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigator_server_node");
  none_move_base_navigation::NavigatorServerNode node;
  ros::spin();
  return 0;
}
