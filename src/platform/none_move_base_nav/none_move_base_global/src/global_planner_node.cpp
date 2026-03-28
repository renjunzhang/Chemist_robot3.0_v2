#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "none_move_base_common/frame_transform.h"
#include "none_move_base_global/astar_planner.h"
#include "none_move_base_global/grid_adapter.h"
#include "none_move_base_global/path_postprocessor.h"

namespace none_move_base_global {

class GlobalPlannerNode {
public:
  GlobalPlannerNode() : private_nh_("~"), has_pose_(false) {
    private_nh_.param<std::string>("planner_frame", planner_frame_, "map");
    private_nh_.param<std::string>("map_topic", map_topic_, "/map");
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/amcl_pose_tf");
    private_nh_.param<std::string>("goal_topic", goal_topic_,
                                   "/none_move_base/global_goal");
    private_nh_.param<std::string>("path_topic", path_topic_,
                                   "/none_move_base/global_path");
    private_nh_.param("occupied_threshold", occupied_threshold_, 50);
    private_nh_.param("allow_unknown", allow_unknown_, false);
    private_nh_.param("inflation_radius", inflation_radius_, 0.20);
    private_nh_.param("prune_collinear", prune_collinear_, true);
    private_nh_.param("line_of_sight_prune", line_of_sight_prune_, true);
    private_nh_.param("resample_step", resample_step_, 0.08);

    map_sub_ = nh_.subscribe(map_topic_, 1, &GlobalPlannerNode::mapCallback, this);
    pose_sub_ =
        nh_.subscribe(pose_topic_, 1, &GlobalPlannerNode::poseCallback, this);
    goal_sub_ =
        nh_.subscribe(goal_topic_, 1, &GlobalPlannerNode::goalCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  }

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {
    grid_adapter_.setMap(*msg, occupied_threshold_, inflation_radius_,
                         allow_unknown_);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    current_pose_ = none_move_base_common::toPoseStamped(*msg);
    has_pose_ = true;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    geometry_msgs::PoseStamped goal = *msg;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = planner_frame_;
    }

    if (!grid_adapter_.hasMap() || !has_pose_) {
      ROS_WARN("Global planner is missing map or current pose.");
      publishEmptyPath();
      return;
    }

    if (goal.header.frame_id != planner_frame_ ||
        current_pose_.header.frame_id != planner_frame_) {
      ROS_WARN("Global planner only supports goals in frame %s for phase 1.",
               planner_frame_.c_str());
      publishEmptyPath();
      return;
    }

    GridCell start_cell;
    GridCell goal_cell;
    if (!grid_adapter_.worldToGrid(current_pose_.pose.position.x,
                                   current_pose_.pose.position.y, &start_cell.x,
                                   &start_cell.y) ||
        !grid_adapter_.worldToGrid(goal.pose.position.x, goal.pose.position.y,
                                   &goal_cell.x, &goal_cell.y)) {
      ROS_WARN("Global planner start or goal is outside the occupancy grid.");
      publishEmptyPath();
      return;
    }

    std::vector<GridCell> raw_path;
    if (!planner_.plan(grid_adapter_, start_cell, goal_cell, &raw_path)) {
      ROS_WARN("A* failed to generate a valid path.");
      publishEmptyPath();
      return;
    }

    std::vector<GridCell> processed = raw_path;
    if (prune_collinear_) {
      processed = postprocessor_.removeCollinear(processed);
    }
    if (line_of_sight_prune_) {
      processed = postprocessor_.pruneLineOfSight(processed, grid_adapter_);
    }

    nav_msgs::Path path = postprocessor_.buildPath(
        processed, grid_adapter_, current_pose_, goal, planner_frame_, ros::Time::now(),
        resample_step_);
    path_pub_.publish(path);
  }

  void publishEmptyPath() {
    nav_msgs::Path path;
    path.header.frame_id = planner_frame_;
    path.header.stamp = ros::Time::now();
    path_pub_.publish(path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

  std::string planner_frame_;
  std::string map_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  int occupied_threshold_;
  bool allow_unknown_;
  double inflation_radius_;
  bool prune_collinear_;
  bool line_of_sight_prune_;
  double resample_step_;

  bool has_pose_;
  geometry_msgs::PoseStamped current_pose_;
  GridAdapter grid_adapter_;
  AStarPlanner planner_;
  PathPostprocessor postprocessor_;
};

} // namespace none_move_base_global

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_node");
  none_move_base_global::GlobalPlannerNode node;
  ros::spin();
  return 0;
}
