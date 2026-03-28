#!/usr/bin/env python3

import argparse
import math
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from none_move_base_msgs.msg import NavigateTaskAction, NavigateTaskGoal


def yaw_from_pose(msg):
    q = msg.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def main():
    parser = argparse.ArgumentParser(description="Send a relative NavigateTask goal.")
    parser.add_argument("--dx", type=float, default=0.3, help="Forward offset in base frame, meters.")
    parser.add_argument("--dy", type=float, default=0.0, help="Lateral offset in base frame, meters.")
    parser.add_argument("--dyaw", type=float, default=0.0, help="Yaw offset, radians.")
    parser.add_argument("--id", type=str, default="", help="Task id.")
    parser.add_argument("--action", type=str, default="move", help="Task action name.")
    parser.add_argument("--need-final-yaw", action="store_true", help="Require final yaw alignment.")
    parser.add_argument("--xy-goal-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-goal-tolerance", type=float, default=0.17)
    parser.add_argument("--pose-topic", type=str, default="/amcl_pose_tf")
    parser.add_argument("--action-name", type=str, default="/none_move_base/navigate_task")
    args = parser.parse_args()

    rospy.init_node("send_relative_goal", anonymous=True)

    rospy.loginfo("waiting for current pose on %s", args.pose_topic)
    current_pose = rospy.wait_for_message(
        args.pose_topic, PoseWithCovarianceStamped, timeout=5.0
    )

    current_yaw = yaw_from_pose(current_pose)
    target_x = (
        current_pose.pose.pose.position.x
        + math.cos(current_yaw) * args.dx
        - math.sin(current_yaw) * args.dy
    )
    target_y = (
        current_pose.pose.pose.position.y
        + math.sin(current_yaw) * args.dx
        + math.cos(current_yaw) * args.dy
    )
    target_yaw = current_yaw + args.dyaw

    goal = NavigateTaskGoal()
    goal.id = args.id or f"relative_goal_{int(time.time())}"
    goal.action = args.action
    goal.use_named_station = False
    goal.need_final_yaw = args.need_final_yaw
    goal.enable_terminal_adjust = False
    goal.xy_goal_tolerance = args.xy_goal_tolerance
    goal.yaw_goal_tolerance = args.yaw_goal_tolerance
    goal.target_pose = PoseStamped()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    q = quaternion_from_euler(0.0, 0.0, target_yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    client = actionlib.SimpleActionClient(args.action_name, NavigateTaskAction)
    rospy.loginfo("waiting for action server %s", args.action_name)
    if not client.wait_for_server(rospy.Duration(5.0)):
        raise RuntimeError("navigate_task action server not available")

    rospy.loginfo(
        "sending goal id=%s target=(%.3f, %.3f, %.3f) need_final_yaw=%s",
        goal.id,
        target_x,
        target_y,
        target_yaw,
        goal.need_final_yaw,
    )
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("result state=%d result=%s", client.get_state(), client.get_result())


if __name__ == "__main__":
    main()
