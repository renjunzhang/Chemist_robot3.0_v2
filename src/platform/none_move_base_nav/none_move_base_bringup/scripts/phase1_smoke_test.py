#!/usr/bin/env python3

import math
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

from none_move_base_msgs.msg import NavigateTaskAction, NavigateTaskGoal


class Phase1SmokeTest:
    def __init__(self):
        self.last_cmd = Twist()
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher(
            "/amcl_pose_tf", PoseWithCovarianceStamped, queue_size=1
        )
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.scan_pub = rospy.Publisher("/scan_full_filtered", LaserScan, queue_size=1)
        rospy.Subscriber("/hf_platform/nav_vel", Twist, self.cmd_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient(
            "/none_move_base/navigate_task", NavigateTaskAction
        )

    def cmd_callback(self, msg):
        self.last_cmd = msg

    @staticmethod
    def quaternion_from_yaw(yaw):
        return quaternion_from_euler(0.0, 0.0, yaw)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def make_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info = MapMetaData()
        grid.info.resolution = 0.05
        grid.info.width = 120
        grid.info.height = 120
        grid.info.origin.position.x = -2.0
        grid.info.origin.position.y = -2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        return grid

    def publish_pose(self, x, y, yaw, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = self.quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.pose_pub.publish(msg)

    def publish_odom(self, vx, vy, wz, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)

    def publish_scan(self, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser_link"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.range_min = 0.05
        msg.range_max = 10.0
        count = int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        msg.ranges = [8.0] * count
        self.scan_pub.publish(msg)

    def publish_static_inputs(self, grid, x, y, yaw, vx, vy, wz):
        stamp = rospy.Time.now()
        grid.header.stamp = stamp
        self.map_pub.publish(grid)
        self.publish_pose(x, y, yaw, stamp)
        self.publish_odom(vx, vy, wz, stamp)
        self.publish_scan(stamp)

    def build_goal(self):
        goal = NavigateTaskGoal()
        goal.id = "phase1_smoke_test"
        goal.action = "move"
        goal.use_named_station = False
        goal.need_final_yaw = True
        goal.enable_terminal_adjust = False
        goal.xy_goal_tolerance = 0.12
        goal.yaw_goal_tolerance = 0.17
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def run(self):
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("navigate_task action server not available")

        grid = self.make_map()
        for _ in range(10):
            self.publish_static_inputs(grid, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            rospy.sleep(0.05)

        self.client.send_goal(self.build_goal())

        x = 0.0
        y = 0.0
        yaw = 0.0
        last_time = time.time()
        start_time = last_time
        last_print = start_time
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown() and time.time() - start_time < 15.0:
            now_wall = time.time()
            dt = max(1e-3, now_wall - last_time)
            last_time = now_wall

            vx = self.last_cmd.linear.x
            vy = self.last_cmd.linear.y
            wz = self.last_cmd.angular.z

            x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
            y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
            yaw = self.normalize_angle(yaw + wz * dt)

            self.publish_static_inputs(grid, x, y, yaw, vx, vy, wz)

            if now_wall - last_print >= 1.0:
                rospy.loginfo(
                    "phase1_smoke_test t=%.1fs pose=(%.3f, %.3f, %.3f) "
                    "cmd=(%.3f, %.3f, %.3f) state=%d",
                    now_wall - start_time,
                    x,
                    y,
                    yaw,
                    vx,
                    vy,
                    wz,
                    self.client.get_state(),
                )
                last_print = now_wall

            state = self.client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.PREEMPTED,
                GoalStatus.REJECTED,
            ):
                break
            rate.sleep()

        result = self.client.get_result()
        rospy.loginfo("phase1_smoke_test final_state=%d", self.client.get_state())
        rospy.loginfo(
            "phase1_smoke_test final_pose=(%.3f, %.3f, %.3f)", x, y, yaw
        )
        rospy.loginfo(
            "phase1_smoke_test final_cmd=(%.3f, %.3f, %.3f)",
            self.last_cmd.linear.x,
            self.last_cmd.linear.y,
            self.last_cmd.angular.z,
        )
        rospy.loginfo("phase1_smoke_test final_result=%s", result)

        if self.client.get_state() != GoalStatus.SUCCEEDED:
            raise SystemExit(2)


if __name__ == "__main__":
    rospy.init_node("phase1_smoke_test", anonymous=True)
    Phase1SmokeTest().run()
