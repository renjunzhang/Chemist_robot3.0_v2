#!/usr/bin/env python3
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rospkg
import rospy
import math
import os

rospack = rospkg.RosPack()
current_pose_file = os.path.expanduser("~/speed_develop/config/current_pose.yaml")


def PoseRecordCallback(pose_msg):

    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y
    z = pose_msg.pose.pose.position.z
    Qx = pose_msg.pose.pose.orientation.x
    Qy = pose_msg.pose.pose.orientation.y
    Qz = pose_msg.pose.pose.orientation.z
    Qw = pose_msg.pose.pose.orientation.w

    # Rx = math.atan2(2 * (Qw * Qx + Qy * Qz), 1 - 2 * (Qx ** 2 + Qy ** 2))
    # Ry = math.asin(2 * (Qw * Qy - Qz * Qx))
    Rz = math.atan2(2 * (Qw * Qz + Qx * Qy), 1 - 2 * (Qy**2 + Qz**2))

    pose_data = {
        "initial_pose_x": x,
        "initial_pose_y": y,
        "initial_pose_a": Rz,
    }

    pose_data = {
        "initial_pose_x": x,
        "initial_pose_y": y,
        "initial_pose_z": z,
        "initial_pose_ox": Qx,
        "initial_pose_oy": Qy,
        "initial_pose_oz": Qz,
        "initial_pose_ow": Qw,
        "initial_pose_a": Rz,
    }

    with open(current_pose_file, "w") as f:
        yaml.dump(pose_data, f)
    f.close()


if __name__ == "__main__":
    rospy.init_node("record_platform_pose_node")

    sub = rospy.Subscriber(
        "/amcl_pose_tf", PoseWithCovarianceStamped, PoseRecordCallback, queue_size=10
    )

    rospy.spin()
