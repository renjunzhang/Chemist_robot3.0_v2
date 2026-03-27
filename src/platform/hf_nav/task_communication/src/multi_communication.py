#!/usr/bin/env python3
import argparse
import json
import threading
import sys
import math
import copy
import requests
import os
from collections import defaultdict
from pprint import pprint
import paho.mqtt.client as mqtt

import numpy as np
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation

import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf_conversions
import tf2_ros
import tf2_geometry_msgs

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool, Int8, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Quaternion

from aichem_msg_srv.srv import DmsService, DmsServiceResponse

from communication_rs485.msg import platformInfo

# 如果是起始样品架出发，横移多一点
# 如果是角落里的液体进样或吸液工作站或休息工作站出发，x和y方向都需要横移
stroll_offsets = {
    "starting_station": (0, 0.35),
    "liquid_dispensing_1": (-0.3, 0.25),
    "imbibition_workstation": (-0.3, 0.25),
    "ultrasonic_cleaner": (-0.3, 0.25),
    "multi_robots_exchange_workstation": (-0.3, 0.3),
    "charge_station_hf_0": (0, 0.25),
    "charge_station_hf_1": (0.5, 0),
}

conflict_distence = 5.0
max_wait_try_times = 5

mb_action_status_dict = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


def load_station_pose_json(file_json_name):
    try:
        with open(os.path.expanduser(file_json_name), "r") as file:
            data = json.load(file)
    except FileNotFoundError:
        print(f"Error: Opening {file_json_name} failed")
        return None

    station_pose = {}
    for item in data:
        pose = item.get("pose", {})
        if "ox" in pose:
            station_pose[item["name"]] = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose["z"]),
                "ox": float(pose["ox"]),
                "oy": float(pose["oy"]),
                "oz": float(pose["oz"]),
                "ow": float(pose["ow"]),
            }
        else:
            station_pose[item["name"]] = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose["z"]),
                "w": float(pose["w"]),
            }

    return station_pose


def vector_to_eigen_pose(vector_pose):
    # Identity transformation matrix
    eigen_pose = np.eye(4)

    # Quaternion elements (order: w, x, y, z)
    quat = vector_pose["ox"], vector_pose["oy"], vector_pose["oz"], vector_pose["ow"]
    # Normalize quaternion
    quat /= np.linalg.norm(quat)

    # Translation vector (x, y, z)
    t = vector_pose["x"], vector_pose["y"], vector_pose["z"]

    # Create rotation matrix from quaternion
    rot_matrix = Rotation.from_quat(quat).as_matrix()

    # Populate transformation matrix
    eigen_pose[:3, :3] = rot_matrix
    eigen_pose[:3, 3] = t

    return eigen_pose


def coordinate_transform(camera_pose, refer_camera_pose, scale):

    move_pose = np.dot(camera_pose, np.linalg.inv(refer_camera_pose))

    pose_transform = PoseStamped()
    pose_transform.header.frame_id = "hf_base_link"
    pose_transform.header.stamp = rospy.Time(0)

    pose_transform.pose.position.x = move_pose[0, 3] * scale
    pose_transform.pose.position.y = move_pose[1, 3] * scale
    orientation = tf_conversions.transformations.quaternion_from_matrix(move_pose)
    pose_transform.pose.orientation.x = orientation[0]
    pose_transform.pose.orientation.y = orientation[1]
    pose_transform.pose.orientation.z = orientation[2]
    pose_transform.pose.orientation.w = orientation[3]

    pose_transform_map = PoseStamped()
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    transform_flag = False
    while not transform_flag:
        try:
            pose_transform_map = buffer.transform(
                pose_transform, "map", rospy.Duration(1)
            )
            transform_flag = True
        except tf2_ros.TransformException as e:
            rospy.loginfo("wait tf buffer listener.....")

    # rospy.loginfo(
    #     "mappose_tranform.pose.position.x: " + str(pose_transform_map.pose.position.x)
    # )
    # rospy.loginfo(
    #     "mappose_tranform.pose.position.y: " + str(pose_transform_map.pose.position.y)
    # )
    # rospy.loginfo(
    #     "mappose_tranform.pose.position.z: " + str(pose_transform_map.pose.position.z)
    # )

    return pose_transform_map


class PlatformCommunication:
    def __init__(
        self,
        host_ip,
        robot_name,
        pose_topic,
        station_pose_path,
        camera_pose_path,
        aruco_id,
        use_prefix=False,
    ):
        # launch文件的传入参数
        self.station_pose_path = station_pose_path
        self.camera_pose_path = camera_pose_path
        self.aruco_id = int(aruco_id)

        # 主机ip
        self.host_ip = "http://" + host_ip + ":5000"
        
        # mqtt ip
        self.mqtt_address = host_ip
        # self.mqtt_address = "127.0.0.1"
        self.mqtt_port = 1883
        self.mqtt_client = mqtt.Client("robot_client")
        self.mqtt_client.on_connect = self.mqtt_on_connect
        self.mqtt_client.on_message = self.mqtt_on_message
        self.mqtt_client.connect(self.mqtt_address, self.mqtt_port)
        self.mqtt_client.loop_start()

        # 机器人名称
        self.robot_prefix = robot_name if use_prefix else ""
        self.base_link_frame = (
            robot_name + "/base_link" if use_prefix else "hf_base_link"
        )

        self.robot_name = robot_name
        self.pose_topic = pose_topic

        # 充电站名称
        self.charge_station = "charge_station_" + self.robot_name

        # 连接flask服务器
        self.session = requests.Session()
        self.session.cookies.set("sender", self.robot_name)

        # 底盘当前状态
        self.platform_status = defaultdict()
        self.platform_status["id"] = ""
        self.platform_status["exper_no"] = ""
        self.platform_status["stamp"] = rospy.Time.now().to_time()
        self.platform_status["state"] = "idle"
        self.platform_status["detail"] = ""
        self.platform_status["cur_station"] = "charge_station"

        self.global_status = defaultdict(dict)
        self.pub_global_status_msg = rospy.Publisher(
            self.robot_prefix + "/global_robots_status", String, queue_size=2
        )
        self.init_flag = False

        # 机械臂复位指令
        self.oper_reset_msg = {
            "id": "",
            "destination": "",
            "operations": [{"operation": "reset"}],
        }
        self.oper_pub = rospy.Publisher(
            self.robot_prefix + "/obsOperation_in", String, queue_size=10
        )

        # 当前位姿信息
        self.cur_location = PoseWithCovarianceStamped()
        self.cur_yaw = 0
        # 当前目标站点信息
        self.cur_station_name = ""
        # 上一个站点信息(从哪个站点出发)
        # 根据从哪个站点出发，需要的起始横移不同
        self.last_station_name = "map_center"

        # move_base client
        self.ac = actionlib.SimpleActionClient(
            self.robot_prefix + "/move_base", MoveBaseAction
        )
        # 当前move_base goal
        self.cur_goal = MoveBaseGoal()

        # 订阅话题
        # 订阅 task_manager 的目标站指令
        self.sub_dms_cmd = rospy.Subscriber(
            self.robot_prefix + "/obsNavigation_in",
            String,
            self.dms_callback,
            buff_size=10,
        )
        # 订阅 amcl 的当前位姿
        self.sub_amcl_pose = rospy.Subscriber(
            self.robot_prefix + self.pose_topic,
            PoseWithCovarianceStamped,
            self.amcl_pose_callback,
            queue_size=1,
        )

        # 发布话题
        # 发送给 task_manager , 到达目标站的反馈
        self.pub_dms_msg = rospy.Publisher(
            self.robot_prefix + "/obsNavigation_out", String, queue_size=10
        )
        # 语音播报
        self.pub_voice_broadcast = rospy.Publisher(
            self.robot_prefix + "/voide_broadcast_msg", String, queue_size=2
        )
        # dms的实时查询服务
        self.dms_inquiry_server_ = rospy.Service(
            self.robot_prefix + "/inquiryNavigation_out",
            DmsService,
            self.dms_inquiry_service,
        )
        # 记录amcl_pose获取时间
        self.amcl_pose_time = rospy.Time.now()

        # 全部站点坐标文件
        self.station_pose = load_station_pose_json(self.station_pose_path)
        # 相机-充电站坐标文件
        self.refer_camera_pose = load_station_pose_json(self.camera_pose_path)
        if self.station_pose is None or self.refer_camera_pose is None:
            exit(-1)

        # 表示当前接收到了新目标点，正在移动状态
        self.state_move = False
        self.state_move_lock = threading.Lock()

        self.last_action = ""
        # 是否在躲避状态
        self.avoid = False

        # 充电状态
        self.recharge_flag = False
        self.recharge_lock = threading.Lock()

        self.loop_rate = rospy.Rate(10)

        # 这个线程专门接收mapf后端发来的move_base.goal消息
        self.thread1 = threading.Thread(target=self.run_to_goal)
        self.thread1.start()

        # 这个线程用来接收去rest_station的消息
        self.thread2 = threading.Thread(target=self.run_to_rest)
        self.thread2.start()

    def get_camera_tran(self, scale):
        get_camera_pose_publish = rospy.Publisher(
            "/camera_operation", Int32, queue_size=1
        )
        op_code = Int32()
        op_code.data = self.aruco_id
        camera_pose = None

        camera_try_count = 0
        while True:
            while get_camera_pose_publish.get_num_connections() == 0:
                self.loop_rate.sleep()
            get_camera_pose_publish.publish(op_code)
            camera_pose = rospy.wait_for_message("/point_coordinate", PoseStamped)
            camera_try_count += 1
            if camera_try_count > 5:
                # self.platform_status["state"] = "error"
                self.platform_status["state"] = "idle"
                self.platform_status["detail"] = "充电未检测到二维码"
                pub_msg = String()
                pub_msg.data = json.dumps(self.platform_status)
                self.loop_rate.sleep()
                self.pub_dms_msg.publish(pub_msg)
                self.platform_status["state"] = "idle"
                return False, None
            if abs(camera_pose.pose.position.z) >= 1e-5:
                break

        T_baselink_cameracoloframe = vector_to_eigen_pose(
            self.refer_camera_pose["baselink_cameracoloframe"]
        )
        refer_marker_pose_for_recharge = vector_to_eigen_pose(
            self.refer_camera_pose["充电站相机"]
        )

        # 获取位置信息
        position = np.array(
            [
                camera_pose.pose.position.x,
                camera_pose.pose.position.y,
                camera_pose.pose.position.z,
            ]
        )

        # 获取四元数姿态信息并转换为旋转矩阵
        quaternion = [
            camera_pose.pose.orientation.x,
            camera_pose.pose.orientation.y,
            camera_pose.pose.orientation.z,
            camera_pose.pose.orientation.w,
        ]
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

        # 构建相机位姿矩阵
        Tw_camera_pose = np.eye(4)
        Tw_camera_pose[:3, :3] = rotation_matrix
        Tw_camera_pose[:3, 3] = position
        eigen_marker_pose = np.dot(T_baselink_cameracoloframe, Tw_camera_pose)

        trans_map = coordinate_transform(
            eigen_marker_pose, refer_marker_pose_for_recharge, scale
        )

        return True, trans_map

    # 实时储存位姿, 并发送给flask服务器
    def amcl_pose_callback(self, pos):
        # 实时储存位姿
        self.cur_location = pos
        (_, _, self.cur_yaw) = euler_from_quaternion(
            (
                pos.pose.pose.orientation.x,
                pos.pose.pose.orientation.y,
                pos.pose.pose.orientation.z,
                pos.pose.pose.orientation.w,
            )
        )
        self.amcl_pose_time = rospy.Time.now()

        # 发送给flask服务器
        pose = pos.pose.pose
        # 组装 pose 信息
        pose_dict = {
            "robot_name": self.robot_name,
            "status": self.platform_status["state"],
            "stamp": self.amcl_pose_time.to_sec(),
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }

        try:
            self.mqtt_client.publish("/mapf_pose", json.dumps(pose_dict), qos=1, retain=True)
        except:
            rospy.logwarn("FUNC callback: 无法连接MQTT服务器")
            # rospy.loginfo(f"Rcv msg from flask_core: {response.text}")

    # 将当前机器人目标站位置发送给flask服务器
    def dms_callback(self, msg):
        # 接收到第一帧指令时，初始化成功
        self.init_flag = True

        cmd = json.loads(msg.data)

        pprint(cmd)
        rospy.loginfo(
            f"\033[1;32m做任何操作前机械臂复位\033[0m, 复位工作站{self.last_station_name}"
        )
        if "charge_station" in self.last_station_name:
            self.oper_reset_msg["destination"] = "charge_station"
        else:
            self.oper_reset_msg["destination"] = self.last_station_name
        self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))

        # 获取JSON中的值
        action = cmd["action"]
        dest_station = cmd["destination"]

        self.last_action = action

        # 更新platform_status_
        self.platform_status["id"] = cmd["id"]

        if action == "move" or action == "charge":
            rospy.loginfo(f"Action = \033[1;32m{action}\033[0m")

            if action == "charge":
                dest_station = self.charge_station

            if action == "charge" and self.recharge_flag:
                self.platform_status["state"] = "done"
                self.platform_status["detail"] = "已在充电中"
                pub_msg = String()
                pub_msg.data = json.dumps(self.platform_status)
                self.loop_rate.sleep()
                self.pub_dms_msg.publish(pub_msg)
                self.platform_status["state"] = "charging"
                return
            if action == "move" and self.recharge_flag:
                self.platform_status["state"] = "idle"
                self.platform_status["detail"] = "移动前解除充电"
                pub_msg = String()
                pub_msg.data = json.dumps(self.platform_status)
                self.loop_rate.sleep()
                self.pub_dms_msg.publish(pub_msg)
                self.platform_status["state"] = "charging"
                return

            rospy.loginfo(
                "\033[0;33m等待2s让避让状态更新\033[0m"
            )
            rospy.sleep(2.0)

            # 如果在避让状态，首先先完成避让动作，先到充电站,再执行下一步指令
            while self.avoid == True:
                rospy.sleep(1.0)
                rospy.loginfo(
                    "\033[0;33m当前机器人在避让状态(去充电站), 先等待避让完成\033[0m"
                )

            # 针对hf_0
            # robot_0准备移动时，如果robot_0在充电站而robot_1在running状态
            # robot_0必须等robot_1在idle状态才能动
            if (
                self.robot_name == "hf_0"
                and self.cur_station_name == self.charge_station
            ):
                try_times = 0
                while (
                    not self.global_status.get("hf_1")
                    or abs(
                        self.global_status["hf_1"]["stamp"]
                        - self.global_status["hf_0"]["stamp"]
                    )
                    > 1.0
                    or abs(
                        rospy.Time.now().to_time() - self.global_status["hf_0"]["stamp"]
                    )
                    > 2.0
                ) and try_times < max_wait_try_times:
                    try_times += 1
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"等待两台机器人状态数据同步更新，尝试次数{try_times}/{max_wait_try_times}"
                    )
                while self.global_status["hf_1"]["status"] == "running":
                    rospy.sleep(1.0)
                    rospy.loginfo("当前机器人在充电站, 等待hf_1停止运动")

            # 针对两台机器人，如果距离过近，先等另一台机器人先动
            if self.robot_name == "hf_0":
                try_times = 0
                while (
                    not self.global_status.get("hf_1")
                    or abs(
                        self.global_status["hf_1"]["stamp"]
                        - self.global_status["hf_0"]["stamp"]
                    )
                    > 1.0
                    or abs(
                        rospy.Time.now().to_time() - self.global_status["hf_0"]["stamp"]
                    )
                    > 2.0
                ) and try_times < max_wait_try_times:
                    try_times += 1
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"等待两台机器人状态数据同步更新，尝试次数{try_times}/{max_wait_try_times}"
                    )
                rospy.loginfo(
                    f'机器人距离: {self.disdance:.2f}m, hf_1状态: {self.global_status["hf_1"]["status"]}'
                )
                while (
                    self.global_status["hf_1"]["status"] == "running"
                    and self.disdance < conflict_distence
                ):
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"机器人距离: {self.disdance:.2f}<{conflict_distence}m, 等待hf_1停止运动"
                    )
            elif self.robot_name == "hf_1":
                rospy.sleep(2.0)  # 设置时间差，防止状态更新不及时

                try_times = 0
                while (
                    not self.global_status.get("hf_0")
                    or abs(
                        self.global_status["hf_1"]["stamp"]
                        - self.global_status["hf_0"]["stamp"]
                    )
                    > 1.0
                    or abs(
                        rospy.Time.now().to_time() - self.global_status["hf_1"]["stamp"]
                    )
                    > 2.0
                ) and try_times < max_wait_try_times:
                    try_times += 1
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"等待两台机器人状态数据同步更新，尝试次数{try_times}/{max_wait_try_times}"
                    )

                rospy.loginfo(
                    f'机器人距离: {self.disdance:.2f}m, hf_0状态: {self.global_status["hf_0"]["status"]}'
                )
                while (
                    self.global_status["hf_0"]["status"] == "running"
                    and self.disdance < conflict_distence
                ):
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"机器人距离: {self.disdance:.2f}<{conflict_distence}m, 等待hf_0停止运动"
                    )

            # 保存当前目标站点信息
            self.cur_station_name = dest_station

            if "charge" in self.cur_station_name:
                self.platform_status["cur_station"] = "charge_station"
            else:
                self.platform_status["cur_station"] = self.cur_station_name

            # 每次运行重新加载站点信息进行更新
            self.station_pose = load_station_pose_json(self.station_pose_path)

            # 插入报错反馈话题
            if dest_station not in self.station_pose:
                rospy.logerr("%s dose not exist", dest_station)
                self.platform_status.update(
                    {
                        "state": "idle",
                        "stamp": rospy.Time.now().to_time(),
                        "detail": dest_station + "站点未标定",
                    }
                )
                self.loop_rate.sleep()
                self.pub_dms_msg.publish(String(data=json.dumps(self.platform_status)))
                return

            # 以 move_base 格式保存当前目标站点位姿
            self.cur_goal.target_pose.header.frame_id = "map"
            self.cur_goal.target_pose.header.stamp = rospy.Time.now()

            station = self.station_pose[dest_station]
            self.cur_goal.target_pose.pose.position.x = station["x"]
            self.cur_goal.target_pose.pose.position.y = station["y"]
            self.cur_goal.target_pose.pose.orientation.z = station["z"]
            self.cur_goal.target_pose.pose.orientation.w = station["w"]

            self.platform_status["state"] = "running"
            self.platform_status["detail"] = "底盘前往" + dest_station

            # 等待1s用于amcl_pose回调函数修改底盘running状态
            rospy.sleep(rospy.Duration(1.0))

            # 每次接收到新目标地点，进行微调操作
            # 先发给mapf后端，给mapf计算时间
            self.send_goal_to_flask()

            # 微调前先中止移动状态，使运动线程停止
            # with self.state_move_lock:
            self.state_move = False

            # 停止move_base运动线程
            rospy.loginfo("停止move_base运动线程...")
            self.ac.cancel_goal()
            rospy.sleep(0.2)

            # 先后退0.2m
            rospy.loginfo("调整姿态以方便去下一个工作站...")

            offset = stroll_offsets.get(self.last_station_name, (0, 0.3))
            self.stroll_to_back(offset[0], offset[1])

            rospy.loginfo("横移完毕, 原地旋转为终点姿态")

            # 停止之前的move_base运动线程
            self.ac.cancel_goal()
            # 原地旋转为终点姿态
            (_, _, yaw) = euler_from_quaternion(
                (
                    self.cur_goal.target_pose.pose.orientation.x,
                    self.cur_goal.target_pose.pose.orientation.y,
                    self.cur_goal.target_pose.pose.orientation.z,
                    self.cur_goal.target_pose.pose.orientation.w,
                )
            )
            self.rotate_in_place(yaw)
            rospy.loginfo("原地旋转完毕")
            self.ac.cancel_goal()

            # 设置为移动状态
            with self.state_move_lock:
                self.state_move = True

            self.last_station_name = self.cur_station_name
        # 解除充电
        elif action == "uncharge":
            if not self.recharge_flag:
                self.platform_status.update({"state": "done", "detail": "已解除充电"})
                self.pub_dms_msg.publish(String(data=json.dumps(self.platform_status)))
                self.platform_status["state"] = "idle"
                return

            # robot_0准备移动时，如果robot_0在充电站而robot_1在running状态
            # robot_0必须等robot_1在idle状态才能动
            if (
                self.robot_name == "hf_0"
                and self.cur_station_name == self.charge_station
            ):
                try_times = 0
                while (
                    not self.global_status.get("hf_1")
                    or abs(
                        self.global_status["hf_1"]["stamp"]
                        - self.global_status["hf_0"]["stamp"]
                    )
                    > 1.0
                    or abs(
                        rospy.Time.now().to_time() - self.global_status["hf_0"]["stamp"]
                    )
                    > 2.0
                ) and try_times < max_wait_try_times:
                    try_times += 1
                    rospy.sleep(1.0)
                    rospy.loginfo(
                        f"等待两台机器人状态数据同步更新，尝试次数{try_times}/{max_wait_try_times}"
                    )
                rospy.loginfo(
                    f'机器人距离: {self.disdance:.2f}m, hf_1状态: {self.global_status["hf_1"]["status"]}'
                )
                while self.global_status["hf_1"]["status"] == "running":
                    rospy.sleep(1.0)
                    rospy.loginfo("当前机器人在充电站, 等待hf_1停止运动")

            #  向左平移0.15米解除充电
            if self.last_station_name == "charge_station_hf_0":
                self.stroll_to_back(0, 0.05)
            elif self.last_station_name == "charge_station_hf_1":
                self.stroll_to_back(0, 0.2)

            self.recharge_flag = False

            self.platform_status.update({"state": "done", "detail": "已解除充电"})
            self.loop_rate.sleep()
            self.pub_dms_msg.publish(String(data=json.dumps(self.platform_status)))
            self.platform_status["state"] = "idle"

        else:
            pass

        return

    def rotate_in_place(self, yaw):
        tmp_goal = MoveBaseGoal()
        tmp_goal.target_pose.header.frame_id = "map"
        tmp_goal.target_pose.header.stamp = rospy.Time.now()
        tmp_goal.target_pose.pose.position = self.cur_location.pose.pose.position
        tmp_goal.target_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, yaw)
        )

        while not rospy.is_shutdown():
            # 计算当前机器人方向与目标方向的差异
            curr_goal_orientate_difference = math.fabs(self.cur_yaw - yaw)

            # 如果差异小于 5° 或大于 355°，则停止发送命令
            if (
                curr_goal_orientate_difference < 5 * math.pi / 180
                or curr_goal_orientate_difference > 355 * math.pi / 180
            ):
                break

            # 发送移动命令并等待结果
            self.ac.send_goal(tmp_goal)
            self.ac.wait_for_result()
            rospy.loginfo("Wait for inplace-rotate")
            if self.ac.get_state() == actionlib.GoalStatus.ABORTED:
                rospy.logwarn("原地旋转 STATE: Reach Goal ABORTED, Resend Goal")
                self.ac.cancel_goal()
            elif self.ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
                break

    # 先发送全局目标位置给flask后端，然后一步步接受move_base的目标点
    def send_goal_to_flask(self):
        # 将目标位置发送给flask服务器
        rospy.loginfo("目标位置: {} 发送给flask服务器".format(self.cur_station_name))
        # rospy.loginfo("Send Pose to flask_core")
        response = None
        while response is None:
            self.loop_rate.sleep()
            try:
                response = self.session.put(
                    self.host_ip + "/mapf_global_goal",
                    headers={"Content-Type": "text/plain"},
                    data=self.cur_station_name,
                )
                response = response.text
            except Exception as e:
                rospy.logwarn(e)
        rospy.loginfo("发送成功，接到返回消息...")

    # 用于往hf_base_link的x轴和y轴方向移动distance距离，由于机械臂在站点一侧
    # 往y轴方向偏移看起来就是"后退"(远离站点)
    def stroll_to_back(self, x_distance, y_distance):
        # 相对于base_link的偏移目标点
        pose_transform = tf2_geometry_msgs.PoseStamped()
        pose_transform.header.frame_id = self.base_link_frame
        pose_transform.pose.position.x = x_distance
        pose_transform.pose.position.y = y_distance
        pose_transform.pose.position.z = 0
        pose_transform.pose.orientation.x = 0
        pose_transform.pose.orientation.y = 0
        pose_transform.pose.orientation.z = 0
        pose_transform.pose.orientation.w = 1

        # 将该点转化到map下
        pose_transform_map = tf2_geometry_msgs.PoseStamped()

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        transform_flag = False

        while not transform_flag:
            try:
                # 尝试进行坐标系变换
                pose_transform_map = buffer.transform(
                    pose_transform, "map", rospy.Duration(0)
                )
                transform_flag = True
            except tf2_ros.TransformException as e:
                # 如果出现异常，打印信息并继续等待
                rospy.loginfo("wait tf buffer listener.....")
                rospy.sleep(1.0)

        self.stroll_a_little(pose_transform_map)

    # 往pose_transform_map点挪动
    def stroll_a_little(self, pose_transform_map):
        tmp_goal = MoveBaseGoal()
        tmp_goal.target_pose.header.frame_id = "map"
        tmp_goal.target_pose.header.stamp = rospy.Time.now()
        tmp_goal.target_pose.pose.position.x = pose_transform_map.pose.position.x
        tmp_goal.target_pose.pose.position.y = pose_transform_map.pose.position.y
        tmp_goal.target_pose.pose.position.z = pose_transform_map.pose.position.z
        tmp_goal.target_pose.pose.orientation = pose_transform_map.pose.orientation

        while not rospy.is_shutdown():
            self.ac.send_goal(tmp_goal)
            self.ac.wait_for_result()
            if self.ac.get_state() == actionlib.GoalStatus.ABORTED:
                rospy.logwarn("挪动 STATE: Reach Goal ABORTED, Resend Goal")
                self.ac.cancel_goal()
            elif self.ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
                break

    # 正常请求终点的goal线程：self.state_move == True
    # 一步步接受move_base的目标点
    # 如果move_base state为success并且到达全局(非局部)目标点，发布到达消息
    def run_to_goal(self):
        while not rospy.is_shutdown():
            with self.state_move_lock:
                if self.state_move == True:
                    try:
                        response = self.session.get(self.host_ip + "/mapf_goal")
                    except:
                        # rospy.logwarn("FUNC run to goal: 无法连接flask_core服务器")
                        continue

                    robot_goal = json.loads(response.text)
                    # 如果当前有goal, 发布move_base_goal
                    if robot_goal:
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = robot_goal["position"]["x"]
                        goal.target_pose.pose.position.y = robot_goal["position"]["y"]
                        goal.target_pose.pose.position.z = robot_goal["position"]["z"]
                        goal.target_pose.pose.orientation.x = robot_goal["orientation"][
                            "x"
                        ]
                        goal.target_pose.pose.orientation.y = robot_goal["orientation"][
                            "y"
                        ]
                        goal.target_pose.pose.orientation.z = robot_goal["orientation"][
                            "z"
                        ]
                        goal.target_pose.pose.orientation.w = robot_goal["orientation"][
                            "w"
                        ]

                        self.ac.send_goal(goal)
                        self.ac.wait_for_result(rospy.Duration(1.0))
                        rospy.loginfo("%s发送step目标位置", self.robot_name)
                        rospy.loginfo(
                            "当前move_base状态: %s",
                            mb_action_status_dict[self.ac.get_state()],
                        )

                        dis_to_goal = (
                            self.cur_goal.target_pose.pose.position.x
                            - self.cur_location.pose.pose.position.x
                        ) ** 2 + (
                            self.cur_goal.target_pose.pose.position.y
                            - self.cur_location.pose.pose.position.y
                        ) ** 2
                        dis_to_goal = math.sqrt(dis_to_goal)

                        if (
                            dis_to_goal > 0.4
                            and self.ac.get_state() != actionlib.GoalStatus.ACTIVE
                        ):
                            rospy.loginfo(
                                "%s任务规划挂起, 临时进入直接规划模式, 当前离目标点距离: %.2f",
                                self.robot_name,
                                dis_to_goal,
                            )
                            self.ac.cancel_goal()
                            rospy.sleep(0.5)
                            final_goal = MoveBaseGoal()
                            final_goal.target_pose.header.frame_id = "map"
                            final_goal.target_pose.header.stamp = rospy.Time.now()
                            final_goal.target_pose.pose.position = (
                                self.cur_goal.target_pose.pose.position
                            )
                            final_goal.target_pose.pose.orientation = (
                                self.cur_goal.target_pose.pose.orientation
                            )
                            self.ac.send_goal(final_goal)
                            self.ac.wait_for_result(rospy.Duration(2))

                    # 检查是否到达终点
                    dis_to_goal = (
                        self.cur_goal.target_pose.pose.position.x
                        - self.cur_location.pose.pose.position.x
                    ) ** 2 + (
                        self.cur_goal.target_pose.pose.position.y
                        - self.cur_location.pose.pose.position.y
                    ) ** 2
                    dis_to_goal = math.sqrt(dis_to_goal)

                    # rospy.loginfo(self.ac.get_state())

                    if (
                        self.ac.get_state() == actionlib.GoalStatus.SUCCEEDED
                        and dis_to_goal <= 0.2
                    ):
                        rospy.loginfo(
                            f"{self.robot_name}: \033[1;32m Reach Goal SUCCEEDED\033[0m"
                        )
                        self.state_move = False

                        # 如果不是充电站
                        if "charge_station" not in self.cur_station_name:
                            self.platform_status.update(
                                {
                                    "state": "done",
                                    "detail": "底盘已运动到 " + self.cur_station_name,
                                }
                            )
                            # 主动发布话题
                            self.pub_dms_msg.publish(
                                String(data=json.dumps(self.platform_status))
                            )
                            self.platform_status["state"] = "idle"
                        elif self.last_action == "charge":
                            # 如果是充电站，到达后进行微调
                            rospy.loginfo("该站为充电站，到达后进行微调:")
                            self.loop_rate.sleep()
                            # 微调：x, y方向一起平移
                            rospy.loginfo("微调: x, y方向一起平移")
                            for i in range(1, 6):
                                flag_get_camera_tran, trans_map = self.get_camera_tran(
                                    i / 5.0
                                )
                                while not flag_get_camera_tran:
                                    rospy.loginfo("获取相机位置失败,每隔2s重新检测...")
                                    rospy.sleep(2)
                                    flag_get_camera_tran, trans_map = (
                                        self.get_camera_tran(i / 5.0)
                                    )
                                trans_map.pose.position.z = 0
                                self.stroll_a_little(trans_map)
                                self.loop_rate.sleep()

                            with self.recharge_lock:
                                self.recharge_flag = True

                            # 发布充电命令
                            charging_thread = threading.Thread(target=self.recharging)
                            charging_thread.start()
                            self.loop_rate.sleep()

                            self.platform_status.update(
                                {"state": "done", "detail": "底盘已运动到充电桩"}
                            )
                            self.pub_dms_msg.publish(
                                String(data=json.dumps(self.platform_status))
                            )
                            self.platform_status["state"] = "charging"

            rospy.sleep(0.5)

    # 地点冲突时躲避到rest_station的线程
    def run_to_rest(self):
        reach_charge_station = True
        while not rospy.is_shutdown():
            with self.state_move_lock:
                if self.state_move == False:
                    try:
                        response = self.session.get(self.host_ip + "/mapf_goal")
                        if response.status_code == 502:
                            raise requests.exceptions.HTTPError(f"Received 502 Bad Gateway: {response.text}")
                    except requests.exceptions.RequestException as e:
                        # rospy.logwarn(f"FUNC run_to_rest: 无法连接flask_core服务器, 错误: {e}")
                        continue
                    
                    robot_goal = json.loads(response.text)
                    # 如果最终goal为"rest_station", 发布move_base_goal
                    if (
                        robot_goal
                        and robot_goal["name"] == "charge_station_" + self.robot_name
                        and self.last_action == "move"
                        and self.state_move == False
                        and self.init_flag
                        == True  # 必须要初始化后(接收到第一帧指令后)才能开始避让
                    ):
                        # 如果上一个站名称不是休息工作站，说明准备去休息工作站
                        # 出发前进行微调
                        if (
                            self.last_station_name
                            != "charge_station_" + self.robot_name
                        ):
                            # 进入避让状态
                            self.avoid = True
                            # 停止之前的move_base运动线程
                            self.ac.cancel_goal()
                            rospy.loginfo("前往充电站前, 调整姿态")
                            rospy.loginfo(
                                "\033[0;33m进入避让状态\033[0m"
                            )
                            rospy.loginfo(
                                f"\033[1;32m做任何操作前机械臂复位\033[0m, 复位工作站{self.last_station_name}"
                            )
                            if "charge_station" in self.last_station_name:
                                self.oper_reset_msg["destination"] = "charge_station"
                            else:
                                self.oper_reset_msg["destination"] = (
                                    self.last_station_name
                                )
                            self.oper_pub.publish(
                                String(json.dumps(self.oper_reset_msg))
                            )

                            # 当前工作站改为充电站
                            self.platform_status["cur_station"] = "charge_station"
                            self.platform_status["state"] = "running"
                            self.platform_status["detail"] = (
                                "底盘前往" + "charge_station"
                            )

                            offset = stroll_offsets.get(
                                self.last_station_name, (0, 0.3)
                            )
                            self.stroll_to_back(offset[0], offset[1])

                            rospy.loginfo("横移完毕, 原地旋转为充电站姿态")
                            charge_station_pose = self.station_pose[
                                "charge_station_" + self.robot_name
                            ]

                            (_, _, yaw) = euler_from_quaternion(
                                (
                                    0.0,
                                    0.0,
                                    charge_station_pose["z"],
                                    charge_station_pose["w"],
                                )
                            )

                            self.rotate_in_place(yaw)
                            rospy.loginfo("原地旋转完毕")
                            # 将上一个工作站置为充电站，以防二次挪动
                            self.last_station_name = "charge_station_" + self.robot_name

                            reach_charge_station = False
                            # 跳到下次循环，更新goal
                            continue

                        if (
                            self.last_action != "charge"
                            and reach_charge_station == False
                        ):
                            # 检查是否到达充电站
                            charge_station_pose = self.station_pose[
                                "charge_station_" + self.robot_name
                            ]
                            dis_to_goal = (
                                charge_station_pose["x"]
                                - self.cur_location.pose.pose.position.x
                            ) ** 2 + (
                                charge_station_pose["y"]
                                - self.cur_location.pose.pose.position.y
                            ) ** 2
                            dis_to_goal = math.sqrt(dis_to_goal)

                            if (
                                self.ac.get_state() == actionlib.GoalStatus.SUCCEEDED
                                and dis_to_goal < 0.2
                            ):
                                rospy.loginfo("已到充电站")
                                self.ac.cancel_goal()
                                reach_charge_station = True
                                # 退出避让模式
                                self.avoid = False

                                # self.platform_status.update(
                                #     {"state": "done", "detail": "底盘已运动到充电站"}
                                # )
                                # self.pub_dms_msg.publish(
                                #     String(data=json.dumps(self.platform_status))
                                # )
                                self.platform_status["state"] = "idle"
                            else:
                                rospy.loginfo("前往充电站")
                                goal = MoveBaseGoal()
                                goal.target_pose.header.frame_id = "map"
                                goal.target_pose.header.stamp = rospy.Time.now()
                                goal.target_pose.pose.position.x = robot_goal[
                                    "position"
                                ]["x"]
                                goal.target_pose.pose.position.y = robot_goal[
                                    "position"
                                ]["y"]
                                goal.target_pose.pose.position.z = robot_goal[
                                    "position"
                                ]["z"]
                                goal.target_pose.pose.orientation.x = robot_goal[
                                    "orientation"
                                ]["x"]
                                goal.target_pose.pose.orientation.y = robot_goal[
                                    "orientation"
                                ]["y"]
                                goal.target_pose.pose.orientation.z = robot_goal[
                                    "orientation"
                                ]["z"]
                                goal.target_pose.pose.orientation.w = robot_goal[
                                    "orientation"
                                ]["w"]

                                self.ac.send_goal(goal)
                                self.ac.wait_for_result(rospy.Duration(1.0))
                                rospy.loginfo(
                                    "%s前往充电站休息,发送step目标位置", self.robot_name
                                )
                                rospy.loginfo(
                                    "当前move_base状态: %s",
                                    mb_action_status_dict[self.ac.get_state()],
                                )

                                if (
                                    dis_to_goal > 0.4
                                    and self.ac.get_state()
                                    != actionlib.GoalStatus.ACTIVE
                                ):
                                    rospy.loginfo(
                                        "%s任务规划挂起, 临时进入直接规划模式, 当前离目标点距离: %.2f",
                                        self.robot_name,
                                        dis_to_goal,
                                    )
                                    self.ac.cancel_goal()
                                    rospy.sleep(0.5)
                                    final_goal = MoveBaseGoal()
                                    final_goal.target_pose.header.frame_id = "map"
                                    final_goal.target_pose.header.stamp = (
                                        rospy.Time.now()
                                    )

                                    final_goal.target_pose.pose.position.x = (
                                        charge_station_pose["x"]
                                    )
                                    final_goal.target_pose.pose.position.y = (
                                        charge_station_pose["y"]
                                    )

                                    final_goal.target_pose.pose.orientation.z = (
                                        charge_station_pose["z"]
                                    )
                                    final_goal.target_pose.pose.orientation.w = (
                                        charge_station_pose["w"]
                                    )

                                    self.ac.send_goal(final_goal)
                                    self.ac.wait_for_result(rospy.Duration(2))

            rospy.sleep(0.5)

    # 持续充电线程
    def recharging(self):
        rospy.loginfo("\033[1;32m start charge...\033[0m")
        v = Twist()

        pub_crg_msg = rospy.Publisher("/hf_platform/crg_vel", Twist, queue_size=10)
        rospy.sleep(2)
        is_full = False
        old_quantity = 0
        new_quantity = None
        charge_success = True
        while self.recharge_flag:
            platform_info = rospy.wait_for_message(
                "/hf_platform/platform_info", platformInfo
            )
            new_quantity = int(platform_info.batteryPower)
            rospy.loginfo("quantity of electricity is %.3f", platform_info.batteryPower)
            if platform_info.batteryPower >= 100:
                v.angular.x = 0
                is_full = True
                rospy.loginfo(
                    "\033[1;32m 电池电量大于100，设置状态为充满，停止充电\033[0m"
                )

            if not is_full:
                v.angular.x = 1
                rospy.loginfo("\033[1;32m 状态为未充满，继续充电\033[0m")
                charge_success = new_quantity - old_quantity >= 0
                old_quantity = new_quantity
            elif platform_info.batteryPower <= 90:
                v.angular.x = 1
                is_full = False
                rospy.loginfo(
                    "\033[1;32m 电池电量低于90，设置状态为未充满，继续充电\033[0m"
                )
                old_quantity = new_quantity
            else:
                v.angular.x = 0
                rospy.loginfo("\033[1;32m 电池电量大于100，状态充满，停止充电\033[0m")

            pub_crg_msg.publish(v)
            rospy.sleep(2)

        rospy.loginfo("\033[1;32m end charge...\033[0m")

    def dms_inquiry_service(self, req):
        self.platform_status["stamp"] = rospy.Time.now().to_time()
        time_diff = rospy.Time.now() - self.amcl_pose_time
        if time_diff.to_sec() > 30:
            platform_status_copy = copy.deepcopy(self.platform_status)
            platform_status_copy["state"] = "idle"
            platform_status_copy["detail"] = "位姿数据超时"
            return json.dumps(platform_status_copy, indent=2)

        return json.dumps(self.platform_status, indent=2)

    def mqtt_on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected MQTT with result code {rc}")
        client.subscribe("/mapf_status", qos=1)
    
    def mqtt_on_message(self, client, userdata, msg):
        try:
            # 解码消息内容，并解析为JSON
            # print(msg.payload.decode("utf-8"))
            self.global_status = json.loads(msg.payload.decode("utf-8"))
            x0, y0 = (
                self.global_status["hf_0"]["position"]["x"],
                self.global_status["hf_0"]["position"]["y"],
            )
            x1, y1 = (
                self.global_status["hf_1"]["position"]["x"],
                self.global_status["hf_1"]["position"]["y"],
            )
            self.disdance = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

            self.pub_global_status_msg.publish(String(json.dumps(self.global_status)))
            
            # pprint(self.global_status)
            # pprint(self.global_status["hf_0"]["stamp"])
            # pprint(self.global_status["hf_1"]["stamp"])
        except Exception as e:
            # print(f"Error processing message: {e}")
            pass


def main():
    # 清除 ROS 特有的命令行参数
    argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(
        description="Platform Communication Node Parameters"
    )
    parser.add_argument("-aruco_id", type=str, default="1", help="ArUco ID")
    parser.add_argument(
        "-station_pose_path", type=str, required=True, help="Station pose file path"
    )
    parser.add_argument(
        "-camera_pose_path", type=str, required=True, help="Camera pose file path"
    )

    args = parser.parse_args(argv[1:])

    # 初始化 ROS 节点
    rospy.init_node("platform_communication_node")
    robot_name = rospy.get_param("~robot_name", default="hf_0")
    pose_topic = rospy.get_param("~pose_topic", default="/amcl_pose_tf")
    host_ip = rospy.get_param("~host_ip", default="192.168.1.170")
    debug = rospy.get_param("~debug", default="false")

    use_prefix = True if debug == True else False

    pc = PlatformCommunication(
        host_ip=host_ip,
        robot_name=robot_name,
        pose_topic=pose_topic,
        station_pose_path=args.station_pose_path,
        camera_pose_path=args.camera_pose_path,
        aruco_id=args.aruco_id,
        use_prefix=use_prefix,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
