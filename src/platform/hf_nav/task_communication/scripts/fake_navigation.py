#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

# 你原来的两个站点
stationsList = [
    "starting_station",
    "ending_station",
]

done_flag = False


def feedback_callback(msg):
    global done_flag
    print("=============rev msg===============")
    print(msg.data)
    print("============================")
    done_flag = True


def command_pub(pub, destination):
    """
    和你原来的接口完全一致，只是把 input 去掉，
    destination 在外部决定（自动切换）。
    """
    global done_flag
    done_flag = False

    obsNavigation_in = {
        "id": "0001",
        "exper_no": "1",
        "stamp": str(int(time.time())),
        "destination": destination,
        "action": "move",
    }

    navigation_in = String(
        json.dumps(
            obsNavigation_in,
            sort_keys=True,
            indent=4,
            separators=(",", ": ")
        )
    )

    print("=============pub msg===============")
    print(navigation_in.data)
    print("============================")

    pub.publish(navigation_in)


def spin_robot(cmd_vel_pub, spins=2, angular_velocity=0.2):
    """
    通过发送 Twist 消息让机器人自转
    
    Args:
        cmd_vel_pub: /cmd_vel 的发布者
        spins: 转圈数
        angular_velocity: 角速度（rad/s），正值为逆时针
    """
    # 两圈的总弧度
    total_angle = spins * 2 * math.pi
    # 根据角速度计算需要的时间
    duration = total_angle / angular_velocity
    
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_velocity
    
    print(f"开始自转 {spins} 圈，持续 {duration:.2f} 秒...")
    
    start_time = time.time()
    while time.time() - start_time < duration and not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)  # 20Hz 发布频率
    
    # 发送停止命令
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    time.sleep(0.5)
    
    print("自转完成")


def main():
    rospy.init_node("auto_navigation_in", anonymous=True)

    # 和你原来一样的参数获取方式
    robot_name = rospy.get_param("~robot_name", default="")
    pub = rospy.Publisher(robot_name + "/obsNavigation_in", String, queue_size=10)
    
    # 发布 cmd_vel 用于自转控制
    cmd_vel_pub = rospy.Publisher(robot_name + "/cmd_vel", Twist, queue_size=10)

    # 与你原程序一致的多个反馈 topic
    rospy.Subscriber("tb_0/obsNavigation_out", String, feedback_callback)
    rospy.Subscriber("tb_1/obsNavigation_out", String, feedback_callback)
    rospy.Subscriber("hf_0/obsNavigation_out", String, feedback_callback)
    rospy.Subscriber("hf_1/obsNavigation_out", String, feedback_callback)

    print("开始自动来回运行（约10秒一趟，持续5小时以上）")

    index = 0  # 0=start 1=end

    while not rospy.is_shutdown():

        destination = stationsList[index]
        command_pub(pub, destination)

        # 等待机器人反馈（到达站点）
        timeout = 0
        while not done_flag and timeout < 30 and not rospy.is_shutdown():
            time.sleep(0.1)
            timeout += 0.1

        # 到达后自转2圈
        spin_robot(cmd_vel_pub, spins=2, angular_velocity=1.0)

        # 两站之间行驶约10秒
        time.sleep(5)
    
        # 切换到另一个站点
        index = 1 - index


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass