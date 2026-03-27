#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import rospy
import json
from std_msgs.msg import String

done_flag = False

stationsList = [
    "starting_station",
    "infrared_spectrum",
    "libs",
    "high_flux_xrd_workstation",
    "high_flux_electrocatalysis_workstation",
    "raman_spectra",
    "muffle_station",
    "imbibition_workstation",
    "liquid_dispensing",
    "liquid_dispensing_1",
    "solid_dispensing",
    "solid_dispensing_1",
    "magnetic_stirring",
    "magnetic_stirring_1",
    "magnetic_stirring_2",
    "capping_station",
    "dryer_workstation",
    "dryer_workstation_1",
    "new_centrifugation",
    "new_centrifugation_screen",
    "photocatalysis_workstation",
    "photocatalysis_workstation_1",
    "uv_vis_charge",
    "uv_vis_uncharge",
    "flourescence",
    "gc",
    "storage_workstation",
    "multi_robots_exchange_workstation",
    "map_center",
    "charge_station_hf_0",
    "charge_station_hf_1",
    "manual_workstation",
    "ultrasonic_cleaner",
    "furnace_workstation",
    "spotting_workstation",
    "confecting_workstation",
]


def command_pub(robot_name):
    global done_flag
    rate = rospy.Rate(1)
    pub = rospy.Publisher(robot_name + "/obsNavigation_in", String, queue_size=10)

    # rotation_recovery
    while not rospy.is_shutdown():
        for index, item in enumerate(stationsList):
            print("%d:%s" % (index, item))

        char = input("\n输入工作站序号: ")
        nav_cmd = int(char) if char else 0
        destination = stationsList[nav_cmd]

        if destination == "uv_vis_charge":
            obsNavigation_in = {
                "id": "0001",
                "exper_no": "1",
                "stamp": "1212",
                "destination": "uv_vis",
                "action": "charge",
            }
        elif destination == "uv_vis_uncharge":
            obsNavigation_in = {
                "id": "0001",
                "exper_no": "1",
                "stamp": "1212",
                "destination": "uv_vis",
                "action": "uncharge",
            }
        else:
            obsNavigation_in = {
                "id": "0001",
                "exper_no": "1",
                "stamp": "1212",
                "destination": destination,
                "action": "move",
            }

        navigation_in = String(
            json.dumps(
                obsNavigation_in, sort_keys=True, indent=4, separators=(",", ": ")
            )
        )
        print("=============pub msg===============")
        print(navigation_in.data)
        print(
            json.dumps(
                obsNavigation_in, sort_keys=True, indent=4, separators=(",", ": ")
            )
        )
        time.sleep(1)
        print("============================")
        done_flag = False
        pub.publish(navigation_in)


def feedback_callback(msg):
    global done_flag
    print("=============rev msg===============")
    print(msg.data)
    print("============================")
    done_flag = True


if __name__ == "__main__":
    rospy.init_node("fake_navigation_in", anonymous=True)
    try:
        rospy.Subscriber("tb_0/obsNavigation_out", String, feedback_callback)
        rospy.Subscriber("tb_1/obsNavigation_out", String, feedback_callback)
        rospy.Subscriber("hf_0/obsNavigation_out", String, feedback_callback)
        rospy.Subscriber("hf_1/obsNavigation_out", String, feedback_callback)
        robot_name = rospy.get_param("~robot_name", default="")
        command_pub(robot_name)
    except rospy.ROSInterruptException:
        pass
    # rospy.spin()
