#!/bin/zsh

# Check if the robot_name parameter is provided
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Error: Missing robot_name"
    echo "Usage: $0 <robot_name> <aruco_id>"
    exit 1
fi

#---------------------------- platform ---------------------------------------
terminator  --new-tab -x "source ~/speed_develop/chemist_robot3.0/devel/setup.zsh;rosclean purge -y;roslaunch hf_bringup hf_bringup.launch robot_name:=$1;exec zsh;"
sleep 2s

terminator  --new-tab -x "source ~/speed_develop/chemist_robot3.0/devel/setup.zsh;roslaunch hf_navigation hf_navigation.launch;exec zsh;"
sleep 2s

terminator  --new-tab -x "source ~/speed_develop/chemist_robot3.0/devel/setup.zsh;roslaunch task_communication task_communication.launch robot_name:=$1 pose_topic:=/amcl_pose_tf host_ip:=192.168.31.58 aruco_id:=$2;exec zsh;"
sleep 1s
#--------------------------- platform END ------------------------------------


terminator  --new-tab -x "source ~/.zshrc;roslaunch locator cv_locator.launch;exec zsh;"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch web_video_server web_video_server.launch;exec zsh"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch chemical_arm_driver chemical_arm_driver.launch;exec zsh"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch record_rtde record_rtde.launch;exec zsh"
sleep 1s


#------------------------ robot message bridge ---------------------------------
terminator  --new-tab -x "source ~/.zshrc;roslaunch robot_message_bridge robot_message_bridge.launch;exec zsh;"
sleep 1s
