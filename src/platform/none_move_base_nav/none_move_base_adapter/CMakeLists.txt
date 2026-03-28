cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_adapter)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  aichem_msg_srv
  none_move_base_common
  none_move_base_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    aichem_msg_srv
    none_move_base_common
    none_move_base_msgs
    roscpp
    std_msgs
)
