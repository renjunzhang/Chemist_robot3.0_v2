cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

add_action_files(
  FILES
  NavigateTask.action
)

add_message_files(
  FILES
  NavigationStatus.msg
  PathTrackingState.msg
  BehaviorState.msg
)

add_service_files(
  FILES
  ReloadStationPoses.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    nav_msgs
    std_msgs
)
