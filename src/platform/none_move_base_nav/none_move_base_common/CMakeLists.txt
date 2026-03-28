cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_common)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  none_move_base_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    none_move_base_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/frame_transform.cpp
  src/pose_utils.cpp
  src/reach_checker.cpp
  src/station_pose_loader.cpp
  src/status_codec.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
