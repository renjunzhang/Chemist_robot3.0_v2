cmake_minimum_required(VERSION 3.0.2)
project(ridgeback_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ridgeback_tmp
 CATKIN_DEPENDS pluginlib roscpp rospy sensor_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_odom_correct_node src/odom_to_odom_correct.cpp)

target_link_libraries(odom_to_odom_correct_node
  ${catkin_LIBRARIES}
)

add_executable(laser_frame_correct_node src/laser_frame_correct.cpp)

target_link_libraries(laser_frame_correct_node
  ${catkin_LIBRARIES}
)

add_executable(lidar_odometry_rg_node src/lidar_odometry_rg.cpp)
target_link_libraries(lidar_odometry_rg_node
  ${catkin_LIBRARIES}
)

add_executable(lidar_odometry_rg_single_node src/lidar_odometry_rg_single.cpp)
target_link_libraries(lidar_odometry_rg_single_node
  ${catkin_LIBRARIES}
)

