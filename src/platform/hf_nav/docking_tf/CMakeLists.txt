cmake_minimum_required(VERSION 2.8.3)
project(docking_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
   
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_odometry src/lidar_odometry.cpp)
target_link_libraries(lidar_odometry ${catkin_LIBRARIES})

add_executable(lidar_odometry_multi src/lidar_odometry_multi.cpp)
target_link_libraries(lidar_odometry_multi ${catkin_LIBRARIES})



