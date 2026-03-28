cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_local)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  none_move_base_common
  none_move_base_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    none_move_base_common
    none_move_base_msgs
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/goal_checker.cpp
  src/obstacle_adapter.cpp
  src/path_tracker_controller.cpp
  src/velocity_limiter.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(local_controller_node
  src/local_controller_node.cpp
)

add_dependencies(local_controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(local_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
