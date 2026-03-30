cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_local_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  none_move_base_common
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    none_move_base_common
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_dcbf_planner_node
  src/mpc_dcbf_planner_node.cpp
)

add_dependencies(mpc_dcbf_planner_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_dcbf_planner_node
  ${catkin_LIBRARIES}
)
