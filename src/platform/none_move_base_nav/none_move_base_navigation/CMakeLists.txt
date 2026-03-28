cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_navigation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  none_move_base_common
  none_move_base_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    nav_msgs
    none_move_base_common
    none_move_base_msgs
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigator_server_node
  src/navigator_server_node.cpp
)

add_dependencies(navigator_server_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigator_server_node
  ${catkin_LIBRARIES}
)
