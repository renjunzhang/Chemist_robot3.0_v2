cmake_minimum_required(VERSION 3.0.2)
project(none_move_base_global)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  none_move_base_common
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    none_move_base_common
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/astar_planner.cpp
  src/grid_adapter.cpp
  src/path_postprocessor.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(global_planner_node
  src/global_planner_node.cpp
)

add_dependencies(global_planner_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(global_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
