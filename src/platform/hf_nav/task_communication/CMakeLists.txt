cmake_minimum_required(VERSION 3.0.2)
project(task_communication)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib
  roslib
  tf
  tf_conversions
  eigen_conversions
  message_generation
  aichem_msg_srv
  communication_rs485
)
# add_service_files(
#         FILES
# )
generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_communication
  CATKIN_DEPENDS geometry_msgs message_filters roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(task_communication_node src/task_communication_node.cpp)
# target_link_libraries(task_communication_node ${catkin_LIBRARIES} libjsoncpp.a )

add_executable(hf_record_pose_node src/hf_record_pose_node.cpp)
target_link_libraries(hf_record_pose_node ${catkin_LIBRARIES} libjsoncpp.a glog gflags)

add_executable(platform_communication_node src/platform_communication.cpp)
add_dependencies(platform_communication_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(platform_communication_node ${catkin_LIBRARIES} libjsoncpp.a glog gflags )

