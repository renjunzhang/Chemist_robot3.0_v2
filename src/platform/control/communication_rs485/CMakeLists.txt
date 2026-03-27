cmake_minimum_required(VERSION 3.0.2)
project(communication_rs485)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages

# # noetic serial 是自己装的，没有集成 ros
set(serial_DIR "/usr/local/share/serial/cmake")

find_package(catkin REQUIRED COMPONENTS
    serial
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf
)

# find_package(OpenCV REQUIRED)
# set(CMAKE_PREFIX_PATH "/usr/local/share/serial/cmake")
# find_package(serial REQUIRED)
add_message_files(FILES
    platformInfo.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include

    # LIBRARIES communication_rs485
    CATKIN_DEPENDS
    serial
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf

    # DEPENDS system_lib
)

include_directories(
    include
    "/usr/local/include"
    include/communication_rs485
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(rs485_control src/rs485_control.cpp)
add_dependencies(rs485_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(hf_utils src/hf_utils.cpp)
add_dependencies(hf_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hf_utils
    ${catkin_LIBRARIES}
)

target_link_libraries(rs485_control
    hf_utils
    ${catkin_LIBRARIES}
)

add_executable(rs485_control_node src/rs485_control_node.cpp)

target_link_libraries(rs485_control_node
    ${catkin_LIBRARIES}
    rs485_control
    hf_utils
)

add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
add_dependencies(joy_to_twist_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(joy_to_twist_node
    ${catkin_LIBRARIES}
)

add_executable(platform_monitor_node src/platform_monitor_node.cpp)
add_dependencies(platform_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(platform_monitor_node
    ${catkin_LIBRARIES}
)

add_executable(multi_platform_monitor_node src/multi_platform_monitor_node.cpp)
add_dependencies(multi_platform_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_platform_monitor_node
    ${catkin_LIBRARIES}
)
