cmake_minimum_required(VERSION 3.10)
project(gz_truth)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
# Talk to Gazebo directly rather than going through ros_gz_bridge: one process,
# no message-type mapping, and the pose topic is all we need.
find_package(gz-transport12 REQUIRED)

add_executable(gz_truth_bridge src/gz_truth_bridge.cpp)
ament_target_dependencies(gz_truth_bridge rclcpp px4_msgs)
target_link_libraries(gz_truth_bridge gz-transport12::core)

install(TARGETS gz_truth_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
